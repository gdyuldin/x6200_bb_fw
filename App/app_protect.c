#include "app_protect.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_pa_temp_comp.h"
#include "app_power.h"
#include "app_ptt.h"
#include "app_radio.h"
#include "app_tx_monitor.h"
#include "board_config.h"
#include "bsp_board.h"
#include "bsp_i2c_bus.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "drv_adc_mon.h"
#include "drv_charger.h"
#include "drv_gauge.h"

#define LOG_TAG "app.protect"

/* Charge state last written to the charger; UNSYNCED = not yet written. */
typedef enum {
    APP_PROTECT_CHARGE_ALLOWED   = 0,
    APP_PROTECT_CHARGE_INHIBITED = 1,
    APP_PROTECT_CHARGE_UNSYNCED  = 2
} AppProtectChargeState;

static AppProtectSnapshot    s_snap;
static uint32_t              s_tick;
static uint8_t               s_fail_streak;
static AppProtectChargeState s_inhibit_written = APP_PROTECT_CHARGE_UNSYNCED;
static bool                  s_inhibit_desired;
static bool                  s_adapter_present_prev;
/* Host/NVPARAM charger-enable preference; false inhibits charging unless a
 * protection trip already does. Written from the main loop, read by the tick. */
static volatile bool         s_user_charge_enabled = true;
/* STICKY: PA over-temperature or invalid NTC reading blocks TX. */
static bool                  s_pa_tx_inhibit;
static bool                  s_cell_fallback_warned;
static uint16_t              s_pa_ntc_trip_code;
static uint16_t              s_pa_ntc_release_code;

/* Hysteresis latches, one per condition. */
static bool s_batt_temp_high;
static bool s_batt_temp_low;
static bool s_batt_temp_cold;
static bool s_cell_ov;
static bool s_cell_uv;
static bool s_pack_low;
static bool s_soc_low;
static bool s_dc_in_low;
static bool s_batt_warning;

static bool latch_high(bool state, int32_t value, int32_t trip,
                       int32_t release) {
    if (!state) {
        return value >= trip;
    }
    return value > release;
}

static bool latch_low(bool state, int32_t value, int32_t trip,
                      int32_t release) {
    if (!state) {
        return value <= trip;
    }
    return value < release;
}

/* Per-cell voltages; a pack without CellX wiring reports zeros, so fall back to
 * Batt/2 and warn once (pack-provisioning variance). */
static void per_cell_mv(int32_t *c1, int32_t *c2) {
    uint32_t a = s_snap.gauge.cell1_mv;
    uint32_t b = s_snap.gauge.cell2_mv;

    if (a == 0u && b == 0u) {
        a = s_snap.gauge.batt_mv / 2u;
        b = a;
        if (!s_cell_fallback_warned) {
            s_cell_fallback_warned = true;
            LOG_WARN("per-cell zero, using batt/2");
        }
    }
    *c1 = (int32_t)a;
    *c2 = (int32_t)b;
}

/* Charge-current policy: weak cell (s_cell_uv, min cell <= 3.0 V with the
 * 3.1 V release) first, then taper by max per-cell voltage; the same write
 * refreshes the bq24725 SMBus watchdog while charging is allowed. */
static uint32_t charge_current_policy(void) {
    int32_t c1;
    int32_t c2;
    int32_t cmax;

    if (s_cell_uv) {
        return BOARD_CHG_PRECHARGE_CURRENT_MA;
    }

    per_cell_mv(&c1, &c2);
    cmax = (c1 > c2) ? c1 : c2;
    if (cmax >= (int32_t)BOARD_CHG_TAPER2_CELL_MV) {
        return BOARD_CHG_TAPER2_CURRENT_MA;
    }
    if (cmax >= (int32_t)BOARD_CHG_TAPER1_CELL_MV) {
        return BOARD_CHG_TAPER1_CURRENT_MA;
    }
    return DRV_CHARGER_CHG_CURRENT_MA;
}

/* Charging is allowed only when the user preference enables it and no
 * protection condition forbids it. */
static bool charge_inhibit_now(void) {
    return !s_user_charge_enabled || !s_snap.data_valid ||
           s_batt_temp_high || s_batt_temp_low || s_cell_ov;
}

/* Push the desired charge state to the charger; retried on every burst when the
 * write did not take. While charging is allowed the current write doubles as
 * the watchdog refresh. */
static void apply_charge(void) {
    if (!s_snap.adapter_present) {
        /* Charger is unpowered without the adapter; skip I2C writes and force
         * a full re-sync once power returns. */
        s_inhibit_written = APP_PROTECT_CHARGE_UNSYNCED;
        return;
    }
    s_inhibit_desired = charge_inhibit_now();
    if (s_inhibit_desired) {
        if (s_inhibit_written != APP_PROTECT_CHARGE_INHIBITED &&
            Drv_Charger_InhibitCharge() == 0) {
            s_inhibit_written = APP_PROTECT_CHARGE_INHIBITED;
        }
        return;
    }
    (void)Drv_Charger_SetChargeCurrent(charge_current_policy());
    if (s_inhibit_written != APP_PROTECT_CHARGE_ALLOWED &&
        Drv_Charger_EnableCharge() == 0) {
        s_inhibit_written = APP_PROTECT_CHARGE_ALLOWED;
    }
}

/* Sample burst (0.5 s): ADC (vext + PA NTC) and the I2C2 gauge/charger words.
 * An ADC failure only invalidates the PA reading; it never touches the gauge
 * cache. The gauge/charger burst is atomic: any failed word leaves the old
 * cache in place and counts toward the unknown-data threshold. */
static void sample_burst(void) {
    DrvGaugeMeas     g = {0};
    DrvChargerStatus c = {0};
    uint16_t         raw = 0;
    int32_t          pa_mc = 0;
    int              rc;

    if (Drv_AdcMon_Sample() != 0) {
        s_snap.pa_valid = false;
    } else {
        (void)Drv_AdcMon_ReadVextMv(&s_snap.vext_mv);
        (void)Drv_AdcMon_ReadRaw(DRV_ADC_MON_VEXT, &s_snap.vext_raw);
        if (Drv_AdcMon_ReadRaw(DRV_ADC_MON_PA_NTC, &raw) == 0 &&
            Drv_AdcMon_ReadPaTemp(&pa_mc) == 0) {
            s_snap.pa_ntc_raw = raw;
            s_snap.pa_temp_mc = pa_mc;
            s_snap.pa_valid = true;
        } else {
            s_snap.pa_valid = false;
        }
    }

    s_snap.adapter_present = s_snap.vext_mv >= APP_PROTECT_DC_PRESENT_MV;
    if (s_snap.adapter_present && !s_adapter_present_prev) {
        /* Power restored: the BQ24725 has reset to its defaults, so run a
         * full bring-up again and force the next charge write. */
        (void)Drv_Charger_Init();
        s_inhibit_written = APP_PROTECT_CHARGE_UNSYNCED;
    }
    s_adapter_present_prev = s_snap.adapter_present;

    rc = Drv_Gauge_ReadMeasurements(&g);
    if (rc == 0) {
        if (s_snap.adapter_present) {
            rc = Drv_Charger_ReadStatus(&c);
        } else {
            /* Without the adapter the bq24725 may be unpowered and not answer
             * on I2C; that is expected, so keep the gauge data valid and skip
             * the charger read instead of counting an unknown-data failure. */
            c.option = 0u;
            c.ac_ok = false;
        }
    }
    if (rc == 0) {
        s_snap.gauge = g;
        s_snap.charger = c;
        s_snap.data_valid = true;
        s_fail_streak = 0u;
    } else if (s_fail_streak < 0xFFu) {
        s_fail_streak++;
        if (s_fail_streak >= BOARD_PROTECT_I2C_FAIL_MAX) {
            s_snap.data_valid = false;
        }
    }

    apply_charge();
}

/* ADC DMA health (100 Hz). The continuous DMA keeps transferring in hardware,
 * so a delayed completion ISR coalesces callbacks instead of losing transfers.
 * Compare completed scans against elapsed CPU cycles to count the scans the
 * monitor missed, and report ADC overruns and DMA errors. Log at most once a
 * second so a persistent fault cannot flood the ring. */
static void adc_dma_diagnostics(void) {
    static bool     started;
    static uint32_t cpu_hz;
    static uint32_t scan_cycles;
    static uint32_t cycles_prev;
    static uint32_t scans_prev;
    static uint32_t missed_total;
    static uint32_t missed_logged;
    static uint32_t overruns_logged;
    static uint32_t errors_logged;
    static uint32_t gap_max_cycles;
    static uint32_t log_tick;
    DrvAdcMonStats st;
    uint32_t now;
    uint32_t elapsed;
    uint32_t expected;
    uint32_t scans;
    bool     stalled;
    bool     gap_bad;

    Drv_AdcMon_TakeStats(&st);
    now = Bsp_Time_GetCycles();
    if (!started) {
        started = true;
        cpu_hz = Bsp_Time_GetCpuHz();
        scan_cycles =
            (uint32_t)(((uint64_t)cpu_hz * DRV_ADC_MON_SCAN_PERIOD_US) / 1000000u);
        cycles_prev = now;
        scans_prev = st.scans;
        return;
    }

    elapsed = now - cycles_prev;
    scans = st.scans - scans_prev;
    cycles_prev = now;
    scans_prev = st.scans;

    expected = (uint32_t)(((uint64_t)elapsed + scan_cycles / 2u) / scan_cycles);
    if (expected > scans) {
        missed_total += expected - scans;
    }
    if (st.max_gap_cycles > gap_max_cycles) {
        gap_max_cycles = st.max_gap_cycles;
    }
    stalled = scans == 0u;
    /* One scan is ~156 us. A gap over 1.5x that means the DMA completion ISR
     * was held long enough to swallow at least one further completion event. */
    gap_bad = gap_max_cycles > (scan_cycles + scan_cycles / 2u);

    if ((s_tick - log_tick) < 100u) {
        return;
    }
    {
        uint32_t d_ovr  = st.overruns - overruns_logged;
        uint32_t d_err  = st.dma_errors - errors_logged;
        uint32_t d_miss = missed_total - missed_logged;

        if (d_ovr != 0u || d_err != 0u || d_miss > 64u || gap_bad || stalled) {
            uint32_t gap_us =
                (uint32_t)(((uint64_t)gap_max_cycles * 1000000u) / cpu_hz);
            LOG_WARN("adc dma: ovr +%u miss %u err +%u gap %u us%s",
                     (unsigned)d_ovr, (unsigned)d_miss, (unsigned)d_err,
                     (unsigned)gap_us, stalled ? ", stalled" : "");
            overruns_logged = st.overruns;
            errors_logged = st.dma_errors;
            // TODO: maybe stop TX
        }
        missed_logged = missed_total;

        /* Report the peak of one logging interval, not a stale maximum. */
        gap_max_cycles = 0u;
    }
    log_tick = s_tick;
}

static const char *flag_name(uint32_t mask) {
    switch (mask) {
    case APP_PROTECT_FLAG_BATT_TEMP_HIGH: return "BATT_TEMP_HIGH";
    case APP_PROTECT_FLAG_BATT_TEMP_LOW:  return "BATT_TEMP_LOW";
    case APP_PROTECT_FLAG_BATT_TEMP_COLD: return "BATT_TEMP_COLD";
    case APP_PROTECT_FLAG_CELL_OV:        return "CELL_OV";
    case APP_PROTECT_FLAG_CELL_UV:        return "CELL_UV";
    case APP_PROTECT_FLAG_PACK_LOW:       return "PACK_LOW";
    case APP_PROTECT_FLAG_SOC_LOW:        return "SOC_LOW";
    case APP_PROTECT_FLAG_PA_TEMP_HIGH:   return "PA_TEMP_HIGH";
    case APP_PROTECT_FLAG_PA_FAULT:       return "PA_FAULT";
    case APP_PROTECT_FLAG_DC_IN_LOW:      return "DC_IN_LOW";
    case APP_PROTECT_FLAG_DATA_INVALID:   return "DATA_INVALID";
    case APP_PROTECT_FLAG_CHARGE_INHIBIT: return "CHARGE_INHIBIT";
    case APP_PROTECT_FLAG_TX_CUT:         return "TX_CUT";
    default:                              return "UNKNOWN";
    }
}

/* Threshold compares on the cached snapshot (100 Hz). Charge writes are edge
 * triggered; the flags mask is logged only when it changes. */
static void evaluate(void) {
    int32_t  c1;
    int32_t  c2;
    int32_t  cmax;
    int32_t  cmin;
    bool     pa_hot_or_fault;
    bool     inhibit;
    bool     cell_uv_prev = s_cell_uv;
    uint32_t flags;

    adc_dma_diagnostics();

    s_batt_temp_high = latch_high(s_batt_temp_high, s_snap.gauge.temp_mc,
                                  BOARD_BATT_TEMP_HIGH_MC,
                                  BOARD_BATT_TEMP_HIGH_RELEASE_MC);
    s_batt_temp_low = latch_low(s_batt_temp_low, s_snap.gauge.temp_mc,
                                BOARD_BATT_TEMP_LOW_MC,
                                BOARD_BATT_TEMP_LOW_RELEASE_MC);
    s_batt_temp_cold = latch_low(s_batt_temp_cold, s_snap.gauge.temp_mc,
                                 BOARD_BATT_TEMP_COLD_MC,
                                 BOARD_BATT_TEMP_COLD_RELEASE_MC);

    per_cell_mv(&c1, &c2);
    cmax = (c1 > c2) ? c1 : c2;
    cmin = (c1 < c2) ? c1 : c2;
    s_cell_ov = latch_high(s_cell_ov, cmax, BOARD_CELL_OV_MV,
                           BOARD_CELL_OV_RELEASE_MV);
    s_cell_uv = latch_low(s_cell_uv, cmin, BOARD_CELL_UV_MV,
                          BOARD_CELL_UV_RELEASE_MV);
    s_pack_low = latch_low(s_pack_low, (int32_t)s_snap.gauge.batt_mv,
                           BOARD_PACK_LOW_MV, BOARD_PACK_LOW_RELEASE_MV);
    s_soc_low = latch_low(s_soc_low, (int32_t)s_snap.gauge.soc_d_pct,
                          BOARD_SOC_LOW_D_PCT, BOARD_SOC_LOW_RELEASE_D_PCT);

    /* DC input: only an alarm when the adapter is present. Without an
     * adapter the flag stays clear, so running from battery is normal. */
    if (!s_dc_in_low) {
        s_dc_in_low = s_snap.adapter_present &&
                      s_snap.vext_mv <= BOARD_VEXT_LOW_MV;
    } else {
        s_dc_in_low = s_snap.adapter_present &&
                      s_snap.vext_mv < BOARD_VEXT_LOW_RELEASE_MV;
    }

    /* low-battery warning while running from the
     * pack (no adapter) — a critical battery state or unknown gauge data
     * there arms the auto-off countdown. Low battery on the adapter is
     * normal (charging), so it never warns. */
    s_batt_warning = !s_snap.adapter_present &&
                     (s_cell_uv || s_pack_low || s_soc_low ||
                      !s_snap.data_valid);

    /* PA TX-inhibit latch: trip on an invalid NTC reading or over-temperature.
     * The NTC raw code falls as the temperature rises, so the hot condition is
     * raw <= trip code and the release condition is raw >= release code. The
     * latch clears only on a valid reading at or below the release threshold. */
    pa_hot_or_fault = !s_snap.pa_valid ||
                      s_snap.pa_ntc_raw <= s_pa_ntc_trip_code;
    if (!s_pa_tx_inhibit) {
        s_pa_tx_inhibit = pa_hot_or_fault;
    } else {
        s_pa_tx_inhibit = !(s_snap.pa_valid &&
                            s_snap.pa_ntc_raw >= s_pa_ntc_release_code);
    }
    if (s_pa_tx_inhibit) {
        LOG_WARN("PA hot or temp fault, forcing rx");
        App_Ptt_ForceRx();
    }

    /* VSWR protection is a one-shot event: the DMA ISR only sets the flag,
     * the forced RX happens here in TIM6 context. */
    if (App_TxMonitor_TakeSwrTrip()) {
        LOG_WARN("vswr trip, forcing rx");
        App_Ptt_ForceRx();
    }

    /* Charge inhibit: user preference, unknown data or a temperature/
     * over-voltage trip. */
    inhibit = charge_inhibit_now();
    if (inhibit != s_inhibit_desired) {
        s_inhibit_desired = inhibit;
        apply_charge();
    } else if (s_cell_uv != cell_uv_prev && !s_inhibit_desired) {
        /* Weak-cell edge: switch precharge <-> normal current without waiting
         * for the next 0.5 s burst. */
        apply_charge();
    }

    flags = 0u;
    if (s_batt_temp_high) {
        flags |= APP_PROTECT_FLAG_BATT_TEMP_HIGH;
    }
    if (s_batt_temp_low) {
        flags |= APP_PROTECT_FLAG_BATT_TEMP_LOW;
    }
    if (s_batt_temp_cold) {
        flags |= APP_PROTECT_FLAG_BATT_TEMP_COLD;
    }
    if (s_cell_ov) {
        flags |= APP_PROTECT_FLAG_CELL_OV;
    }
    if (s_cell_uv) {
        flags |= APP_PROTECT_FLAG_CELL_UV;
    }
    if (s_pack_low) {
        flags |= APP_PROTECT_FLAG_PACK_LOW;
    }
    if (s_soc_low) {
        flags |= APP_PROTECT_FLAG_SOC_LOW;
    }
    if (s_pa_tx_inhibit) {
        flags |= APP_PROTECT_FLAG_TX_CUT;
    }
    if (s_pa_tx_inhibit && s_snap.pa_valid &&
        s_snap.pa_ntc_raw <= s_pa_ntc_trip_code) {
        flags |= APP_PROTECT_FLAG_PA_TEMP_HIGH;
    }
    if (!s_snap.pa_valid) {
        flags |= APP_PROTECT_FLAG_PA_FAULT;
    }
    if (s_dc_in_low) {
        flags |= APP_PROTECT_FLAG_DC_IN_LOW;
    }
    if (!s_snap.data_valid) {
        flags |= APP_PROTECT_FLAG_DATA_INVALID;
    }
    if (s_inhibit_desired) {
        flags |= APP_PROTECT_FLAG_CHARGE_INHIBIT;
    }
    if (flags != s_snap.flags) {
        uint32_t changed = flags ^ s_snap.flags;
        uint32_t mask;

        for (mask = 1u; mask != 0u; mask <<= 1u) {
            if ((changed & mask) != 0u) {
                LOG_WARN("flag %s %s", flag_name(mask),
                         (flags & mask) != 0u ? "set" : "cleared");
            }
        }
        s_snap.flags = flags;
    }
}

static void protect_tick(void *ctx) {
    (void)ctx;
    s_tick++;
    if ((s_tick % BOARD_PROTECT_SAMPLE_DIV) == 0u) {
        sample_burst();
    }
    evaluate();
    /* Bias compensation shares this tick and applies its own 20/40 ms dividers. */
    App_PaTempComp_Tick10ms();
    App_Power_Tick();
}

void App_Protect_GetSnapshot(AppProtectSnapshot *out) {
    uint32_t irq;

    if (out == 0) {
        return;
    }
    irq = Bsp_Irq_SaveLow();
    *out = s_snap;
    Bsp_Irq_RestoreLow(irq);
}

bool App_Protect_IsTxInhibited(void) {
    return s_pa_tx_inhibit;
}

bool App_Protect_IsBattWarning(void) {
    return s_batt_warning;
}

void App_Protect_SetChargeEnabled(bool enabled) {
    s_user_charge_enabled = enabled;
}

int App_Protect_Init(void) {
    /* Turn the PA temperature trip/release into raw-code thresholds once. If
     * the model cannot be inverted, latch the TX cut (fail-safe). */
    if (Drv_AdcMon_PaRawFromTempMc(BOARD_PA_TEMP_HIGH_MC,
                                   &s_pa_ntc_trip_code) != 0 ||
        Drv_AdcMon_PaRawFromTempMc(BOARD_PA_TEMP_HIGH_RELEASE_MC,
                                   &s_pa_ntc_release_code) != 0) {
        LOG_WARN("pa threshold failed, cutting tx");
        s_pa_ntc_trip_code = 0xFFFFu;
        s_pa_ntc_release_code = 0xFFFFu;
    }

    /* First burst in thread context, before the claim: cached data is fresh
     * before the tick starts and no false inhibit appears. */
    sample_burst();
    evaluate();

    Bsp_I2c_Bus2Claim();
    if (Bsp_Time_StartTimer(BSP_TIME_TIMER_PROTECT, protect_tick, 0) != 0) {
        LOG_ERROR("tim6 start failed");
        return -1;
    }
    return 0;
}
