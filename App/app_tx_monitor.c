#include "app_tx_monitor.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "app_alc.h"
#include "app_atu.h"
#include "app_ptt.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "drv_adc_mon.h"

/*
 * One ADC scan is ~156 us. The power branch models a 50 V detector feeding a
 * 50 ohm load; the raw-to-volt curve in raw_to_pa_out_volt() is calibrated
 * against the scope on the real board. The VSWR branch compares raw detector
 * codes, so its ratio does not depend on those voltage/power coefficients. If
 * the ADC clock changes, update DRV_ADC_MON_SCAN_PERIOD_US and TS follows.
 */
#define APP_TX_MON_TS_S ((float)DRV_ADC_MON_SCAN_PERIOD_US * 1.0e-6f)
#define APP_TX_MON_TAU_P_S 0.00017f
#define APP_TX_MON_TAU_PEP_S 0.100f
#define APP_TX_MON_TAU_REF_FAST_S 0.002f
#define APP_TX_MON_TAU_EXT_ALC_S 0.100f
#define APP_TX_MON_ADC_FULL_SCALE 4096.0f
#define APP_TX_MON_LOAD_OHM 50.0f
/* Below 0.03 W (1.2247 V RMS into 50 ohm) the diode detector is non-linear
 * and unstable; report zero output there. */
#define APP_TX_MON_MIN_OUT_V_RMS 1.2247449f
#define APP_TX_MON_POWER_PRESENT_W 0.2f
#define APP_TX_MON_SWR_TRIP 25.0f
#define APP_TX_MON_SWR_INVALID 99.0f
#define APP_TX_MON_BLOCK_SCANS 10u
#define APP_TX_MON_TRIP_BLOCKS 12u

/* Piecewise-linear detector calibration: raw ADC code -> RMS volts. */
#define APP_TX_MON_DET_HI_RAW   207u
#define APP_TX_MON_DET_HI_SLOPE 0.00852487f
#define APP_TX_MON_DET_HI_OFF   1.70116705f
#define APP_TX_MON_DET_LO_SLOPE 0.01189116f
#define APP_TX_MON_DET_LO_OFF   1.00247660f

static float s_alpha_p;
static float s_alpha_pep;
static float s_alpha_ref_fast;
static float s_alpha_v;
static float s_alpha_ext;

static float s_fwd_power_peak;
static float s_fwd_power_pep;
static float s_fwd_power_inst;
static float s_fwd_power_avg;
static float s_ref_power;
static float s_ref_power_fast;
static float s_ref_power_avg;
static uint16_t s_raw_fwd;
static uint16_t s_raw_ref;

/* External ALC line (EXT ALC): the smoothed raw code handed to the ALC as a
 * power-backoff command. Seeded by the first scan: unlike the power channels it
 * has no meaningful zero to start from. */
static float s_ext_alc_raw;
static bool  s_ext_alc_seeded;

static float s_fwd_v_smooth;
static float s_rev_v_smooth;
static float s_fwd_acc;
static float a_ref_acc;
static uint32_t s_decim;
static float s_swr = APP_TX_MON_SWR_INVALID;
static bool s_swr_valid;
static uint32_t s_trip_blocks;
static volatile bool s_swr_trip_pending;

/* ATU fast window: a separate raw-code accumulator, filled over a caller-set
 * number of scans, independent of the smoothed main path. */
static struct {
    bool     armed;
    uint32_t window_scans;
    uint32_t scans;
    uint32_t fwd_raw_acc;
    float    fwd_acc;
    float    ref_acc;
    volatile bool  fresh;
    volatile float gamma;
} s_atu;


static inline float raw_to_det_volt(uint16_t raw) {
    if (raw > APP_TX_MON_DET_HI_RAW) {
        return (float)raw * APP_TX_MON_DET_HI_SLOPE + APP_TX_MON_DET_HI_OFF;
    }
    return (float)raw * APP_TX_MON_DET_LO_SLOPE + APP_TX_MON_DET_LO_OFF;
}

static inline float raw_to_pa_out_volt(uint16_t raw) {
    float v = raw_to_det_volt(raw);

    if (v < APP_TX_MON_MIN_OUT_V_RMS) {
        return 0.0f;
    }
    return v;
}

/* ATU variant: below the calibrated detector floor, extrapolate the last
 * calibrated point linearly to (0, 0) instead of clamping to zero. The
 * absolute value is approximate there, but the reflection trend stays
 * monotonic, which is what the search needs near a match. */
static inline float raw_to_atu_volt(uint16_t raw) {
    float v = raw_to_det_volt(raw);

    if (v < APP_TX_MON_MIN_OUT_V_RMS) {
        const float raw_min = (APP_TX_MON_MIN_OUT_V_RMS -
                               APP_TX_MON_DET_LO_OFF) /
                              APP_TX_MON_DET_LO_SLOPE;

        v = (float)raw * (APP_TX_MON_MIN_OUT_V_RMS / raw_min);
    }
    return v;
}

/* Runs in the ADC DMA ISR. Keep it short: no logs, no blocking calls. */
static void tx_scan(const volatile uint16_t *raw, size_t count) {
    uint16_t raw_fwd;
    uint16_t raw_ref;
    uint16_t raw_ext;
    float    fwd_v;
    float    ref_v;
    float    corr;
    float    s;
    float    d;
    bool     power_present;

    if (raw == 0 || count < DRV_ADC_MON_CH_COUNT) {
        return;
    }

    raw_fwd = raw[DRV_ADC_MON_FWD];
    raw_ref = raw[DRV_ADC_MON_REFL];
    raw_ext = raw[DRV_ADC_MON_EXT_ALC];
    s_raw_fwd = raw_fwd;
    s_raw_ref = raw_ref;
    fwd_v = raw_to_pa_out_volt(raw_fwd);
    ref_v = raw_to_pa_out_volt(raw_ref);

    /* ATU fast window runs in parallel with the smoothed main branch. The raw
     * ratio is the search metric: it does not depend on the power calibration
     * and has no low-level clamp, so it stays usable near a match. */
    if (s_atu.armed) {
        s_atu.fwd_acc += raw_to_atu_volt(raw_fwd);
        s_atu.ref_acc += raw_to_atu_volt(raw_ref);
        s_atu.fwd_raw_acc += raw_fwd;
        s_atu.scans++;
        if (s_atu.scans >= s_atu.window_scans) {
            float gamma = APP_TX_MON_ATU_GAMMA_INVALID;

            if ((float)s_atu.fwd_raw_acc >=
                    (float)APP_TX_MON_ATU_MIN_FWD_RAW * (float)s_atu.scans &&
                s_atu.fwd_acc > 0.0f) {
                gamma = s_atu.ref_acc / s_atu.fwd_acc;
            }
            s_atu.gamma   = gamma;
            s_atu.fresh   = true;
            s_atu.armed   = false;
        }
    }

    /* The coupler and detector under-read with frequency; restore both the
     * forward and reflected power with the current band's factor. The raw
     * voltages stay unscaled: SWR and the ATU window are ratios, where the
     * factor cancels. */
    corr = App_Alc_GetPowerCorr();
    s_fwd_power_inst = fwd_v * fwd_v / APP_TX_MON_LOAD_OHM * corr;
    s_ref_power = ref_v * ref_v / APP_TX_MON_LOAD_OHM * corr;
    if (s_fwd_power_inst > s_fwd_power_peak) {
        s_fwd_power_peak = s_fwd_power_inst;
    } else {
        s_fwd_power_peak += s_alpha_p * (s_fwd_power_inst - s_fwd_power_peak);
    }
    if (s_fwd_power_inst > s_fwd_power_pep) {
        s_fwd_power_pep = s_fwd_power_inst;
    } else {
        s_fwd_power_pep += s_alpha_pep * (s_fwd_power_inst - s_fwd_power_pep);
    }
    s_fwd_power_avg += s_alpha_v * (s_fwd_power_inst - s_fwd_power_avg);
    s_ref_power_fast += s_alpha_ref_fast * (s_ref_power - s_ref_power_fast);
    s_ref_power_avg += s_alpha_v * (s_ref_power - s_ref_power_avg);

    /* Run the ALC on every scan (~156 us), not once per SWR block. The fast
     * layers need the short peak-hold; the slow trim uses the speech-rate
     * peak-hold so loud audio does not pull the learned gain down. */
    if (!s_ext_alc_seeded) {
        s_ext_alc_raw    = (float)raw_ext;
        s_ext_alc_seeded = true;
    } else {
        s_ext_alc_raw += s_alpha_ext * ((float)raw_ext - s_ext_alc_raw);
    }
    App_Alc_Update(s_fwd_power_peak, s_fwd_power_pep, s_fwd_power_avg,
                   s_ref_power_fast, s_ref_power_avg, (uint16_t)s_ext_alc_raw);

    s_fwd_v_smooth += s_alpha_v * (fwd_v - s_fwd_v_smooth);
    /* Reflected channel shares the scope-calibrated raw-to-volt curve; if the
     * reverse detector needs its own per-unit factor, it is applied here. */
    s_rev_v_smooth += s_alpha_v * (ref_v - s_rev_v_smooth);
    s_fwd_acc += s_fwd_v_smooth;
    a_ref_acc += s_rev_v_smooth;

    s_decim++;
    if (s_decim < APP_TX_MON_BLOCK_SCANS) {
        return;
    }
    s_decim = 0u;

    s = s_fwd_acc + a_ref_acc;
    if (s < APP_TX_MON_MIN_OUT_V_RMS * APP_TX_MON_BLOCK_SCANS) {
        // Low voltage, computed SWR might be wrong
        s_swr = 1.0f;
        s_swr_valid = true;
    } else {
        d = s_fwd_acc - a_ref_acc;
        s_swr_valid = (d > 0.0f) && isfinite(s) && isfinite(d);
        s_swr = s_swr_valid ? (s / d) : APP_TX_MON_SWR_INVALID;
    }

    power_present = (s_fwd_power_peak + s_ref_power) >= APP_TX_MON_POWER_PRESENT_W;
    if (App_Atu_IsTuning() || App_Ptt_IsSwrScanActive()) {
        /* A high SWR is expected during ATU tuning and during an SWR scan; do
         * not accumulate the trip and drop any partial count. */
        s_trip_blocks = 0u;
    } else if (power_present && s_swr > APP_TX_MON_SWR_TRIP) {
        if (s_trip_blocks < APP_TX_MON_TRIP_BLOCKS) {
            s_trip_blocks++;
            if (s_trip_blocks == APP_TX_MON_TRIP_BLOCKS) {
                s_swr_trip_pending = true;
            }
        }
    } else {
        s_trip_blocks = 0u;
    }

    s_fwd_acc *= 0.1f;
    a_ref_acc *= 0.1f;
}

void App_TxMonitor_Init(void) {
    s_alpha_p = 1.0f - expf(-APP_TX_MON_TS_S / APP_TX_MON_TAU_P_S);
    s_alpha_pep = 1.0f - expf(-APP_TX_MON_TS_S / APP_TX_MON_TAU_PEP_S);
    s_alpha_ref_fast =
        1.0f - expf(-APP_TX_MON_TS_S / APP_TX_MON_TAU_REF_FAST_S);
    s_alpha_v = 1.0f - expf(-APP_TX_MON_TS_S / BOARD_TX_AVG_TAU_S);
    s_alpha_ext = 1.0f - expf(-APP_TX_MON_TS_S / APP_TX_MON_TAU_EXT_ALC_S);

    s_fwd_power_peak = 0.0f;
    s_fwd_power_pep = 0.0f;
    s_fwd_power_inst = 0.0f;
    s_fwd_power_avg = 0.0f;
    s_ref_power = 0.0f;
    s_ref_power_fast = 0.0f;
    s_ref_power_avg = 0.0f;
    s_raw_fwd = 0u;
    s_raw_ref = 0u;
    s_ext_alc_raw = 0.0f;
    s_ext_alc_seeded = false;
    s_fwd_v_smooth = 0.0f;
    s_rev_v_smooth = 0.0f;
    s_fwd_acc = 0.0f;
    a_ref_acc = 0.0f;
    s_decim = 0u;
    s_swr = APP_TX_MON_SWR_INVALID;
    s_swr_valid = false;
    s_trip_blocks = 0u;
    s_swr_trip_pending = false;

    s_atu.armed        = false;
    s_atu.window_scans = 0u;
    s_atu.scans        = 0u;
    s_atu.fwd_raw_acc  = 0u;
    s_atu.fwd_acc      = 0.0f;
    s_atu.ref_acc      = 0.0f;
    s_atu.fresh        = false;
    s_atu.gamma        = 0.0f;

    (void)Drv_AdcMon_SetScanCallback(tx_scan);
}

void App_TxMonitor_GetSnapshot(AppTxMonitorSnapshot *out) {
    uint32_t irq;

    if (out == 0) {
        return;
    }
    irq = Bsp_Irq_SaveLow();
    out->fwd_power_peak = s_fwd_power_peak;
    out->fwd_power_pep = s_fwd_power_pep;
    out->fwd_power_inst = s_fwd_power_inst;
    out->fwd_power_avg = s_fwd_power_avg;
    out->ref_power = s_ref_power;
    out->ref_power_fast = s_ref_power_fast;
    out->ref_power_avg = s_ref_power_avg;
    out->swr = s_swr;
    out->swr_valid = s_swr_valid;
    out->raw_fwd = s_raw_fwd;
    out->raw_ref = s_raw_ref;
    Bsp_Irq_RestoreLow(irq);
}

bool App_TxMonitor_TakeSwrTrip(void) {
    bool     tripped;
    uint32_t irq;

    irq = Bsp_Irq_SaveLow();
    tripped = s_swr_trip_pending;
    s_swr_trip_pending = false;
    Bsp_Irq_RestoreLow(irq);
    return tripped;
}

void App_TxMonitor_AtuArm(uint32_t window_ms) {
    uint32_t scans = (window_ms * 1000u + DRV_ADC_MON_SCAN_PERIOD_US / 2u) /
                     DRV_ADC_MON_SCAN_PERIOD_US;
    uint32_t irq;

    if (scans == 0u) {
        scans = 1u;
    }
    irq = Bsp_Irq_SaveLow();
    s_atu.window_scans = scans;
    s_atu.scans        = 0u;
    s_atu.fwd_raw_acc  = 0u;
    s_atu.fwd_acc      = 0.0f;
    s_atu.ref_acc      = 0.0f;
    s_atu.fresh        = false;
    s_atu.armed        = true;
    Bsp_Irq_RestoreLow(irq);
}

bool App_TxMonitor_AtuTake(float *gamma) {
    bool     fresh;
    float    value;
    uint32_t irq;

    irq   = Bsp_Irq_SaveLow();
    fresh = s_atu.fresh;
    value = s_atu.gamma;
    if (fresh) {
        s_atu.fresh = false;
        s_atu.armed = false;
    }
    Bsp_Irq_RestoreLow(irq);

    if (fresh && gamma != 0) {
        *gamma = value;
    }
    return fresh;
}
