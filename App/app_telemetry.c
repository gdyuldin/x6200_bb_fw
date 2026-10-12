#include "app_telemetry.h"

#include <errno.h>
#include <string.h>

#include "app_alc.h"
#include "app_atu.h"
#include "app_power.h"
#include "app_protect.h"
#include "app_radio_rfpath.h"
#include "app_radio_state.h"
#include "app_scheduler.h"
#include "app_tx_monitor.h"
#include "bsp_board.h"
#include "bsp_crc.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "drv_fpga.h"
#include "drv_hm_key.h"
#include "sdr_rx.h"
#include "sdr_squelch.h"
#include "sdr_spectrum.h"

#define LOG_TAG "app.telemetry"

#define APP_TELEMETRY_SLIP_END     0xC0u
#define APP_TELEMETRY_SLIP_ESC     0xDBu
#define APP_TELEMETRY_SLIP_ESC_END 0xDCu
#define APP_TELEMETRY_SLIP_ESC_ESC 0xDDu

_Static_assert(SDR_SPECTRUM_FFT_SIZE == TELEMETRY_FLOW_FFT_BINS,
               "telemetry bins must match the FFT size");

static bool s_tx;
static telemetry_flow_t s_flow;
static uint8_t          s_tx_buf[APP_TELEMETRY_FRAME_MAX]
    __attribute__((section(".dma_d2"), aligned(32)));
static volatile bool    s_tx_busy;
static AppTelemetryFraming s_framing = APP_TELEMETRY_FRAMING_SLIP;
static uint32_t         s_drops;
static uint32_t         s_warn_ms;


static const char *framing_name(AppTelemetryFraming framing) {
    switch (framing) {
    case APP_TELEMETRY_FRAMING_SLIP: return "slip";
    case APP_TELEMETRY_FRAMING_RAW:  return "raw";
    default:                         return "cobs";
    }
}

static void report_drop(void) {
    uint32_t now = Bsp_Time_GetMs();
    /* Warn at most once per second when a packet is dropped. */
    if ((now - s_warn_ms) >= 1000u) {
        s_warn_ms = now;
        LOG_WARN("packet dropped, total %u", (unsigned)s_drops);
    }
}

/* Scale a physical value to 0.1 units and clamp it to the packet byte. */
static uint8_t clamp_tenths(float value) {
    float scaled;

    if (!(value > 0.0f)) {
        return 0u;
    }
    scaled = value * 10.0f;
    if (scaled >= 255.0f) {
        return 255u;
    }
    return (uint8_t)scaled;
}

static void fill_flow(void) {
    AppProtectSnapshot   snap;
    AppTxMonitorSnapshot tx;
    const float          *db = Sdr_Spectrum_Get_Db();
    uint32_t             flags = 0u;

    App_Protect_GetSnapshot(&snap);
    App_TxMonitor_GetSnapshot(&tx);

    memset(&s_flow, 0, sizeof(s_flow));
    s_flow.magic = TELEMETRY_FLOW_MAGIC;
    for (size_t i = 0; i < TELEMETRY_FLOW_FFT_BINS; i++) {
        int v = (int)(db[i] + 127.0f);
        if (v < 0) {
            v = 0;
        } else if (v > 255) {
            v = 255;
        }
        s_flow.samples[i] = (uint8_t)v;
    }

    if (s_tx) {
        flags |= TELEMETRY_FLOW_FLAG_TX;
    }
    if (snap.adapter_present) {
        flags |= TELEMETRY_FLOW_FLAG_VEXT;
    }
    if (snap.charger.ac_ok && (snap.charger.option & 1u) == 0u) {
        flags |= TELEMETRY_FLOW_FLAG_CHARGING;
    }
    if (!snap.data_valid) {
        flags |= TELEMETRY_FLOW_FLAG_BATTERY_NOT_FOUND;
    }
    if ((snap.flags & APP_PROTECT_FLAG_BATT_TEMP_HIGH) != 0u) {
        flags |= TELEMETRY_FLOW_FLAG_BATTERY_HIGH_TEMP;
    }
    if ((snap.flags & APP_PROTECT_FLAG_PA_TEMP_HIGH) != 0u) {
        flags |= TELEMETRY_FLOW_FLAG_PA_HIGH_TEMP;
    }
    if (Bsp_Board_IsPowerButtonPressed()) {
        flags |= TELEMETRY_FLOW_FLAG_POWER_KEY;
    }
    if ((snap.flags & APP_PROTECT_FLAG_DC_IN_LOW) != 0u) {
        flags |= TELEMETRY_FLOW_FLAG_DC_IN_TOO_LOW;
    }
    if (App_Power_IsAutoOffActive()) {
        flags |= TELEMETRY_FLOW_FLAG_AUTO_OFF_ACTIVE;
    }
    if ((snap.flags & APP_PROTECT_FLAG_BATT_TEMP_COLD) != 0u) {
        flags |= TELEMETRY_FLOW_FLAG_BATTERY_LOW_TEMP;
    }
    if (Sdr_Squelch_IsMuted()) {
        if (App_RadioState_GetRxMode() == SDR_MOD_NFM) {
            flags |= TELEMETRY_FLOW_FLAG_SQL_FM_MUTE;
        } else {
            flags |= TELEMETRY_FLOW_FLAG_SQL_MUTE;
        }
    }
    /* ATU status: 1 = idle/finished, 0 = tuning in progress. */
    if (!App_Atu_IsTuning()) {
        flags |= TELEMETRY_FLOW_FLAG_ATU_STATUS;
    }
    s_flow.flag = flags;
    s_flow.atu_params = App_Atu_GetParams();

    {
        float dbm = Sdr_Rx_Get_Level_Db();

        /* The RX front-end gain/loss sits between the antenna and the measured
         * level, so refer the reading back to the antenna while receiving. */
        if (!s_tx) {
            dbm -= App_RadioRfPath_GetRxGainDb();
        }
        /* Host expects inverted */
        dbm = -dbm;
        if (dbm < -128.0f) {
            dbm = -128.0f;
        } else if (dbm > 127.0f) {
            dbm = 127.0f;
        }
        s_flow.dbm = (int8_t)dbm;
    }

    s_flow.tx_power = clamp_tenths(tx.fwd_power_peak);
    s_flow.vswr = clamp_tenths(tx.swr);
    s_flow.alc_level = App_Alc_GetLevel();
    s_flow.vext = (uint8_t)(snap.vext_mv / 100u);
    s_flow.vbat = (uint8_t)(snap.gauge.batt_mv / 100u);
    s_flow.batcap = (int8_t)(snap.gauge.soc_d_pct / 10u);
    s_flow.bat_temp = (int8_t)(snap.gauge.temp_mc / 1000);
    s_flow.shutdown_count = App_Power_GetCountdownS();
    s_flow.hkey = Drv_HmKey_GetCode();
}

static size_t slip_encode(const uint8_t *src, size_t len, uint8_t *dst) {
    size_t n = 0;

    /* Frames carry a single trailing SLIP_END; a leading END would put a
     * "C0 C0" pair (empty frame) between packets and the host parser rejects
     * that. */
    for (size_t i = 0; i < len; i++) {
        if (src[i] == APP_TELEMETRY_SLIP_END) {
            dst[n++] = APP_TELEMETRY_SLIP_ESC;
            dst[n++] = APP_TELEMETRY_SLIP_ESC_END;
        } else if (src[i] == APP_TELEMETRY_SLIP_ESC) {
            dst[n++] = APP_TELEMETRY_SLIP_ESC;
            dst[n++] = APP_TELEMETRY_SLIP_ESC_ESC;
        } else {
            dst[n++] = src[i];
        }
    }
    dst[n++] = APP_TELEMETRY_SLIP_END;
    return n;
}

static void tx_done(int status, void *ctx) {
    (void)status;
    (void)ctx;
    s_tx_busy = false;
}

static void flow_on_ready(void) {
    size_t len;

    if (s_tx_busy) {
        s_drops++;
        report_drop();
        return;
    }

    fill_flow();
    if (Bsp_Crc32_Calc(&s_flow, TELEMETRY_FLOW_CRC_LEN, &s_flow.crc) != 0) {
        s_drops++;
        report_drop();
        return;
    }

    if (s_framing == APP_TELEMETRY_FRAMING_SLIP) {
        len = slip_encode((const uint8_t *)&s_flow, TELEMETRY_FLOW_SIZE, s_tx_buf);
    } else {
        memcpy(s_tx_buf, &s_flow, TELEMETRY_FLOW_SIZE);
        len = TELEMETRY_FLOW_SIZE;
    }

    s_tx_busy = true;
    if (Bsp_Uart_WriteAsync(BSP_UART_AUX, s_tx_buf, (uint16_t)len, tx_done, 0) !=
        0) {
        s_tx_busy = false;
        s_drops++;
        report_drop();
    }
}

int App_Telemetry_SetFraming(AppTelemetryFraming framing) {
    switch (framing) {
    case APP_TELEMETRY_FRAMING_SLIP:
    case APP_TELEMETRY_FRAMING_RAW:
        s_framing = framing;
        LOG_INFO("framing %s", framing_name(framing));
        return 0;
    case APP_TELEMETRY_FRAMING_COBS:
    default:
        return -ENOTSUP;
    }
}

int App_Telemetry_Init(void) {
    if (Bsp_Crc32_SelfTest() != 0) {
        LOG_ERROR("crc self-test failed");
    }
    if (App_Scheduler_AddEvent(flow_on_ready, APP_SCHED_EV_SPECTRUM_READY) != 0) {
        LOG_ERROR("spectrum ready handler not registered");
        return -1;
    }
    LOG_INFO("telemetry flow ready, framing %s", framing_name(s_framing));
    return 0;
}

void App_Telemetry_SetTx(bool tx) {
    s_tx = tx;
}
