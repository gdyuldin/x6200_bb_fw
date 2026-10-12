#include "sdr_squelch.h"

#include <math.h>

#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_codec.h"
#include "app_event.h"
#include "sdr_am.h"
#include "sdr_cw.h"
#include "sdr_nbfm.h"
#include "sdr_ssb.h"

#define LOG_TAG "app.sdr.squelch"

#define SDR_SQUELCH_BLOCK_RATE \
    ((float)BOARD_AUDIO_SAMPLE_RATE_HZ / (float)DRV_CODEC_BLOCK_FRAMES)

/* Threshold scale: host 0..100 -> 0..20 dB. */
#define SDR_SQUELCH_THR_SCALE 0.2f
#define SDR_SQUELCH_THR_OFFSET (-4.0f)
/* Metric EMA (short, so the start of a signal is not missed). The demods return
 * the raw per-block value. */
#define SDR_SQUELCH_EMA_TAU_S 0.02f
/* Open above thr, close below thr - hysteresis. */
#define SDR_SQUELCH_HYST_DB   2.0f

typedef struct {
    uint32_t hold_ms;
    uint32_t decay_ms;
} SdrSquelchTiming;

/* Configuration published by App_RadioState (main loop), read in the ISR. */
static volatile bool          s_enabled;
static volatile uint8_t       s_thr;
static volatile uint8_t       s_thr_fm;
static volatile SdrModulation s_mode;
static volatile AppRadioMode  s_radio_mode;

/* ISR-only decision state. */
static bool          s_open;
static float         s_ema;
static float         s_gain;
static uint32_t      s_hold_blocks;
static float         s_alpha;

/* Reset request set by the event callbacks (main loop), consumed in the ISR. */
static volatile bool s_reset;

static float squelch_metric() {
    switch (s_mode) {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        return Sdr_SSB_GetSignalMetric();
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        return Sdr_CW_GetSignalMetric();
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        return Sdr_AM_GetSignalMetric();
    case SDR_MOD_NFM:
        return Sdr_NBFM_GetSignalMetric();
    default:
        return 0.0f;
    }
}

/* CW/AM/NBFM release fast enough to follow keying and speech gaps; SSB holds
 * long to bridge pauses within a transmission. */
static SdrSquelchTiming squelch_timing(SdrModulation mode) {
    switch (mode) {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        return (SdrSquelchTiming){ 1000u, 1500u };
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        return (SdrSquelchTiming){ 300u, 6u };
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        return (SdrSquelchTiming){ 200u, 30u };
    case SDR_MOD_NFM:
        return (SdrSquelchTiming){ 50u, 40u };
    default:
        return (SdrSquelchTiming){ 1000u, 1500u };
    }
}

static float squelch_ms_to_blocks(uint32_t ms) {
    return (float)ms * SDR_SQUELCH_BLOCK_RATE / 1000.0f;
}

static void squelch_reset_open(void) {
    s_open        = true;
    s_gain        = 1.0f;
    s_ema         = 0.0f;
    s_hold_blocks = (uint32_t)(squelch_ms_to_blocks(squelch_timing(s_mode).hold_ms) + 0.5f);
}

/* Switching mode or RX/TX must never silence: reset to open. */
static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;
    /* Only the mode matters; the block also carries the filter edges. */
    if (cfg->rx_mode != s_mode) {
        s_mode  = cfg->rx_mode;
        s_reset = true;
    }
}

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (evt->mode != s_radio_mode) {
        s_radio_mode = evt->mode;
        s_reset      = true;
    }
}

void Sdr_Squelch_Init(void) {
    s_enabled    = false;
    s_thr        = 0u;
    s_thr_fm     = 0u;
    s_mode       = SDR_MOD_LSB;
    s_radio_mode = APP_RADIO_MODE_OFF;
    s_reset      = false;

    s_alpha = 1.0f - expf(-1.0f / (SDR_SQUELCH_EMA_TAU_S * SDR_SQUELCH_BLOCK_RATE));
    squelch_reset_open();

    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
}

void Sdr_Squelch_Configure(bool enable, uint8_t threshold, uint8_t threshold_fm) {
    uint32_t key = Bsp_Irq_SaveLow();

    s_enabled = enable;
    s_thr     = threshold;
    s_thr_fm  = threshold_fm;
    Bsp_Irq_RestoreLow(key);
}

void Sdr_Squelch_Process(float *audio48, uint32_t n) {
    uint8_t          threshold;
    SdrSquelchTiming timing;
    uint32_t         hold_blocks;
    float            decay_blocks;

    if (audio48 == 0 || n == 0u) {
        return;
    }

    timing       = squelch_timing(s_mode);
    hold_blocks  = (uint32_t)(squelch_ms_to_blocks(timing.hold_ms) + 0.5f);
    decay_blocks = squelch_ms_to_blocks(timing.decay_ms);

    threshold = (s_mode == SDR_MOD_NFM) ? s_thr_fm : s_thr;
    if (s_reset) {
        s_reset = false;
        squelch_reset_open();
    }

    /* Bypass (disabled, threshold 0, or unsupported mode) keeps the audio open
     * but still ramps the gain back so a config change does not produce a step. */
    if (!s_enabled || threshold == 0u || s_mode == SDR_MOD_WFM) {
        s_open        = true;
        s_hold_blocks = hold_blocks;
    } else {
        float metric = squelch_metric();
        float thr_db = (float)threshold * SDR_SQUELCH_THR_SCALE + SDR_SQUELCH_THR_OFFSET;

        s_ema += (metric - s_ema) * s_alpha;

        if (!s_open) {
            if (s_ema > thr_db) {
                s_open        = true;
                s_hold_blocks = hold_blocks;
            }
        } else if (s_ema < thr_db - SDR_SQUELCH_HYST_DB) {
            if (s_hold_blocks > 0u) {
                s_hold_blocks--;
            } else {
                s_open = false;
            }
        } else {
            s_hold_blocks = hold_blocks;
        }
    }

    if (s_open && s_gain >= 1.0f) {
        s_gain = 1.0f;
        return;
    }

    if (s_open) {
        /* Attack: reach full gain within one block. */
        float step = (1.0f - s_gain) / (float)n;

        for (uint32_t i = 0; i < n; ++i) {
            s_gain += step;
            audio48[i] *= s_gain;
        }
        s_gain = 1.0f;
    } else {
        /* Release: linear ramp down over the mode's decay time. */
        float step = 1.0f / (decay_blocks * (float)n);

        for (uint32_t i = 0; i < n; ++i) {
            s_gain -= step;
            if (s_gain < 0.0f) {
                s_gain = 0.0f;
            }
            audio48[i] *= s_gain;
        }
    }
}

bool Sdr_Squelch_IsMuted(void) {
    return !s_open;
}
