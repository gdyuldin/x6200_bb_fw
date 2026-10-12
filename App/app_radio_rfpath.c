#include "app_radio_rfpath.h"

#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_band_volt.h"
#include "bsp_log.h"
#include "drv_pca9535.h"

#define LOG_TAG "app.radio_rfpath"

/* Bit fields on the preamp expander 0x20: pre [14], att [13], gain [5:0];
 * BPF band code [11:8]. */
#define APP_RADIO_RFPATH_BPF_MASK  0x0F00u

/* LPF decode field in bits [15:6] of the band-relay expander; bits [5:0]
 * (tuner_on) are owned by the shadow and never touched here. */
#define APP_RADIO_RFPATH_LPF_MASK  0xFFC0u
/* Reset pulse before the new code: hold all six relay lines while releasing. */
#define APP_RADIO_RFPATH_LPF_RESET 0x7FC0u

/* HMC472A full-scale code: 6 bits, 0.5 dB LSB, 31.5 dB. */
#define APP_RADIO_RFPATH_GAIN_MAX  0x3Fu

/* Front-end gain estimates, tune on hardware. */
#define APP_RADIO_RFPATH_PRE_GAIN_DB 15.0f
#define APP_RADIO_RFPATH_ATT_LOSS_DB 15.0f
#define APP_RADIO_RFPATH_GAIN_STEP_DB 0.5f

/* Upper inclusive band edge per index; the value past the last edge maps to
 * the terminal index (BPF_COUNT-1 / LPF_COUNT-1). */
static const uint32_t s_bpf_hi[APP_RADIO_RFPATH_BPF_COUNT - 1u] = {
    1799999u, 2200000u, 3800000u, 5500000u, 7500000u,
    13500000u, 21500000u, 33000000u, 49999999u, 55000000u,
};

static const uint16_t s_bpf_code[APP_RADIO_RFPATH_BPF_COUNT] = {
    0x000u, 0x800u, 0x400u, 0xC00u, 0x200u, 0xA00u,
    0x600u, 0xE00u, 0x100u, 0x900u, 0x500u,
};

static const uint32_t s_lpf_hi[APP_RADIO_RFPATH_LPF_COUNT - 1u] = {
    2500000u, 4000000u, 5405000u, 7400000u,
    10800000u, 15500000u, 25500000u, 37000000u,
};

static const uint16_t s_lpf_code[APP_RADIO_RFPATH_LPF_COUNT] = {
    0xBFC0u, 0xDFC0u, 0xEFC0u, 0xF7C0u, 0xFBC0u,
    0xFDC0u, 0xFEC0u, 0xFF40u, 0xFF80u,
};

/* Band-voltage analog output (PA3): target millivolts per amateur band, keyed
 * off the active frequency with the upper band edges as inclusive thresholds.
 * The edges differ from the BPF/LPF tables and the ALC band boundaries, so the
 * map is kept separate. Indices run 160m..6m. */
static const uint32_t s_band_volt_hi[APP_RADIO_RFPATH_BAND_VOLT_COUNT - 1u] = {
    2000000u, 4000000u, 6000000u, 8000000u, 10500000u,
    14500000u, 18500000u, 21500000u, 24900000u, 30000000u,
};

static const uint16_t s_band_volt_mv[APP_RADIO_RFPATH_BAND_VOLT_COUNT] = {
    230u, 460u, 690u, 920u, 1150u, 1380u,
    1610u, 1840u, 2070u, 2300u, 2530u,
};

static int8_t  s_lpf_target  = -1;
static int8_t  s_lpf_playing = -1;
static int8_t  s_lpf_applied = -1;
static uint8_t s_lpf_phase;

/* Applied RX front-end state, kept in sync with the last successful writes. */
static struct {
    bool    pre;
    bool    att;
    uint8_t gain_steps;
} s_rx_path;

static uint8_t index_from_edges(uint32_t freq_hz, const uint32_t *hi,
                                uint8_t terminal) {
    for (uint8_t k = 0; k < terminal; ++k) {
        if (freq_hz <= hi[k]) {
            return k;
        }
    }
    return terminal;
}

uint8_t App_RadioRfPath_BpfIndex(uint32_t freq_hz) {
    return index_from_edges(freq_hz, s_bpf_hi,
                            APP_RADIO_RFPATH_BPF_COUNT - 1u);
}

uint16_t App_RadioRfPath_BpfCode(uint8_t index) {
    if (index >= APP_RADIO_RFPATH_BPF_COUNT) {
        return 0u;
    }
    return s_bpf_code[index];
}

uint8_t App_RadioRfPath_LpfIndex(uint32_t freq_hz) {
    return index_from_edges(freq_hz, s_lpf_hi,
                            APP_RADIO_RFPATH_LPF_COUNT - 1u);
}

uint16_t App_RadioRfPath_LpfCode(uint8_t index) {
    if (index >= APP_RADIO_RFPATH_LPF_COUNT) {
        return 0u;
    }
    return s_lpf_code[index];
}

uint16_t App_RadioRfPath_BandVoltMv(uint32_t freq_hz) {
    return s_band_volt_mv[index_from_edges(
        freq_hz, s_band_volt_hi, APP_RADIO_RFPATH_BAND_VOLT_COUNT - 1u)];
}

/*
 * The HMC472A control lines are low-active and its six blocks sit in reverse
 * order relative to the ladder code, so this writes W = reverse6(0x3F - g)
 * where g is the desired attenuation in 0.5 dB steps (0 = 0 dB, 63 = 31.5 dB).
 * GainCode() feeds this function g = clamp(100 - rfg, 0, 63).
 */
static uint8_t reverse6(uint8_t v) {
    uint8_t r = 0u;

    for (uint8_t i = 0u; i < 6u; ++i) {
        r = (uint8_t)((r << 1) | ((v >> i) & 1u));
    }
    return r;
}

/* rfg 100 = 0 dB, rfg 37 = 31.5 dB, rfg <= 36 saturates; rfg > 100 is not a
 * valid host value and clamps to 0 dB like the setter. */
static uint8_t rfg_gain_steps(uint8_t rfg) {
    uint8_t g = (rfg > APP_RADIO_RFPATH_RFG_MAX)
                    ? 0u
                    : (uint8_t)(APP_RADIO_RFPATH_RFG_MAX - rfg);

    return (g > APP_RADIO_RFPATH_GAIN_MAX) ? APP_RADIO_RFPATH_GAIN_MAX : g;
}

uint8_t App_RadioRfPath_GainCode(uint8_t rfg) {
    return reverse6((uint8_t)(APP_RADIO_RFPATH_GAIN_MAX - rfg_gain_steps(rfg)));
}

void App_RadioRfPath_SetPre(bool on) {
    if (Drv_Pca9535_WriteMasked(DRV_PCA9535_ADDR_PREAMP, 0x4000u,
                                on ? 0x4000u : 0u) != 0) {
        LOG_WARN("preamp write failed");
        return;
    }
    s_rx_path.pre = on;
}

void App_RadioRfPath_SetAtt(bool on) {
    if (Drv_Pca9535_WriteMasked(DRV_PCA9535_ADDR_PREAMP, 0x2000u,
                                on ? 0x2000u : 0u) != 0) {
        LOG_WARN("atten write failed");
        return;
    }
    s_rx_path.att = on;
}

void App_RadioRfPath_SetRfg(uint8_t rfg) {
    if (Drv_Pca9535_WriteMasked(DRV_PCA9535_ADDR_PREAMP, 0x003Fu,
                                App_RadioRfPath_GainCode(rfg)) != 0) {
        LOG_WARN("rf gain write failed");
        return;
    }
    s_rx_path.gain_steps = rfg_gain_steps(rfg);
}

float App_RadioRfPath_GetRxGainDb(void) {
    float db = -(float)s_rx_path.gain_steps * APP_RADIO_RFPATH_GAIN_STEP_DB;

    if (s_rx_path.pre) {
        db += APP_RADIO_RFPATH_PRE_GAIN_DB;
    }
    if (s_rx_path.att) {
        db -= APP_RADIO_RFPATH_ATT_LOSS_DB;
    }
    return db;
}

/*
 * One relay step per tick; the four phases are reset pulse, release, new code,
 * release. phase 0 is idle and makes no transfer while target matches applied,
 * so a relay inside the current band is never actuated. s_lpf_playing latches
 * the target of the running sequence, so a target that moves mid-sequence is
 * picked up after the current sequence finishes rather than skipped.
 */
static void App_RadioRfPath_Task(void) {
    uint16_t value;

    if (s_lpf_phase == 0u) {
        if (s_lpf_target < 0 || s_lpf_target == s_lpf_applied) {
            return;
        }
        s_lpf_playing = s_lpf_target;
        value = APP_RADIO_RFPATH_LPF_RESET;
    } else if (s_lpf_phase == 1u) {
        value = 0u;
    } else if (s_lpf_phase == 2u) {
        value = App_RadioRfPath_LpfCode((uint8_t)s_lpf_playing);
    } else {
        value = 0u;
    }

    if (Drv_Pca9535_WriteMasked(DRV_PCA9535_ADDR_BAND_RELAY,
                                APP_RADIO_RFPATH_LPF_MASK, value) != 0) {
        LOG_WARN("lpf write phase %u failed", (unsigned)s_lpf_phase);
    }

    if (s_lpf_phase == 3u) {
        s_lpf_applied = s_lpf_playing;
    }
    s_lpf_phase = (uint8_t)((s_lpf_phase + 1u) & 0x3u);
}

void App_RadioRfPath_SetFreq(uint32_t freq_hz) {
    uint8_t code = App_RadioRfPath_BpfIndex(freq_hz);

    if (Drv_Pca9535_WriteMasked(DRV_PCA9535_ADDR_PREAMP,
                                APP_RADIO_RFPATH_BPF_MASK,
                                App_RadioRfPath_BpfCode(code)) != 0) {
        LOG_WARN("bpf write failed");
    }
    s_lpf_target = (int8_t)App_RadioRfPath_LpfIndex(freq_hz);
    Bsp_BandVolt_SetMillivolts(App_RadioRfPath_BandVoltMv(freq_hz));
}

bool App_RadioRfPath_LpfWillChange(uint32_t freq_hz) {
    return s_lpf_target != (int8_t)App_RadioRfPath_LpfIndex(freq_hz);
}

int App_RadioRfPath_Init(void) {
    int rc;
    int task_rc;

    s_lpf_target  = -1;
    s_lpf_playing = -1;
    s_lpf_applied = -1;
    s_lpf_phase   = 0u;

    /* Force preamp/att/gain to their boot defaults in one masked write so a
     * zeroed shadow cannot be mistaken for "already applied". BPF and LPF are
     * left untouched until the first SetFreq, which arrives only after the host
     * configuration is accepted; s_lpf_target stays -1 so the relay task emits
     * no pulses. */
    rc = Drv_Pca9535_WriteMasked(DRV_PCA9535_ADDR_PREAMP, 0x603Fu, 0u);
    if (rc != 0) {
        LOG_WARN("init write failed (%d)", rc);
    } else {
        s_rx_path.pre        = false;
        s_rx_path.att        = false;
        s_rx_path.gain_steps = APP_RADIO_RFPATH_GAIN_MAX;
    }

    /* Start the band-voltage PWM at its idle pulse; the first SetFreq sets the
     * level of the active band. */
    if (Bsp_BandVolt_Init() != 0) {
        LOG_WARN("band volt init failed");
    }

    task_rc = App_Scheduler_Add(App_RadioRfPath_Task, BOARD_RF_RELAY_STEP_MS);
    return rc != 0 ? rc : task_rc;
}
