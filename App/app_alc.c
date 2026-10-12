#include "app_alc.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_atu.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_adc_mon.h"
#include "drv_fpga.h"
#include "sdr_common.h"

#define LOG_TAG "app.alc"

#define APP_ALC_Q15_ONE ((int32_t)32768)

#define APP_ALC_FAST_LIMIT_PERCENT 120u
#define APP_ALC_FAST_HYST_PERCENT 105u
#define APP_ALC_FAST_HOLD_SCANS 16u
#define APP_ALC_ATTACK_DIV 3

/* 32768 * 156 us / 500 ms, rounded down. */
#define APP_ALC_RELEASE_STEP ((int32_t)(32768u * 156u / 500000u))
/* 32768 * 156 us / 2 s, rounded down. */
#define APP_ALC_SWR_RELEASE_STEP ((int32_t)(32768u * 156u / 2000000u))

#define APP_ALC_SWR_REF_LIMIT_MW 1000u
#define APP_ALC_SWR_RELEASE_HYST_MW 700u
/* Input power in the normalized IQ domain (mean square of a block), not watts. */
#define APP_ALC_SWR_RELEASE_POW_MIN 0.01f
#define APP_ALC_SWR_HOLD_SCANS 3200u
#define APP_ALC_SWR_ATTACK_COOLDOWN_SCANS 32u

/* External ALC backoff (EXT ALC line): a linear profile in raw counts,
 * re-anchored so the zero point sits at APP_ALC_EXT_RAW_ZERO. The profile is
 * clamped by raw counts at both ends: the zero anchor and a lower floor at the
 * ADC quantization limit (APP_ALC_EXT_RAW_MIN). Only attenuation is possible,
 * so the power factor is >= 1 everywhere and the applied gain never exceeds
 * unity. */
#define APP_ALC_EXT_RAW_ZERO ((3000u * 4096u) / DRV_ADC_MON_VREF_MV)
#define APP_ALC_EXT_RAW_MIN  ((160u * 4096u) / DRV_ADC_MON_VREF_MV)

/* Feed-forward transfer estimate and TX soft-start. */
/* Smoothing time constant of the transfer K estimate, both directions. */
#define APP_ALC_K_TAU_S 0.1f
/* Conservative starting transfer estimate. The per-band level calibration makes
 * the real transfer settle at BOARD_TX_MAX_POWER_W, so the seed is that value
 * times a small margin: a higher K means more attenuation, so the first blocks
 * are already safe instead of open-loop. */
#define APP_ALC_K_SEED_RATIO 1.2f
#define APP_ALC_K_SEED (BOARD_TX_MAX_POWER_W * APP_ALC_K_SEED_RATIO)
#define APP_ALC_BAND_COUNT   11u
/* Clamp window for the per-band level calibration (dBFS). Guards against a held
 * console key overdriving the PA. */
#define APP_ALC_LEVEL_DB_MIN (-40.0f)
#define APP_ALC_LEVEL_DB_MAX (0.0f)
/* Clamp window for the per-band measured-power correction (dB). Positive scales
 * the reading up: the coupler/detector under-reads with frequency. */
#define APP_ALC_POWER_CORR_DB_MIN (0.0f)
#define APP_ALC_POWER_CORR_DB_MAX (6.0f)
/* Forward-power floor of the detector's low-count region (dead zone 0.03 W,
 * calibration knee ~0.24 W). A K update is taken only when the commanded output
 * (input level times setpoint) is above it, and a measured fwd below it is
 * clamped up to it: the true value is unknown but smaller, so the clamped ratio
 * can only pull K down and the estimate converges once the output is
 * measurable. */
#define APP_ALC_K_MIN_FWD_W 0.3f
/* Input mean power below which the feed-forward model is not applied; equals the
 * old amplitude floor (0.17825) squared. */
#define APP_ALC_MIN_IQ_POW 0.03177f
/* Fast limiter headroom over the setpoint, derived from the percent above so
 * the reactive (measured) and feed-forward (predicted) ceilings stay equal. */
#define APP_ALC_FF_LIMIT_RATIO ((float)APP_ALC_FAST_LIMIT_PERCENT / 100.0f)
#define APP_ALC_TX_RAMP_STEPS 32u
/* Transport delay from the applied drive to the measured forward power (staging
 * ring, FPGA, PA and detector), measured on hardware: tau ~6.2 ms. The nominal
 * IQ block rate is 750 Hz (64 samples at 48 kHz), so ~5 blocks. K compares the
 * current fwd against the predicted power of that many blocks ago; this matters
 * only on fast level changes (AM/SSB voice). */
#define APP_ALC_K_DELAY_BLOCKS 6u
/* Blocks of TX drive ignored before the first K update. After the drive was
 * absent the prediction delay line holds no TX data (RX, or the ATU carrier-off
 * tail); dividing the detector floor by that near-zero prediction throws K up
 * by orders of magnitude. Cover the delay line plus the prediction average. */
#define APP_ALC_K_SETTLE_BLOCKS 96u

typedef enum {
    ALC_TUNE_NONE = 0,
    ALC_TUNE_ATU,
    ALC_TUNE_SWR_SCAN
} AlcTuneOwner;

static volatile uint32_t s_limit_mw;
static volatile int32_t  s_g_slow_q15;
static volatile int32_t  s_g_fast_q15;
static int32_t           s_fast_clamp_q15;
static int32_t           s_swr_clamp_q15;
static int32_t           s_ext_alc_q15;
static volatile float    s_k_mod;
static volatile uint32_t s_gain_q15;

static uint32_t s_fast_hold;
static uint32_t s_swr_hold;
static uint32_t s_swr_attack_cooldown;
static volatile uint32_t s_fast_attacks;
static volatile uint32_t s_swr_attacks;

static volatile uint8_t  s_active_layer;
static volatile uint32_t s_slow_blocks;
static volatile uint32_t s_fast_blocks;

/* Low-power tune mode shared by the ATU and the SWR scan: forced tune setpoint,
 * saved user setpoint to restore, and the current owner so only the owner can
 * release it. */
static struct {
    volatile bool active;
    AlcTuneOwner  owner;
    uint32_t      saved_limit_mw;
    float         saved_k_mod;
} s_tune;

static uint8_t s_band_index;
static bool    s_band_valid;
static float   s_level_scale = 1.0f;
static float   s_power_corr = 1.0f;

static volatile float    s_k_est;
static volatile float    s_k_inst;
static volatile bool     s_k_valid;
static uint32_t          s_k_settle;
static volatile float    s_p_pred;
static volatile float    s_iq_pow_pk;
static volatile float    s_fwd_pep_w;
static volatile float    s_fwd_avg_w;
static volatile float    s_ref_avg_w;
static float             s_iq_pow_avg;
static float             s_pred_pow_avg;
static float             s_gain_avg;
static float             s_pred_pow_dly[APP_ALC_K_DELAY_BLOCKS];
static uint32_t          s_pred_pow_dly_pos;
static volatile float    s_tx_ramp;
static volatile bool     s_tx_ramp_active;

static uint32_t isqrt_u32(uint32_t x) {
    uint32_t res = 0u;
    uint32_t bit = 1u << 30;

    while (bit > x) {
        bit >>= 2;
    }
    while (bit != 0u) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* Move toward target with the fixed 1/3 attack rate. Only used above a limit,
 * where target is always below current: never increase the gain here. */
static int32_t alc_attack(int32_t current, int32_t target) {
    int32_t next;

    if (target >= current) {
        return current;
    }
    next = current - (current - target + APP_ALC_ATTACK_DIV - 1) /
                         APP_ALC_ATTACK_DIV;
    if (next < 0) {
        next = 0;
    }
    return next;
}

static uint32_t watts_to_mw(float w) {
    if (!(w > 0.0f)) {
        return 0u;
    }
    return (uint32_t)(w * 1000.0f);
}

static int32_t alc_total_gain(void) {
    int32_t gain;
    uint8_t layer;

    gain  = APP_ALC_Q15_ONE;
    layer = APP_ALC_LAYER_NONE;
    /* All layers are absolute gain limits, not multipliers: they never stack.
     * Track which layer produced the result for diagnostics. */
    if (s_g_slow_q15 < gain) {
        gain  = s_g_slow_q15;
        layer = APP_ALC_LAYER_SLOW;
    }
    if (s_g_fast_q15 < gain) {
        gain  = s_g_fast_q15;
        layer = APP_ALC_LAYER_FAST;
    }
    if (s_fast_clamp_q15 < gain) {
        gain  = s_fast_clamp_q15;
        layer = APP_ALC_LAYER_FAST_CLAMP;
    }
    /* The SWR clamp is skipped during a tune mode (ATU/SWR scan): a mismatched
     * network is the normal starting point and would otherwise strangle the tune
     * carrier. */
    if (!s_tune.active && s_swr_clamp_q15 < gain) {
        gain  = s_swr_clamp_q15;
        layer = APP_ALC_LAYER_SWR_CLAMP;
    }
    /* The external ALC line is a power-backoff command from the amplifier (a
     * lower voltage asks for more attenuation). Skipped during a tune mode, like
     * the SWR clamp: the tune carrier must ignore it. */
    if (!s_tune.active && s_ext_alc_q15 < gain) {
        gain  = s_ext_alc_q15;
        layer = APP_ALC_LAYER_EXT;
    }
    s_active_layer = layer;

    gain = (int32_t)((float)gain * s_tx_ramp);
    if (gain < 0) {
        gain = 0;
    } else if (gain > APP_ALC_Q15_ONE) {
        gain = APP_ALC_Q15_ONE;
    }
    return gain;
}

static uint8_t alc_level_from_gain(uint32_t gain_q15) {
    float level;

    if (gain_q15 == 0u) {
        return 0u;
    }

    /* 0.5 dB per step, 100 = unity gain (no limitation). */
    level = 100.0f + 40.0f * log10f((float)gain_q15 / 32768.0f);
    if (level <= 0.0f) {
        return 0u;
    }
    if (level >= 100.0f) {
        return 100u;
    }
    return (uint8_t)(level + 0.5f);
}

/* Per-band TX calibration: the amateur-band reference frequency, the drive
 * level offset in dBFS and the measured-power correction in dB. The band is
 * selected by the real TX frequency (not the BPF index: several bands share one
 * BPF filter), with boundaries at the arithmetic mean of adjacent frequencies.
 * The level is calibrated so the input-to-output transfer settles at
 * BOARD_TX_MAX_POWER_W on every band (which makes K band-independent); trim it
 * on hardware at the minimum supply voltage (ALC console mode: watch g_slow
 * reach 1.0). power_corr_db compensates the coupler/detector response droop
 * with frequency: it scales the measured forward and reflected power so the
 * reading matches the true power (identical tandem-match arms assumed). The
 * array is writable so the trims update it in RAM (no persistence);
 * s_band_default keeps the factory values for a reset. */
typedef struct {
    uint32_t freq_hz;
    float    level_db;
    float    power_corr_db;
} AlcBandCal;

static const AlcBandCal s_band_default[APP_ALC_BAND_COUNT] = {
    {  1900000u, -15.5f, 0.14f },
    {  3700000u, -20.5f, 0.14f },
    {  5360000u, -20.0f, 0.19f },
    {  7100000u, -19.5f, 0.23f },
    { 10100000u, -18.0f, 0.26f },
    { 14200000u, -18.0f, 0.23f },
    { 18100000u, -19.0f, 0.27f },
    { 21200000u, -19.0f, 0.35f },
    { 24900000u, -16.0f, 0.45f },
    { 28500000u, -12.0f, 0.63f },
    { 51000000u, -12.0f, 1.14f },
};

static AlcBandCal s_band[APP_ALC_BAND_COUNT];

void App_Alc_Init(void) {
    s_limit_mw        = 0u;
    s_g_slow_q15      = APP_ALC_Q15_ONE;
    s_g_fast_q15      = APP_ALC_Q15_ONE;
    s_fast_clamp_q15  = APP_ALC_Q15_ONE;
    s_swr_clamp_q15   = APP_ALC_Q15_ONE;
    s_ext_alc_q15     = APP_ALC_Q15_ONE;
    s_k_est           = APP_ALC_K_SEED;
    s_k_inst          = 0.0f;
    s_k_valid         = false;
    s_k_settle        = 0u;
    for (uint32_t idx = 0u; idx < APP_ALC_K_DELAY_BLOCKS; idx++) {
        s_pred_pow_dly[idx] = 0.0f;
    }
    s_pred_pow_dly_pos = 0u;
    s_p_pred          = 0.0f;
    s_iq_pow_pk       = 0.0f;
    s_fwd_pep_w       = 0.0f;
    s_fwd_avg_w       = 0.0f;
    s_ref_avg_w       = 0.0f;
    s_iq_pow_avg      = 0.0f;
    s_pred_pow_avg    = 0.0f;
    s_gain_avg        = 1.0f;
    s_tx_ramp         = 1.0f;
    s_tx_ramp_active  = false;
    s_k_mod           = 0.0f;
    s_gain_q15        = 0u;
    s_fast_hold           = 0u;
    s_swr_hold            = 0u;
    s_swr_attack_cooldown = 0u;
    s_fast_attacks    = 0u;
    s_swr_attacks     = 0u;
    s_active_layer    = APP_ALC_LAYER_NONE;
    s_slow_blocks     = 0u;
    s_fast_blocks     = 0u;
    s_tune.active      = false;
    s_tune.owner       = ALC_TUNE_NONE;
    s_tune.saved_limit_mw = 0u;
    s_tune.saved_k_mod    = 0.0f;
    s_band_index      = 0u;
    s_band_valid      = false;
    s_level_scale     = 1.0f;
    s_power_corr      = 1.0f;
    for (uint32_t idx = 0u; idx < APP_ALC_BAND_COUNT; idx++) {
        s_band[idx] = s_band_default[idx];
    }

    Drv_Fpga_SetTxGainQ15(0);
}

void App_Alc_SetPowerLimit(float txpwr_w) {
    float    k_mod = 0.0f;
    uint32_t irq;
    int32_t  gain;

    if (txpwr_w <= 0.0f) {
        txpwr_w = 0.0f;
    } else {
        k_mod = sqrtf(txpwr_w / BOARD_TX_MAX_POWER_W);
        if (k_mod > 1.0f) {
            k_mod = 1.0f;
        }
    }

    irq = Bsp_Irq_Save();
    if (s_tune.active) {
        /* During a tune mode the setpoint is forced; remember the user value so
         * it is restored when the mode ends. */
        s_tune.saved_limit_mw = (uint32_t)(txpwr_w * 1000.0f + 0.5f);
        s_tune.saved_k_mod    = k_mod;
        Bsp_Irq_Restore(irq);
        return;
    }
    s_limit_mw = (uint32_t)(txpwr_w * 1000.0f + 0.5f);
    s_k_mod    = k_mod;

    if (txpwr_w <= 0.0f) {
        /* No power requested: silence the output but keep the learned transfer
         * K and the trim/clamps, so a re-enable does not restart open-loop. K is
         * not updated while the setpoint is 0 (the k_mod > 0 gate). */
        s_gain_q15       = 0u;
        s_p_pred         = 0.0f;
        s_fwd_pep_w      = 0.0f;
        s_fwd_avg_w      = 0.0f;
        s_ref_avg_w      = 0.0f;
        s_tx_ramp        = 1.0f;
        s_tx_ramp_active = false;
        Drv_Fpga_SetTxGainQ15(0);
        Bsp_Irq_Restore(irq);
        return;
    }

    gain       = alc_total_gain();
    s_gain_q15 = (uint32_t)gain;
    Drv_Fpga_SetTxGainQ15(gain);
    Bsp_Irq_Restore(irq);
}

static void tune_enter(AlcTuneOwner owner, float power_w) {
    float k_mod = sqrtf(power_w / BOARD_TX_MAX_POWER_W);

    if (k_mod > 1.0f) {
        k_mod = 1.0f;
    }
    s_tune.saved_limit_mw = s_limit_mw;
    s_tune.saved_k_mod    = s_k_mod;
    s_limit_mw            = (uint32_t)(power_w * 1000.0f + 0.5f);
    s_k_mod               = k_mod;
    s_tune.owner          = owner;
    s_tune.active         = true;
}

static void tune_leave(void) {
    s_limit_mw    = s_tune.saved_limit_mw;
    s_k_mod       = s_tune.saved_k_mod;
    s_tune.owner  = ALC_TUNE_NONE;
    s_tune.active = false;
}

static void tune_apply_gain_locked(void) {
    int32_t gain = alc_total_gain();

    s_gain_q15 = (uint32_t)gain;
    Drv_Fpga_SetTxGainQ15(gain);
}

void App_Alc_SetAtuActive(bool active) {
    uint32_t irq = Bsp_Irq_Save();
    bool     changed;

    if (active) {
        changed = (s_tune.owner == ALC_TUNE_NONE);
        if (changed) {
            tune_enter(ALC_TUNE_ATU, APP_ATU_TUNE_POWER_W);
        }
    } else {
        changed = (s_tune.owner == ALC_TUNE_ATU);
        if (changed) {
            tune_leave();
        }
    }

    if (changed) {
        tune_apply_gain_locked();
    }
    Bsp_Irq_Restore(irq);
}

void App_Alc_SetSwrScanActive(bool active) {
    uint32_t irq = Bsp_Irq_Save();
    bool     changed;

    if (active) {
        changed = (s_tune.owner == ALC_TUNE_NONE);
        if (changed) {
            tune_enter(ALC_TUNE_SWR_SCAN, APP_ALC_SWR_SCAN_POWER_W);
        }
    } else {
        changed = (s_tune.owner == ALC_TUNE_SWR_SCAN);
        if (changed) {
            tune_leave();
        }
    }

    if (changed) {
        tune_apply_gain_locked();
    }
    Bsp_Irq_Restore(irq);
}

void App_Alc_ResetSwrClamp(void) {
    uint32_t irq = Bsp_Irq_Save();

    s_swr_clamp_q15       = APP_ALC_Q15_ONE;
    s_swr_hold            = 0u;
    s_swr_attack_cooldown = 0u;
    Bsp_Irq_Restore(irq);
}

/* Pure function: safe to call outside the IRQ lock. A frequency above the last
 * boundary maps to the terminal band (51 MHz). */
static uint8_t alc_band_from_freq(uint32_t freq_hz) {
    for (uint8_t i = 0u; i + 1u < APP_ALC_BAND_COUNT; i++) {
        uint32_t mid = (s_band[i].freq_hz + s_band[i + 1u].freq_hz) / 2u;

        if (freq_hz < mid) {
            return i;
        }
    }
    return APP_ALC_BAND_COUNT - 1u;
}

void App_Alc_NotifyFrequency(uint32_t freq_hz) {
    uint32_t irq;
    int32_t  gain;
    uint8_t  band = alc_band_from_freq(freq_hz);

    irq = Bsp_Irq_Save();
    if (s_band_valid && band == s_band_index) {
        /* QSY inside the same band: keep the learned transfer K, the level and
         * the per-transmission transients. */
        Bsp_Irq_Restore(irq);
        return;
    }

    /* The per-band level offset is the only band-dependent part. The transfer K
     * is band-independent by construction (the level calibration makes it
     * settle at BOARD_TX_MAX_POWER_W), so it is kept across a band change.
     * Only the per-transmission transients are reset. */
    s_band_index     = band;
    s_band_valid     = true;
    s_level_scale    = Sdr_Db2Lin(s_band[band].level_db);
    s_power_corr     = Sdr_Db2Lin(s_band[band].power_corr_db);
    s_p_pred         = 0.0f;
    s_fwd_avg_w      = 0.0f;
    s_ref_avg_w      = 0.0f;
    s_iq_pow_pk      = 0.0f;
    s_fast_hold           = 0u;
    s_swr_hold            = 0u;
    s_swr_attack_cooldown = 0u;

    if (s_limit_mw != 0u) {
        gain       = alc_total_gain();
        s_gain_q15 = (uint32_t)gain;
        Drv_Fpga_SetTxGainQ15(gain);
    }
    Bsp_Irq_Restore(irq);
}

float App_Alc_AdjustLevelDb(float delta_db) {
    uint32_t irq;
    float    level;

    /* delta_db == 0 reads the current value without changing it. */
    irq   = Bsp_Irq_Save();
    level = s_band[s_band_index].level_db + delta_db;
    if (level < APP_ALC_LEVEL_DB_MIN) {
        level = APP_ALC_LEVEL_DB_MIN;
    } else if (level > APP_ALC_LEVEL_DB_MAX) {
        level = APP_ALC_LEVEL_DB_MAX;
    }
    s_band[s_band_index].level_db = level;
    s_level_scale = Sdr_Db2Lin(level);
    Bsp_Irq_Restore(irq);

    return level;
}

float App_Alc_ResetLevelDb(void) {
    uint32_t irq;
    float    level;

    irq   = Bsp_Irq_Save();
    level = s_band_default[s_band_index].level_db;
    s_band[s_band_index].level_db = level;
    s_level_scale = Sdr_Db2Lin(level);
    Bsp_Irq_Restore(irq);

    return level;
}

float App_Alc_GetPowerCorr(void) {
    return s_power_corr;
}

float App_Alc_AdjustPowerCorrDb(float delta_db) {
    uint32_t irq;
    float    corr;

    /* delta_db == 0 reads the current value without changing it. */
    irq  = Bsp_Irq_Save();
    corr = s_band[s_band_index].power_corr_db + delta_db;
    if (corr < APP_ALC_POWER_CORR_DB_MIN) {
        corr = APP_ALC_POWER_CORR_DB_MIN;
    } else if (corr > APP_ALC_POWER_CORR_DB_MAX) {
        corr = APP_ALC_POWER_CORR_DB_MAX;
    }
    s_band[s_band_index].power_corr_db = corr;
    s_power_corr = Sdr_Db2Lin(corr);
    Bsp_Irq_Restore(irq);

    return corr;
}

float App_Alc_ResetPowerCorrDb(void) {
    uint32_t irq;
    float    corr;

    irq  = Bsp_Irq_Save();
    corr = s_band_default[s_band_index].power_corr_db;
    s_band[s_band_index].power_corr_db = corr;
    s_power_corr = Sdr_Db2Lin(corr);
    Bsp_Irq_Restore(irq);

    return corr;
}

void App_Alc_NotifyTxStart(void) {
    uint32_t irq;

    irq = Bsp_Irq_Save();
    /* Keep the learned transfer K, the slow gain and the input power average:
     * they are still valid, so the feed-forward limiter acts on the first block
     * and a CW/FM key-down starts at the configured level instead of unity. Only
     * the per-transmission transients are reset. The learned state is re-derived
     * on a band change (NotifyFrequency, by rescaling K) and kept across a 0 W
     * setpoint (SetPowerLimit). */
    s_tx_ramp        = 0.0f;
    s_tx_ramp_active = true;
    s_iq_pow_pk      = 0.0f;
    s_k_settle       = APP_ALC_K_SETTLE_BLOCKS;
    s_fast_hold           = 0u;
    s_swr_hold            = 0u;
    s_swr_attack_cooldown = 0u;
    s_gain_q15       = 0u;
    Drv_Fpga_SetTxGainQ15(0);
    Bsp_Irq_Restore(irq);
}

/* Runs in the ADC DMA ISR. Keep it short: no logs, no blocking calls. */
void App_Alc_Update(float fwd_peak_w, float fwd_pep_w, float fwd_avg_w,
                    float ref_fast_w, float ref_avg_w, uint16_t ext_alc_raw) {
    uint32_t fwd_peak_mw;
    uint32_t ref_fast_mw;
    uint32_t fast_limit_mw;
    uint32_t ext_raw;
    float    ext_corr;
    int32_t  gain;

    /* External ALC: turn the smoothed EXT ALC code into a power-backoff gain,
     * before the 0 W early return so the backoff keeps tracking while silent. A
     * code at or above the zero anchor means no backoff; a lower code asks for
     * more attenuation. The factor is a power multiplier, so the applied
     * amplitude is 1/sqrt(factor). */
    ext_raw = ext_alc_raw;
    if (ext_raw > APP_ALC_EXT_RAW_ZERO) {
        ext_raw = APP_ALC_EXT_RAW_ZERO;
    } else if (ext_raw < APP_ALC_EXT_RAW_MIN) {
        ext_raw = APP_ALC_EXT_RAW_MIN;
    }
    ext_corr = 1.0f + 0.005f * (float)(APP_ALC_EXT_RAW_ZERO - ext_raw);
    s_ext_alc_q15 =
        (int32_t)((float)APP_ALC_Q15_ONE / sqrtf(ext_corr) + 0.5f);

    if (s_limit_mw == 0u) {
        /* No power requested: the output stays silent regardless of the
         * measured power, but the learned transfer K and the trim/clamps are
         * kept so a re-enable does not restart open-loop. */
        s_p_pred         = 0.0f;
        s_fwd_pep_w      = 0.0f;
        s_fwd_avg_w      = 0.0f;
        s_ref_avg_w      = 0.0f;
        s_tx_ramp        = 1.0f;
        s_tx_ramp_active = false;
        s_active_layer   = APP_ALC_LAYER_NONE;
        s_gain_q15       = 0u;
        Drv_Fpga_SetTxGainQ15(0);
        return;
    }

    s_fwd_pep_w = fwd_pep_w;
    s_fwd_avg_w = fwd_avg_w;
    s_ref_avg_w = ref_avg_w;

    if (s_tx_ramp_active) {
        s_tx_ramp += 1.0f / (float)APP_ALC_TX_RAMP_STEPS;
        if (s_tx_ramp >= 1.0f) {
            s_tx_ramp        = 1.0f;
            s_tx_ramp_active = false;
        }
    }

    fwd_peak_mw = watts_to_mw(fwd_peak_w);
    ref_fast_mw = watts_to_mw(ref_fast_w);

    /* Layer 2: fast protection against a sudden forward-power overshoot. It
     * caps the peak at 1.15x the setpoint; the slow trim settles on the exact
     * setpoint. */
    fast_limit_mw =
        s_limit_mw + (s_limit_mw * (APP_ALC_FAST_LIMIT_PERCENT - 100u)) / 100u;
    if (fwd_peak_mw > fast_limit_mw) {
        /* One attack per hold window: the detector/ADC delay must catch up
         * before the next correction, otherwise the relative target compounds
         * and over-clamps on a key-up transient. */
        if (s_fast_hold != 0u) {
            s_fast_hold--;
        } else {
            uint32_t ratio_q16 =
                (uint32_t)(((uint64_t)fast_limit_mw << 16) / fwd_peak_mw);
            int32_t corr   = (int32_t)isqrt_u32(ratio_q16) << 7;
            int32_t target = (int32_t)(((int64_t)s_gain_q15 * corr) >> 15);

            s_fast_clamp_q15 = alc_attack(s_fast_clamp_q15, target);
            s_fast_hold      = APP_ALC_FAST_HOLD_SCANS;
            s_fast_attacks++;
        }
    } else if (s_fast_hold != 0u) {
        s_fast_hold--;
    } else if (fwd_peak_mw <
               (s_limit_mw * APP_ALC_FAST_HYST_PERCENT) / 100u) {
        s_fast_clamp_q15 += APP_ALC_RELEASE_STEP;
        if (s_fast_clamp_q15 > APP_ALC_Q15_ONE) {
            s_fast_clamp_q15 = APP_ALC_Q15_ONE;
        }
    }

    /* Layer 3: fast reflected-power clamp. It limits the *absolute* applied
     * gain, using the last applied gain and the fast reflected measurement, so
     * it does not multiply with the slow trim. The slow layer owns the steady
     * state through the forward ceiling; this clamp only bridges transients.
     * The release is gated by the input IQ power average (s_iq_pow_avg), not by
     * the measured forward power: while the clamp is active the output is
     * necessarily suppressed, so a forward-power release condition would
     * deadlock. */
    if (s_tune.active) {
        /* Tune mode: the SWR clamp layer is disabled (alc_total_gain skips it);
         * leave its state untouched so it is not wound down by a mismatch. */
    } else if (ref_fast_mw > APP_ALC_SWR_REF_LIMIT_MW) {
        s_swr_hold = APP_ALC_SWR_HOLD_SCANS;
        if (s_swr_attack_cooldown != 0u) {
            s_swr_attack_cooldown--;
        } else {
            uint32_t ratio_q16 = (uint32_t)(((uint64_t)APP_ALC_SWR_REF_LIMIT_MW
                                             << 16) /
                                            ref_fast_mw);
            int32_t corr   = (int32_t)isqrt_u32(ratio_q16) << 7;
            int32_t target = (int32_t)(((int64_t)s_gain_q15 * corr) >> 15);

            s_swr_clamp_q15       = alc_attack(s_swr_clamp_q15, target);
            s_swr_attack_cooldown = APP_ALC_SWR_ATTACK_COOLDOWN_SCANS;
            s_swr_attacks++;
        }
    } else if (s_swr_hold != 0u) {
        s_swr_hold--;
    } else if (ref_fast_mw < APP_ALC_SWR_RELEASE_HYST_MW &&
               s_iq_pow_avg > APP_ALC_SWR_RELEASE_POW_MIN) {
        s_swr_clamp_q15 += APP_ALC_SWR_RELEASE_STEP;
        if (s_swr_clamp_q15 > APP_ALC_Q15_ONE) {
            s_swr_clamp_q15 = APP_ALC_Q15_ONE;
        }
    }

    if (s_g_slow_q15 < APP_ALC_Q15_ONE) {
        s_slow_blocks++;
    }
    if (s_g_fast_q15 < APP_ALC_Q15_ONE) {
        s_fast_blocks++;
    }

    gain       = alc_total_gain();
    s_gain_q15 = (uint32_t)gain;
    Drv_Fpga_SetTxGainQ15(gain);
}

float App_Alc_GetOutputScale(const float *i, const float *q, uint32_t count) {
    float    peak_sq = 0.0f;
    float    sum_sq  = 0.0f;
    float    count_f;
    float    dt;
    float    iq_pow_pk;
    float    iq_pow_avg_blk;
    float    alpha;
    float    k_mod;
    float    gain;
    float    pred_pow_blk;
    float    pred_pow_dly;
    float    target_w;
    float    fwd;
    float    expected_w;
    float    g_slow;
    float    g_fast;
    float    p_pred;
    uint32_t idx;

    if (i == 0 || q == 0 || count == 0u) {
        return s_k_mod * s_level_scale;
    }

    for (idx = 0u; idx < count; ++idx) {
        float ii = i[idx];
        float qq = q[idx];
        float p  = ii * ii + qq * qq;

        if (p > peak_sq) {
            peak_sq = p;
        }
        sum_sq += p;
    }

    /* Everything below works in the power domain: iq_pow_pk is the block peak
     * power and iq_pow_avg_blk the block mean power, both read straight from
     * the IQ sums (i*i + q*q) with no sqrt. */
    count_f        = (float)count;
    iq_pow_pk      = peak_sq;
    iq_pow_avg_blk = sum_sq / count_f;
    dt             = count_f / (float)BOARD_AUDIO_SAMPLE_RATE_HZ;
    s_iq_pow_pk    = iq_pow_pk;

    alpha = 1.0f - expf(-dt / BOARD_TX_AVG_TAU_S);
    s_iq_pow_avg += alpha * (iq_pow_avg_blk - s_iq_pow_avg);

    /* Applied gain, averaged for diagnostics only; K uses the instantaneous
     * gain including the soft-start ramp (alc_total_gain). */
    gain = (float)s_gain_q15 / (float)APP_ALC_Q15_ONE;
    s_gain_avg += alpha * (gain - s_gain_avg);

    k_mod    = s_k_mod;
    target_w = (float)s_limit_mw * 0.001f;
    fwd      = s_fwd_avg_w;

    /* Predicted output power for this block, K excluded. Average the product
     * (block power * k_mod^2 * gain^2), not the factors: fwd_avg is an average
     * of power over the same window, and a product of averages would be held up
     * on a keying gap by the slow trim raising gain while the input falls. The
     * soft-start ramp is inside s_gain_q15, so it is accounted for here. */
    pred_pow_blk  = iq_pow_avg_blk * k_mod * k_mod * gain * gain;
    s_pred_pow_avg += alpha * (pred_pow_blk - s_pred_pow_avg);

    /* Transport delay: the measured fwd corresponds to the drive emitted
     * APP_ALC_K_DELAY_BLOCKS blocks ago. */
    pred_pow_dly = s_pred_pow_dly[s_pred_pow_dly_pos];
    s_pred_pow_dly[s_pred_pow_dly_pos] = s_pred_pow_avg;
    s_pred_pow_dly_pos = (s_pred_pow_dly_pos + 1u) % APP_ALC_K_DELAY_BLOCKS;

    /* Input-to-output transfer, a power ratio. Both sides are an average of
     * power over the same window, so a modulated envelope moves numerator and
     * denominator together and does not bias K. The update is taken only when
     * the commanded output (input level times setpoint) is above the detector
     * floor, so blocks with nothing measurable are skipped. A measured fwd below
     * the floor is clamped up to it: the true value is unknown but smaller, so
     * the clamped ratio can only pull K down. */
    expected_w = BOARD_TX_MAX_POWER_W * s_iq_pow_avg * k_mod * k_mod;
    if (s_k_settle != 0u) {
        s_k_settle--;
    }
    if (!s_tune.active && k_mod > 0.0f && expected_w > APP_ALC_K_MIN_FWD_W &&
        s_k_settle == 0u && pred_pow_dly > 0.0f) {
        float fwd_use = (fwd < APP_ALC_K_MIN_FWD_W) ? APP_ALC_K_MIN_FWD_W : fwd;
        float k_inst  = fwd_use / pred_pow_dly;
        float alpha_k = 1.0f - expf(-dt / APP_ALC_K_TAU_S);

        s_k_inst = k_inst;
        if (!s_k_valid) {
            s_k_est   = k_inst;
            s_k_valid = true;
        } else {
            s_k_est += alpha_k * (k_inst - s_k_est);
        }
    }

    /* g_slow is a pure gain correction that pulls the measured average output
     * to the commanded estimate P_cmd = BOARD_TX_MAX_POWER_W * k_mod^2 *
     * iq_pow_avg. With the per-band level calibrated the transfer K settles at
     * BOARD_TX_MAX_POWER_W on every band, so g_slow = sqrt(MAX/K) is the
     * residual trim (calibration error, temperature, supply). It is independent
     * of the setpoint and the input level and never exceeds 1: a transfer above
     * nominal (more supply) is attenuated, a lower one is not boosted. The fast
     * limiter caps the instantaneous block peak at APP_ALC_FF_LIMIT_RATIO of
     * the setpoint through the same model. */
    g_slow = 1.0f;
    g_fast = 1.0f;
    /* s_k_est holds the conservative seed until the first measurement, so the
     * correction is active from the first block (k_valid only marks whether it
     * is measured yet). */
    if (s_k_est > 0.0f && k_mod > 0.0f) {
        g_slow = sqrtf(BOARD_TX_MAX_POWER_W / s_k_est);
        if (iq_pow_pk > APP_ALC_MIN_IQ_POW && target_w > 0.0f) {
            float denom = s_k_est * iq_pow_pk * k_mod * k_mod;

            if (denom > 0.0f) {
                g_fast = sqrtf(APP_ALC_FF_LIMIT_RATIO * target_w / denom);
            }
        }
    }
    if (!(g_slow > 0.0f)) {
        g_slow = 0.0f;
    } else if (g_slow > 1.0f) {
        g_slow = 1.0f;
    }
    if (!(g_fast > 0.0f)) {
        g_fast = 0.0f;
    } else if (g_fast > 1.0f) {
        g_fast = 1.0f;
    }
    s_g_slow_q15 = (int32_t)(g_slow * (float)APP_ALC_Q15_ONE + 0.5f);
    s_g_fast_q15 = (int32_t)(g_fast * (float)APP_ALC_Q15_ONE + 0.5f);

    /* Predict the PEP for this block before it leaves for the FPGA. K is slow,
     * iq_pow_pk is the instantaneous block peak power. */
    p_pred = 0.0f;
    if (s_k_est > 0.0f && k_mod > 0.0f && iq_pow_pk > APP_ALC_MIN_IQ_POW &&
        target_w > 0.0f) {
        p_pred = s_k_est * iq_pow_pk * k_mod * k_mod * gain * gain;
    }
    s_p_pred = p_pred;

    return k_mod * s_level_scale;
}

uint8_t App_Alc_GetLevel(void) {
    return alc_level_from_gain(s_gain_q15);
}

void App_Alc_GetSnapshot(AppAlcSnapshot *out) {
    uint32_t irq;

    if (out == 0) {
        return;
    }

    irq = Bsp_Irq_SaveLow();
    out->limit_mw       = s_limit_mw;
    out->k_mod          = s_k_mod;
    out->g_slow         = (float)s_g_slow_q15 / (float)APP_ALC_Q15_ONE;
    out->g_fast         = (float)s_g_fast_q15 / (float)APP_ALC_Q15_ONE;
    out->fast_clamp_q15 = s_fast_clamp_q15;
    out->swr_clamp_q15  = s_swr_clamp_q15;
    out->gain_q15       = (int32_t)s_gain_q15;
    out->iq_pow_pk      = s_iq_pow_pk;
    out->iq_pow_avg     = s_iq_pow_avg;
    out->gain_avg       = s_gain_avg;
    out->fwd_avg_w      = s_fwd_avg_w;
    out->k_est          = s_k_est;
    out->k_inst         = s_k_inst;
    out->k_valid        = s_k_valid;
    out->p_pred         = s_p_pred;
    out->tx_ramp        = s_tx_ramp;
    out->fast_attacks   = s_fast_attacks;
    out->swr_attacks    = s_swr_attacks;
    out->active_layer   = s_active_layer;
    out->slow_blocks    = s_slow_blocks;
    out->fast_blocks    = s_fast_blocks;
    Bsp_Irq_RestoreLow(irq);

    /* log10f is not cheap: keep it outside the IRQ-locked section. */
    out->level = alc_level_from_gain((uint32_t)out->gain_q15);
}
