#include "sdr_agc.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "app_event.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_codec.h"
#include "sdr_am.h"
#include "sdr_common.h"
#include "sdr_config.h"
#include "sdr_spectrum.h"

#define LOG_TAG "app.sdr.agc"

#define SDR_AGC_SAMPLE_RATE_DIVISOR 2u

#define SDR_AGC_SAMPLE_RATE_HZ \
    (BOARD_AUDIO_SAMPLE_RATE_HZ / SDR_AGC_SAMPLE_RATE_DIVISOR)
#define SDR_AGC_BLOCK_SIZE \
    (DRV_CODEC_BLOCK_FRAMES / SDR_AGC_SAMPLE_RATE_DIVISOR)
#define SDR_AGC_BLOCK_RATE \
    ((float)SDR_AGC_SAMPLE_RATE_HZ / (float)SDR_AGC_BLOCK_SIZE)
#define SDR_AGC_MAX_DELAY_BLOCKS    16u
#define SDR_AGC_FADE_BLOCKS         8u

#define SDR_AGC_TARGET_DB           (0.0f)
#define SDR_AGC_NOISE_DELTA_DB      (16.0f)
#define SDR_AGC_SLOPE               (0.1f)
#define SDR_AGC_GAIN_MIN_DB         (-20.0f)
#define SDR_AGC_GAIN_MAX_DB         (110.0f)
#define SDR_AGC_NOISE_RISE_S        (10.0f)
#define SDR_AGC_NOISE_FALL_S        (1.0f)
#define SDR_AGC_FAST_DECAY_DB_PER_S (1500.0f)
#define SDR_AGC_GAIN_FLOOR          (1e-5f)
#define SDR_AGC_OFF_GAIN_DB         (70.0f)
#define SDR_AGC_MIN_NOISE_DB        (-240.0f)

/* Per-block AGC state log for on-device measurements; off in normal builds. */
#define SDR_AGC_DEBUG_LOG 0


enum sdr_agc_mode_t {
    SDR_AGC_CW,
    SDR_AGC_SSB,
    SDR_AGC_AM,
    SDR_AGC_GROUP_COUNT
};

enum sdr_agc_speed_t {
    SDR_AGC_SPEED_FAST,
    SDR_AGC_SPEED_SLOW,
    SDR_AGC_SPEED_COUNT
};

typedef struct {
    float    attack_ms;
    float    hold_ms;
    float    decay_db_s;
    float    attack_alpha;
    uint32_t hold_blocks;
    float    decay_step_db;
    uint32_t delay_blocks;
} SdrAgcTiming;

/* One entry per modulation group and AGC speed. AM ignores the speed setting. */
static SdrAgcTiming s_timing[SDR_AGC_GROUP_COUNT][SDR_AGC_SPEED_COUNT];

static float    s_off_gain_lin;
static float    s_env_db;
static float    s_noise_db;
static float    s_current_gain_lin;
static uint32_t s_hold_remaining;
static uint32_t s_peak_duration_blocks;
static uint32_t s_delay_pos;
static float    s_delay[SDR_AGC_MAX_DELAY_BLOCKS][SDR_AGC_BLOCK_SIZE]
    __attribute__((aligned(32)));
static float    s_input[SDR_AGC_BLOCK_SIZE] __attribute__((aligned(32)));
static float    s_alpha_noise_up;
static float    s_alpha_noise_down;
static float    s_fast_decay_step_db;
static bool     s_fast_decay_active;

/* Modulation group of the active timing (SDR_AGC_CW/SSB/AM). */
static uint32_t s_group;

/* Configuration published to the audio ISR. Writes are protected by masking
 * interrupts in agc_configure(); the processing side runs in the audio
 * callback (I2S DMA, priority 1) and only reads these fields. Bsp_Irq_SaveLow()
 * is enough: the FPGA PE10 tick runs at priority 0 and never touches AGC
 * state, so it must not be held off while a configuration is published. */
static bool                  s_agc_enabled;
static const SdrAgcTiming   *s_timing_cur;
static uint32_t              s_delay_cur;
static uint32_t              s_delay_target;
static uint32_t              s_fade_remaining;
static bool                  s_fade_bypass;

/* Last published configuration: rx_mode arrives through SdrRxCfg and agc_mode
 * through Sdr_AGC_SetMode, so the timing is reapplied when either changes. */
static SdrModulation s_cfg_modulation = SDR_MOD_LSB;
static SdrAgcMode    s_cfg_mode       = SDR_AGC_OFF;

static void agc_configure(SdrModulation modulation, SdrAgcMode mode);
static void on_rx_cfg(const SdrRxCfg *cfg, void *user);

static float agc_alpha_from_blocks(float blocks)
{
    if (blocks <= 0.0f) {
        return 1.0f;
    }
    return 1.0f - expf(-1.0f / blocks);
}

static void agc_timing_init(SdrAgcTiming *t, float attack_ms, float hold_ms,
                            float decay_db_s)
{
    float attack_blocks = attack_ms * SDR_AGC_BLOCK_RATE / 1000.0f;
    uint32_t delay = (uint32_t)ceilf(attack_blocks);

    t->attack_ms    = attack_ms;
    t->hold_ms      = hold_ms;
    t->decay_db_s   = decay_db_s;
    t->attack_alpha = agc_alpha_from_blocks(attack_blocks);
    t->hold_blocks  = (uint32_t)(hold_ms * SDR_AGC_BLOCK_RATE / 1000.0f + 0.5f);
    t->decay_step_db = decay_db_s / SDR_AGC_BLOCK_RATE;

    if (delay < 1u) {
        delay = 1u;
    } else if (delay > SDR_AGC_MAX_DELAY_BLOCKS) {
        delay = SDR_AGC_MAX_DELAY_BLOCKS;
    }
    t->delay_blocks = delay;
}

void Sdr_AGC_Init(void)
{
    agc_timing_init(&s_timing[SDR_AGC_CW][SDR_AGC_SPEED_FAST],
                    1.0f, 25.0f, 75.0f);
    agc_timing_init(&s_timing[SDR_AGC_CW][SDR_AGC_SPEED_SLOW],
                    2.0f, 250.0f, 25.0f);

    agc_timing_init(&s_timing[SDR_AGC_SSB][SDR_AGC_SPEED_FAST],
                    2.0f, 50.0f, 40.0f);
    agc_timing_init(&s_timing[SDR_AGC_SSB][SDR_AGC_SPEED_SLOW],
                    5.0f, 1500.0f, 8.0f);

    /* AM ignores the speed setting. */
    agc_timing_init(&s_timing[SDR_AGC_AM][SDR_AGC_SPEED_FAST],
                    15.0f, 50.0f, 6.0f);
    agc_timing_init(&s_timing[SDR_AGC_AM][SDR_AGC_SPEED_SLOW],
                    15.0f, 50.0f, 6.0f);

    s_off_gain_lin = Sdr_Db2Lin(SDR_AGC_OFF_GAIN_DB);

    s_env_db             = -100.0f;
    s_noise_db           = 0.0f;
    s_current_gain_lin   = 1.0f;
    s_hold_remaining     = 0u;
    s_peak_duration_blocks = 0u;
    s_delay_pos          = 0u;
    s_alpha_noise_up        = agc_alpha_from_blocks(SDR_AGC_NOISE_RISE_S *
                                                 SDR_AGC_BLOCK_RATE);
    s_alpha_noise_down        = agc_alpha_from_blocks(SDR_AGC_NOISE_FALL_S *
                                                 SDR_AGC_BLOCK_RATE);
    s_fast_decay_step_db = SDR_AGC_FAST_DECAY_DB_PER_S / SDR_AGC_BLOCK_RATE;
    s_fast_decay_active  = false;

    s_group = 0u;

    s_agc_enabled  = false;
    s_timing_cur   = NULL;
    s_delay_cur    = 0u;
    s_delay_target = 0u;
    s_fade_remaining = 0u;
    s_fade_bypass  = false;

    memset(s_delay, 0, sizeof(s_delay));

    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
}

static void agc_configure(SdrModulation modulation, SdrAgcMode mode)
{
    uint32_t group;
    uint32_t speed;
    bool     supported = true;
    bool     enabled;
    const SdrAgcTiming *timing;
    uint32_t basepri;

    switch (modulation) {
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        group = SDR_AGC_CW;
        break;
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        group = SDR_AGC_SSB;
        break;
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        group = SDR_AGC_AM;
        break;
    default:
        group = 0u;
        supported = false;
        break;
    }

    speed = (mode == SDR_AGC_FAST) ? SDR_AGC_SPEED_FAST : SDR_AGC_SPEED_SLOW;
    enabled = supported && (mode != SDR_AGC_OFF);
    timing = enabled ? &s_timing[group][speed] : NULL;

    basepri = Bsp_Irq_SaveLow();

    if (enabled == s_agc_enabled && (!enabled || timing == s_timing_cur)) {
        Bsp_Irq_RestoreLow(basepri);
        return;
    }

    bool was_enabled = s_agc_enabled;
    s_agc_enabled = enabled;
    s_group       = group;

    if (enabled) {
        s_timing_cur = timing;

        /* Keep the state, but make the new hold setting effective. */
        if (s_hold_remaining > timing->hold_blocks) {
            s_hold_remaining = timing->hold_blocks;
        }

        if (!was_enabled) {
            /* Crossfade from bypassed audio into the delayed AGC output. */
            s_delay_cur      = 0u;
            s_delay_target   = timing->delay_blocks;
            s_fade_remaining = SDR_AGC_FADE_BLOCKS;
            s_fade_bypass    = true;
        } else if (s_delay_cur != timing->delay_blocks) {
            s_delay_target   = timing->delay_blocks;
            s_fade_remaining = SDR_AGC_FADE_BLOCKS;
            s_fade_bypass    = false;
        } else {
            s_delay_target   = timing->delay_blocks;
            s_fade_remaining = 0u;
            s_fade_bypass    = false;
        }
    } else {
        s_timing_cur     = NULL;
        s_delay_cur      = 0u;
        s_delay_target   = 0u;
        s_fade_remaining = 0u;
        s_fade_bypass    = false;
    }

    Bsp_Irq_RestoreLow(basepri);
}

void Sdr_AGC_SetMode(SdrAgcMode mode)
{
    s_cfg_mode = mode;
    agc_configure(s_cfg_modulation, s_cfg_mode);
}

static void on_rx_cfg(const SdrRxCfg *cfg, void *user)
{
    (void)user;
    if (cfg->rx_mode == s_cfg_modulation) {
        return;
    }
    s_cfg_modulation = cfg->rx_mode;
    agc_configure(s_cfg_modulation, s_cfg_mode);
}

void Sdr_AGC_ResetStats(void)
{
    uint32_t basepri = Bsp_Irq_SaveLow();

    s_peak_duration_blocks = 0u;

    Bsp_Irq_RestoreLow(basepri);
}

void Sdr_AGC_Process(float *samples, uint32_t size)
{
    float peak = 0.0f;
    float lvl_db;
    float noise_src;
    float threshold_db;
    float total_gain_db;
    float target_gain_lin;
    float gain_step_lin;
    const float *tap_cur;
    const float *tap_tgt = NULL;
    bool fade;
    float w = 0.0f;
    uint32_t pos;
    uint32_t i;

    if (samples == 0 || size != SDR_AGC_BLOCK_SIZE) {
        return;
    }

    /* Keep the original block for the delay line even when AGC is bypassed. */
    arm_copy_f32(samples, s_input, size);

    pos = s_delay_pos;

    if (!s_agc_enabled) {
        arm_copy_f32(s_input, s_delay[pos], size);
        s_delay_pos = (pos + 1u) % SDR_AGC_MAX_DELAY_BLOCKS;
        arm_scale_f32(samples, s_off_gain_lin, samples, size);
        return;
    }

    const SdrAgcTiming *t = s_timing_cur;

    for (i = 0u; i < size; i++) {
        float mag = fabsf(s_input[i]);
        if (mag > peak) {
            peak = mag;
        }
    }

    if (s_group == SDR_AGC_AM) {
        /* The carrier (the DC the demod removes) tracks the signal independently
         * of modulation. When the SAM PLL has not locked it reads too low, so
         * fall back to a fraction of the audio peak (which cannot exceed the
         * carrier for AM). */
        peak = fmaxf(Sdr_AM_GetCarrierLevel(), 0.8f * peak);
    }

    lvl_db = Sdr_Lin2Db(peak);
    if (lvl_db < SDR_AGC_MIN_NOISE_DB) lvl_db = SDR_AGC_MIN_NOISE_DB;

    /* Noise reference: the spectrum estimate (independent of the demodulator).
     * While it is invalid (in TX, during a retune holdoff, or the first frame
     * after a generation change) keep the last value instead of updating. */
    if (Sdr_Spectrum_Is_Noise_Valid()) {
        noise_src = Sdr_Spectrum_Get_Demod_Noise_Est_Db();

        if (noise_src < s_noise_db) {
            s_noise_db += (noise_src - s_noise_db) * s_alpha_noise_down;
        } else {
            s_noise_db += (noise_src - s_noise_db) * s_alpha_noise_up;
        }
    }

    threshold_db = s_noise_db + SDR_AGC_NOISE_DELTA_DB;

    if (s_env_db > threshold_db) {
        s_peak_duration_blocks++;
    }

    if (lvl_db > s_env_db) {
        s_env_db += (lvl_db - s_env_db) * t->attack_alpha;
        s_hold_remaining = t->hold_blocks;
    } else {
        /* A short peak gets a fast recovery; a longer burst uses normal decay. */
        if (lvl_db < s_env_db) {
            if (s_hold_remaining == t->hold_blocks) {
                if (s_peak_duration_blocks <= 2u) {
                    s_hold_remaining    = 0u;
                    s_fast_decay_active = true;
                } else {
                    s_fast_decay_active = false;
                }
            }
            if (lvl_db <= threshold_db) {
                s_peak_duration_blocks = 0u;
            }
        }
        if (s_hold_remaining > 0u) {
            s_hold_remaining--;
        } else {
            s_env_db -= s_fast_decay_active ? s_fast_decay_step_db :
                                             t->decay_step_db;
        }
    }

    if (s_env_db <= threshold_db) {
        total_gain_db = SDR_AGC_TARGET_DB - threshold_db;
    } else {
        total_gain_db = SDR_AGC_TARGET_DB - s_env_db +
                        SDR_AGC_SLOPE * (s_env_db - threshold_db);
    }

    if (total_gain_db < SDR_AGC_GAIN_MIN_DB) {
        total_gain_db = SDR_AGC_GAIN_MIN_DB;
    } else if (total_gain_db > SDR_AGC_GAIN_MAX_DB) {
        total_gain_db = SDR_AGC_GAIN_MAX_DB;
    }

    if (s_current_gain_lin < SDR_AGC_GAIN_FLOOR) {
        s_current_gain_lin = SDR_AGC_GAIN_FLOOR;
    }

#if SDR_AGC_DEBUG_LOG
    static uint32_t cnt = 0u;
    if (++cnt >= 128u) {
        cnt = 0u;
        LOG_INFO("sig_level: %f, tracked_noise: %f, env: %f, gain: %f",
                 lvl_db, s_noise_db, s_env_db, total_gain_db);
    }
#endif

    target_gain_lin = Sdr_Db2Lin(total_gain_db);
    gain_step_lin = powf(target_gain_lin / s_current_gain_lin,
                         1.0f / (float)size);

    if (s_delay_cur == 0u) {
        tap_cur = s_input;
    } else {
        tap_cur = s_delay[(pos + SDR_AGC_MAX_DELAY_BLOCKS - s_delay_cur) %
                          SDR_AGC_MAX_DELAY_BLOCKS];
    }

    fade = (s_fade_remaining > 0u) && (s_delay_cur != s_delay_target);
    if (fade) {
        if (s_delay_target == 0u) {
            tap_tgt = s_input;
        } else {
            tap_tgt = s_delay[(pos + SDR_AGC_MAX_DELAY_BLOCKS - s_delay_target) %
                              SDR_AGC_MAX_DELAY_BLOCKS];
        }
        uint32_t elapsed = SDR_AGC_FADE_BLOCKS - s_fade_remaining + 1u;
        w = (float)elapsed / (float)SDR_AGC_FADE_BLOCKS;
        if (w > 1.0f) {
            w = 1.0f;
        }
    }

    for (i = 0u; i < size; i++) {
        s_current_gain_lin *= gain_step_lin;
        if (s_current_gain_lin < SDR_AGC_GAIN_FLOOR) {
            s_current_gain_lin = SDR_AGC_GAIN_FLOOR;
        }

        if (fade) {
            if (s_fade_bypass) {
                /* Bypass -> AGC: keep the direct signal out of the gain path. */
                float bypass = s_input[i];
                float agc    = tap_tgt[i] * s_current_gain_lin;
                samples[i] = bypass * (1.0f - w) + agc * w;
            } else {
                samples[i] = (tap_cur[i] * (1.0f - w) +
                              tap_tgt[i] * w) * s_current_gain_lin;
            }
        } else {
            samples[i] = tap_cur[i] * s_current_gain_lin;
        }
    }

    arm_copy_f32(s_input, s_delay[pos], size);
    s_delay_pos = (pos + 1u) % SDR_AGC_MAX_DELAY_BLOCKS;

    if (fade) {
        s_fade_remaining--;
        if (s_fade_remaining == 0u) {
            s_delay_cur   = s_delay_target;
            s_fade_bypass = false;
        }
    }
}
