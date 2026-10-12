#include "sdr_rx.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_event.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_codec.h"
#include "drv_fpga.h"
#include "sdr_agc.h"
#include "sdr_am.h"
#include "sdr_audio_resample.h"
#include "sdr_cw.h"
#include "sdr_nbfm.h"
#include "sdr_notch.h"
#include "sdr_nr.h"
#include "sdr_resample.h"
#include "sdr_spectrum.h"
#include "sdr_squelch.h"
#include "sdr_ssb.h"
#include "sdr_wfm.h"
#include "sdr_nb.h"

#define LOG_TAG "app.sdr.rx"

/*
 * RX I/Q frontend. The FPGA streams M = 8 >> fftdec samples per 48 kHz tick, so
 * one I2S block (64 frames) carries BLOCK = 64 * M complex samples. The raw ring
 * is read one block behind the write index by default; a producer slower than
 * the 48 kHz tick shrinks that lag and is compensated one sample at a time. One
 * block of lag fits the ring for every fftdec (max 512 of 1024); two blocks
 * would alias to delta 0 at fftdec = 0.
 */
#define SDR_RX_IQ_LAG_BLOCKS   1u
#define SDR_RX_IQ_RING_MASK    (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u)

/* Ignore this many 48 kHz audio frames (FPGA ticks) of raw I/Q after an fftdec
 * generation change, then fade the raw I/Q back in over the following block. */
#define SDR_RX_FFTDEC_SKIP_FRAMES 96u

#define SDR_RX_AUDIO_48_KHZ BOARD_AUDIO_SAMPLE_RATE_HZ
#define SDR_RX_AUDIO_24_KHZ (BOARD_AUDIO_SAMPLE_RATE_HZ / 2u)
#define SDR_RX_AUDIO_12_KHZ (BOARD_AUDIO_SAMPLE_RATE_HZ / 4u)
#define SDR_RX_AUDIO_BLOCK_MAX 256u

#define Q31_SCALE (2147483648.0f)

/* Audio stage that follows demodulation.  All non-WFM modes converge on the
 * 24 kHz common chain and are interpolated to 48 kHz just before DMA. */
typedef enum {
    SDR_RX_AUDIO_PRE_NONE = 0,    /* AM/SAM: demod already at 24 kHz */
    SDR_RX_AUDIO_PRE_DECIM_48_24,    /* NFM */
    SDR_RX_AUDIO_PRE_INTERP_12_24,    /* SSB/CW */
} SdrRxAudioPre;

typedef enum {
    SDR_RX_AUDIO_POST_NONE = 0,   /* WFM: already 48 kHz */
    SDR_RX_AUDIO_POST_INTERP_24_48,  /* all non-WFM modes */
} SdrRxAudioPost;

typedef struct {
    uint32_t        rate_hz; /* IQ target and audio rate after demodulation */
    SdrRxAudioPre   pre;
    SdrRxAudioPost  post;
    bool            common_chain;
} SdrRxAudioPlan;

static const SdrRxAudioPlan k_rx_audio_plans[SDR_MOD_WFM + 1u] = {
    [SDR_MOD_LSB] = {
        SDR_RX_AUDIO_12_KHZ, SDR_RX_AUDIO_PRE_INTERP_12_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_LSB_D] = {
        SDR_RX_AUDIO_12_KHZ, SDR_RX_AUDIO_PRE_INTERP_12_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_USB] = {
        SDR_RX_AUDIO_12_KHZ, SDR_RX_AUDIO_PRE_INTERP_12_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_USB_D] = {
        SDR_RX_AUDIO_12_KHZ, SDR_RX_AUDIO_PRE_INTERP_12_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_CW] = {
        SDR_RX_AUDIO_12_KHZ, SDR_RX_AUDIO_PRE_INTERP_12_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_CWR] = {
        SDR_RX_AUDIO_12_KHZ, SDR_RX_AUDIO_PRE_INTERP_12_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_AM] = {
        SDR_RX_AUDIO_24_KHZ, SDR_RX_AUDIO_PRE_NONE,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_SAM] = {
        SDR_RX_AUDIO_24_KHZ, SDR_RX_AUDIO_PRE_NONE,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    [SDR_MOD_NFM] = {
        SDR_RX_AUDIO_48_KHZ, SDR_RX_AUDIO_PRE_DECIM_48_24,
        SDR_RX_AUDIO_POST_INTERP_24_48, true,
    },
    /* WFM: IQ at 192 kHz; Sdr_WFM_Demod resamples to 48 kHz internally. */
    [SDR_MOD_WFM] = {
        BOARD_AUDIO_SAMPLE_RATE_HZ * 4u, SDR_RX_AUDIO_PRE_NONE,
        SDR_RX_AUDIO_POST_NONE, false,
    },
};

static const SdrRxAudioPlan *s_audio_plan;
static uint32_t              s_audio_src_hz;
static bool                  s_audio_plan_valid;

/* Committed RX configuration. A change that bumps the FPGA raw generation is
 * staged in s_pending_cfg and applied, in step with the hardware, from the I2S
 * DMA ISR on the generation change. s_rx_cfg_published gates the mandatory
 * first publish even when the initial snapshot equals the module defaults. */
static SdrRxCfg s_rx_cfg = { SDR_MOD_LSB, 150, 2900, 0 };
static bool     s_rx_cfg_published;
static SdrRxCfg s_pending_cfg;
static bool     s_pending_config_valid;

/* Driver-owned constant view; only the buffer contents are live. */
static const DrvFpgaRawWindow *iq_data;

static bool     s_iq_started;
static uint32_t s_iq_r;
static uint32_t s_raw_gen_seen;
static uint32_t s_skip_frames;
static bool     s_fade_pending;
static float    s_iq_i[DRV_CODEC_BLOCK_FRAMES * 8] __attribute__((aligned(32)));
static float    s_iq_q[DRV_CODEC_BLOCK_FRAMES * 8] __attribute__((aligned(32)));
static float    s_demod_raw[SDR_RX_AUDIO_BLOCK_MAX] __attribute__((aligned(32)));
static float    s_audio24[SDR_AUDIO_BLOCK_24K] __attribute__((aligned(32)));
static float    s_audio48[DRV_CODEC_BLOCK_FRAMES] __attribute__((aligned(32)));

static SdrModulation s_mode = SDR_MOD_LSB;

static bool rate_hz_allowed(uint32_t rate_hz) {
    return rate_hz == BOARD_AUDIO_SAMPLE_RATE_HZ * 8u ||
           rate_hz == BOARD_AUDIO_SAMPLE_RATE_HZ * 4u ||
           rate_hz == BOARD_AUDIO_SAMPLE_RATE_HZ * 2u ||
           rate_hz == BOARD_AUDIO_SAMPLE_RATE_HZ ||
           rate_hz == BOARD_AUDIO_SAMPLE_RATE_HZ / 2u ||
           rate_hz == BOARD_AUDIO_SAMPLE_RATE_HZ / 4u;
}

static uint32_t source_rate_hz(uint8_t fftdec) {
    if (fftdec > DRV_FPGA_FFTDEC_MAX) {
        fftdec = DRV_FPGA_FFTDEC_MAX;
    }
    uint32_t m = DRV_FPGA_SAMPLES_PER_TICK_BASE >> fftdec;
    return BOARD_AUDIO_SAMPLE_RATE_HZ * m;
}

static const SdrRxAudioPlan *audio_plan_for_mode(SdrModulation mode) {
    if ((unsigned)mode >= (sizeof(k_rx_audio_plans) / sizeof(k_rx_audio_plans[0]))) {
        return NULL;
    }
    if (k_rx_audio_plans[mode].rate_hz == 0u) {
        return NULL;
    }
    return &k_rx_audio_plans[mode];
}

static bool rates_valid(SdrModulation mode, uint8_t fftdec) {
    const SdrRxAudioPlan *plan = audio_plan_for_mode(mode);
    uint32_t src;

    if (plan == NULL) {
        return false;
    }

    src = source_rate_hz(fftdec);
    if (!rate_hz_allowed(src) || !rate_hz_allowed(plan->rate_hz)) {
        return false;
    }
    return src >= plan->rate_hz;
}

static void radio_convert(uint32_t from, uint32_t count) {
    const float k = 1.0f / Q31_SCALE;
    for (uint32_t n = 0; n < count; ++n) {
        uint32_t idx = (from + n) & SDR_RX_IQ_RING_MASK;

        s_iq_i[n] = (float)iq_data->i[idx] * k;
        s_iq_q[n] = (float)iq_data->q[idx] * k;
    }
}

/* Fade a freshly converted raw I/Q block from 0 to 1. Runs only on the first
 * block after the fftdec skip window, so the source is attenuated before the
 * decimation and demodulation stages. `full` is the nominal block size and is
 * used as the ramp denominator so the last sample reaches full scale. */
static void radio_apply_fade(uint32_t count, uint32_t full) {
    if (count == 0u || full < 2u) {
        return;
    }
    for (uint32_t n = 0; n < count; ++n) {
        float gain = (float)n / (float)(full - 1u);

        s_iq_i[n] *= gain;
        s_iq_q[n] *= gain;
    }
}

/* RX audio DSP hook for the common 24 kHz chain.  Notch and noise reduction
 * will run here once implemented.  AGC is bypassed for FM modes. */
static void rx_audio_process(float *samples, uint32_t size,
                             SdrModulation mode) {
    if (mode == SDR_MOD_NFM || mode == SDR_MOD_WFM) {
        return;
    }
    Sdr_Notch_Process(samples, size);
    Sdr_NR_Process(samples, size);
    Sdr_AGC_Process(samples, size);
}

/* Apply a validated resample configuration. Callable from the I2S DMA ISR:
 * Sdr_Resample_SetRates() protects its own state with Bsp_Irq_SaveLow, and the
 * plan/rate snapshot is swapped under the same mask. No logs, no blocking.
 * The audio-filter reset is deliberately outside the mask: it only needs to be
 * ordered after the plan swap and must not delay the FPGA PE10 tick. */
static void apply_resample_config(SdrModulation mode, uint8_t fftdec) {
    const SdrRxAudioPlan *plan = audio_plan_for_mode(mode);
    uint32_t src;
    uint32_t key;
    bool changed;

    if (plan == NULL || !rates_valid(mode, fftdec)) {
        return;
    }

    src = source_rate_hz(fftdec);
    Sdr_Resample_SetRates(src, plan->rate_hz);

    key = Bsp_Irq_SaveLow();
    changed = !s_audio_plan_valid || s_audio_plan != plan ||
              s_audio_src_hz != src;
    if (changed) {
        s_audio_plan      = plan;
        s_audio_src_hz    = src;
        s_audio_plan_valid = true;
    }
    Bsp_Irq_RestoreLow(key);
}

bool Sdr_Rx_ClampFftdec(SdrModulation mode, uint8_t *fftdec) {
    if (fftdec == 0 || audio_plan_for_mode(mode) == NULL) {
        return false;
    }
    while (!rates_valid(mode, *fftdec)) {
        if (*fftdec == 0u) {
            return false;
        }
        (*fftdec)--;
    }
    return true;
}

/* Commit a snapshot as the running configuration and publish it. Callable from
 * the I2S DMA ISR (the pending commit) and from task context (the immediate
 * apply). */
static void rx_cfg_commit(const SdrRxCfg *cfg) {
    bool     mode_changed = cfg->rx_mode != s_mode;
    uint32_t key = Bsp_Irq_SaveLow();

    bool     fftdec_changed = cfg->fftdec != s_rx_cfg.fftdec;
    s_rx_cfg           = *cfg;
    s_rx_cfg_published = true;
    Bsp_Irq_RestoreLow(key);

    s_mode = cfg->rx_mode;
    if (mode_changed || fftdec_changed) {
        Sdr_AudioResample_Reset();
        apply_resample_config(cfg->rx_mode, cfg->fftdec);
    }
    App_Event_NotifyRxCfg(&s_rx_cfg);
}

void Sdr_Rx_SetConfig(const SdrRxCfg *cfg) {
    SdrRxCfg committed;
    uint32_t key;

    if (cfg == 0 || !rates_valid(cfg->rx_mode, cfg->fftdec)) {
        LOG_ERROR("rx config rejected: mode %u fftdec %u",
                  (unsigned)cfg->rx_mode, (unsigned)cfg->fftdec);
        return;
    }

    key = Bsp_Irq_SaveLow();
    committed = s_rx_cfg;
    Bsp_Irq_RestoreLow(key);

    if (cfg->fftdec != committed.fftdec) {
        /* The FPGA raw generation will change; stage the snapshot so the
         * demodulator and the subscribers move together at the commit. */
        key = Bsp_Irq_SaveLow();
        s_pending_cfg          = *cfg;
        s_pending_config_valid = true;
        Bsp_Irq_RestoreLow(key);
        return;
    }

    /* Same generation: apply now. A pending snapshot staged for an fftdec that
     * has since been reverted is discarded. */
    s_pending_config_valid = false;

    if (s_rx_cfg_published &&
        cfg->rx_mode == committed.rx_mode &&
        cfg->filter_low_hz == committed.filter_low_hz &&
        cfg->filter_high_hz == committed.filter_high_hz) {
        return;
    }

    rx_cfg_commit(cfg);
}

/*
 * Read one I2S block of fresh raw I/Q into the float output. Returns true when
 * out->size is a full block; false when the FPGA is behind by more than the one
 * sample that the linear interpolation can restore (out->size stays 0, the
 * caller skips its processing steps). No logs, divides or blocking in here: it
 * runs in the I2S DMA ISR.
 */
static bool radio_read_iq(Sdr_IQ_Data *out) {
    uint32_t m     = Drv_Fpga_GetSamplesPerTick();
    uint32_t block = DRV_CODEC_BLOCK_FRAMES * m;
    uint32_t w     = Drv_Fpga_GetPos();
    uint32_t gen   = Drv_Fpga_GetRawGeneration();
    uint32_t delta;

    out->i    = s_iq_i;
    out->q    = s_iq_q;
    out->size = 0u;

    if (gen != s_raw_gen_seen) {
        /* The raw ring was invalidated: the driver has sent the new control
         * frame and committed the active fftdec/M. Commit the staged snapshot
         * now, in step with the generation change, so the DSP filters and the
         * SdrRxCfg subscribers are not moved ahead of the hardware. */
        if (s_pending_config_valid) {
            SdrRxCfg cfg;
            uint32_t key = Bsp_Irq_SaveLow();

            s_pending_config_valid = false;
            cfg                    = s_pending_cfg;
            Bsp_Irq_RestoreLow(key);

            rx_cfg_commit(&cfg);
        }

        s_raw_gen_seen = gen;
        s_iq_started   = true;
        s_iq_r         = w;
        s_skip_frames  = SDR_RX_FFTDEC_SKIP_FRAMES;
        s_fade_pending = false;
        /* The driver commits M together with the generation bump, but this
         * ISR may have read the previous value before that commit. */
        m     = Drv_Fpga_GetSamplesPerTick();
        block = DRV_CODEC_BLOCK_FRAMES * m;
    }

    if (!s_iq_started) {
        s_iq_r       = (w - SDR_RX_IQ_LAG_BLOCKS * block) & SDR_RX_IQ_RING_MASK;
        s_iq_started = true;
    }

    delta = (w - s_iq_r) & SDR_RX_IQ_RING_MASK;

    if (delta == 0u) {
        return false;
    }

    if (s_skip_frames > 0u) {
        /* Drop all raw I/Q accumulated since the last call and
         * count it in 48 kHz frames (one frame per FPGA tick). */
        uint32_t frames = delta / m;

        if (frames > 0u) {
            s_iq_r = (s_iq_r + delta) & SDR_RX_IQ_RING_MASK;
            s_skip_frames = (s_skip_frames > frames) ?
                            (s_skip_frames - frames) : 0u;
            if (s_skip_frames == 0u) {
                s_fade_pending = true;
            }
        }
        return false;
    }

    if (delta > DRV_FPGA_RAW_WINDOW_SAMPLES - block) {
        /* Writer has run far ahead of the reader; resync and drop stale
         * samples. Strict > keeps the nominal one-block lag valid at
         * fftdec = 0, where block is exactly half the ring. */
        s_iq_r = (w - SDR_RX_IQ_LAG_BLOCKS * block) & SDR_RX_IQ_RING_MASK;
        return false;
    }

    if (delta >= block) {
        radio_convert(s_iq_r, block);
        if (s_fade_pending) {
            radio_apply_fade(block, block);
            s_fade_pending = false;
        }
        s_iq_r    = (s_iq_r + block) & SDR_RX_IQ_RING_MASK;
        out->size = block;
        return true;
    }

    if (delta == block - 1u) {
        /* One sample short: restore it by linear interpolation between its
         * neighbours and advance the read index by one sample less, so the lag
         * recovers by one sample per call. */
        radio_convert(s_iq_r, block - 1u);
        s_iq_i[block - 1u] = s_iq_i[block - 2u];
        s_iq_q[block - 1u] = s_iq_q[block - 2u];
        s_iq_i[block - 2u] = 0.5f * (s_iq_i[block - 3u] + s_iq_i[block - 1u]);
        s_iq_q[block - 2u] = 0.5f * (s_iq_q[block - 3u] + s_iq_q[block - 1u]);
        if (s_fade_pending) {
            radio_apply_fade(block, block);
            s_fade_pending = false;
        }
        s_iq_r    = (s_iq_r + block - 1u) & SDR_RX_IQ_RING_MASK;
        out->size = block;
        return true;
    }

    return false;
}

void Sdr_Rx_Init(void) {
    iq_data        = Drv_Fpga_GetRawWindow();
    s_iq_started   = false;
    s_raw_gen_seen = Drv_Fpga_GetRawGeneration();
    s_skip_frames  = 0u;
    s_fade_pending = false;

    s_pending_config_valid = false;
    s_audio_plan           = NULL;
    s_audio_plan_valid     = false;
    s_audio_src_hz         = 0u;

    s_mode = SDR_MOD_LSB;
    /* The filter edges are carried in the block for the subscribers but are not
     * interpreted here; the first Sdr_Rx_SetConfig() supplies them. The fftdec
     * uses the requested value: App_Fpga_Init() may have queued a rate change
     * that the next PE10 tick has not committed yet, so configuring for the
     * requested rate keeps DSP and hardware in step once the tick fires. */
    s_rx_cfg = (SdrRxCfg){
        SDR_MOD_LSB, 150, 2900, Drv_Fpga_GetRequestedFftdec(),
    };
    s_rx_cfg_published = false;

    apply_resample_config(SDR_MOD_LSB, s_rx_cfg.fftdec);
}

bool Sdr_Rx_Process(float *audio48, uint32_t n) {
    bool produced = false;

    if (audio48 == NULL) {
        return false;
    }

    for (uint32_t i = 0; i < n; ++i) {
        audio48[i] = 0.0f;
    }

    if (n != DRV_CODEC_BLOCK_FRAMES) {
        return false;
    }

    Sdr_IQ_Data iq;

    if (!radio_read_iq(&iq)) {
        return false;
    }
    Sdr_NB_Process(&iq);

    Sdr_IQ_Data decim_data = Sdr_Decimate_Process(iq);
    const SdrRxAudioPlan *plan = s_audio_plan_valid ? s_audio_plan : NULL;
    bool demod_done = false;

    switch (s_mode) {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
        Sdr_LSB_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        Sdr_USB_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        Sdr_CW_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    case SDR_MOD_AM:
        Sdr_AM_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    case SDR_MOD_SAM:
        Sdr_SAM_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    case SDR_MOD_NFM:
        Sdr_NBFM_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    case SDR_MOD_WFM:
        Sdr_WFM_Demod(&decim_data, s_demod_raw);
        demod_done = true;
        break;
    default:
        break;
    }

    const float *audio_out = NULL;
    uint32_t audio_size = 0u;

    if (demod_done && plan != NULL) {
        if (plan->common_chain) {
            float *audio_demod = NULL;
            uint32_t audio_demod_size = 0u;

            if (plan->pre == SDR_RX_AUDIO_PRE_NONE) {
                audio_demod = s_demod_raw;
                audio_demod_size = decim_data.size;
            } else if (plan->pre == SDR_RX_AUDIO_PRE_DECIM_48_24) {
                audio_demod_size = Sdr_Audio_Decimate_48_24(
                    s_demod_raw, decim_data.size, s_audio24);
                audio_demod = s_audio24;
            } else if (plan->pre == SDR_RX_AUDIO_PRE_INTERP_12_24) {
                audio_demod_size = Sdr_Audio_Interpolate_12_To_24(
                    s_demod_raw, decim_data.size, s_audio24);
                audio_demod = s_audio24;
            }

            if (audio_demod != NULL && audio_demod_size == SDR_AUDIO_BLOCK_24K) {
                rx_audio_process(audio_demod, audio_demod_size, s_mode);
                if (plan->post == SDR_RX_AUDIO_POST_INTERP_24_48) {
                    audio_size = Sdr_Audio_Interpolate_24_To_48(
                        audio_demod, audio_demod_size, s_audio48);
                    audio_out = s_audio48;
                } else {
                    audio_out = audio_demod;
                    audio_size = audio_demod_size;
                }
            }
        } else {
            /* WFM: the demodulator already resampled to 48 kHz. */
            audio_out  = s_demod_raw;
            audio_size = n;
        }
    }

    if (audio_out != NULL && audio_size == n) {
        arm_copy_f32(audio_out, audio48, n);
        Sdr_Squelch_Process(audio48, n);
        produced = true;
    }

    return produced;
}

float Sdr_Rx_Get_Level_Db(void) {
    switch (s_mode) {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        return Sdr_SSB_Get_Rx_Level_Db();
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        return Sdr_CW_Get_Rx_Level_Db();
    default:
        /* AM/SAM/NFM/WFM: the spectrum in-band power is representative and has
         * enough bins. */
        return Sdr_Spectrum_Get_S_meter();
    }
}
