#include "sdr_resample.h"

#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"

#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_codec.h"
#include "drv_fpga.h"
#include "sdr_halfband_decim.h"
#include "sdr_iir_decim.h"
#include "sdr_resampling_filters.h"

#define LOG_TAG "app.sdr.resample"

#define SDR_DECIM_STAGE_COUNT   5u
#define SDR_DECIM_RATE_COUNT    6u

/*
 * One I2S block is DRV_CODEC_BLOCK_FRAMES frames of the board tick. At the
 * maximum source rate (8x the tick) that is 8 samples per tick per channel;
 * each x2 stage halves it. Five stages cover 384 -> 192 -> 96 -> 48 -> 24 ->
 * 12 kHz.
 */
#define SDR_MAX_RATE_HZ          (BOARD_AUDIO_SAMPLE_RATE_HZ * 8u)
#define SDR_DECIM_BLOCK_MAX      (DRV_CODEC_BLOCK_FRAMES * 8u)
#define SDR_DECIM_STAGE_BLOCK(i) (SDR_DECIM_BLOCK_MAX >> (i))
#define SDR_DECIM_STAGE_OUT(i)   (SDR_DECIM_STAGE_BLOCK(i) >> 1u)

#define SDR_INTERP_L            2u

/* Input blocks for the two x2 interpolation stages: 24 kHz for the final
 * 24 -> 48 kHz step and 12 kHz for the 12 -> 24 kHz step. */
#define SDR_INTERP_24_BLOCK_IN     (DRV_CODEC_BLOCK_FRAMES / SDR_INTERP_L)
#define SDR_INTERP_12_BLOCK_IN     (SDR_INTERP_24_BLOCK_IN / SDR_INTERP_L)

typedef struct {
    SdrHalfbandDecim              *hb;       /* set for half-band FIR stages */
    SdrIirDecim                   *iir;      /* set for the IIR variant */
    const float32_t               *coeffs;
    float                         *pstate[2];
    float                         *scratch[2]; /* IIR only */
    uint16_t                       num_taps;
    uint16_t                       block_size;
    uint8_t                        num_sections; /* IIR only */
} SdrDecimStage;

static const uint32_t s_rates_hz[SDR_DECIM_RATE_COUNT] = {
    SDR_MAX_RATE_HZ,
    SDR_MAX_RATE_HZ >> 1u,
    SDR_MAX_RATE_HZ >> 2u,
    SDR_MAX_RATE_HZ >> 3u,
    SDR_MAX_RATE_HZ >> 4u,
    SDR_MAX_RATE_HZ >> 5u,
};

/*
 * Decimation output buffer, sized for the largest stage output (the
 * 384 -> 192 kHz stage). Each decimation stage writes here in turn.
 */
static float s_out[2][SDR_DECIM_BLOCK_MAX >> 1u] __attribute__((aligned(32)));

static SdrIirDecim s_iirdec_384_90[2];
static float s_iir_state_384_90[2][4u * SDR_DECIM_384_90_IIR_SECTIONS]
    __attribute__((aligned(32)));
static float s_iir_scratch_384_90[2][SDR_DECIM_STAGE_BLOCK(0u)]
    __attribute__((aligned(32)));

/*
 * The 384 -> 192 kHz stage has two variants: the sharp elliptic IIR for a
 * direct 192 kHz output and the light sdr_decim_384_9 when the result is
 * decimated further to 96/48/24/12 kHz. Unlike the FIR variant the IIR runs
 * at the full input rate and decimates its output.
 */
static SdrHalfbandDecim s_hb_384_9[2];
static float s_dec_pstate_384_9[2]
    [SDR_DECIM_384_9_NUM_TAPS + SDR_DECIM_STAGE_BLOCK(0u) - 1u]
    __attribute__((aligned(32)));

static SdrHalfbandDecim s_hb_192_9[2];
static float s_dec_pstate_192_9[2][SDR_DECIM_192_9_NUM_TAPS + SDR_DECIM_STAGE_BLOCK(1u) - 1u]
    __attribute__((aligned(32)));

static SdrHalfbandDecim s_hb_96_9[2];
static float s_dec_pstate_96_9[2][SDR_DECIM_96_9_NUM_TAPS + SDR_DECIM_STAGE_BLOCK(2u) - 1u]
    __attribute__((aligned(32)));

static SdrHalfbandDecim s_hb_48_9[2];
static float s_dec_pstate_48_9[2][SDR_DECIM_48_9_NUM_TAPS + SDR_DECIM_STAGE_BLOCK(3u) - 1u]
    __attribute__((aligned(32)));

static SdrHalfbandDecim s_hb_24_5[2];
static float s_dec_pstate_24_5[2][SDR_DECIM_24_5_NUM_TAPS + SDR_DECIM_STAGE_BLOCK(4u) - 1u]
    __attribute__((aligned(32)));

/*
 * TX IQ interpolation. The 12 kHz path needs two x2 stages (12 -> 24 -> 48),
 * so s_int_temp holds the intermediate 24 kHz block. s_interp_in_freq tracks
 * the last input rate; changing it resets both interpolator states.
 */
static float s_int_temp[2][DRV_CODEC_BLOCK_FRAMES >> 1u] __attribute__((aligned(32)));
static uint32_t s_interp_in_freq = 0;

static arm_fir_interpolate_instance_f32 s_firint_12_5[2];
static float s_int_pstate_12_5[2][SDR_INTERP_12_BLOCK_IN +
                                (SDR_INTERP_24_5_NUM_TAPS / SDR_INTERP_L) - 1u]
    __attribute__((aligned(32)));

static arm_fir_interpolate_instance_f32 s_firint_24_8[2];
static float s_int_pstate_24_8[2][SDR_INTERP_24_BLOCK_IN +
                                (SDR_INTERP_48_9_NUM_TAPS / SDR_INTERP_L) - 1u]
    __attribute__((aligned(32)));

static const SdrDecimStage s_stages[SDR_DECIM_STAGE_COUNT] = {
    {
        s_hb_384_9,
        0,
        sdr_decim_384_9_coeffs,
        { s_dec_pstate_384_9[0], s_dec_pstate_384_9[1] },
        { 0, 0 },
        SDR_DECIM_384_9_NUM_TAPS,
        SDR_DECIM_STAGE_BLOCK(0u),
        0u,
    },
    {
        s_hb_192_9,
        0,
        sdr_decim_192_9_coeffs,
        { s_dec_pstate_192_9[0], s_dec_pstate_192_9[1] },
        { 0, 0 },
        SDR_DECIM_192_9_NUM_TAPS,
        SDR_DECIM_STAGE_BLOCK(1u),
        0u,
    },
    {
        s_hb_96_9,
        0,
        sdr_decim_96_9_coeffs,
        { s_dec_pstate_96_9[0], s_dec_pstate_96_9[1] },
        { 0, 0 },
        SDR_DECIM_96_9_NUM_TAPS,
        SDR_DECIM_STAGE_BLOCK(2u),
        0u,
    },
    {
        s_hb_48_9,
        0,
        sdr_decim_48_9_coeffs,
        { s_dec_pstate_48_9[0], s_dec_pstate_48_9[1] },
        { 0, 0 },
        SDR_DECIM_48_9_NUM_TAPS,
        SDR_DECIM_STAGE_BLOCK(3u),
        0u,
    },
    {
        s_hb_24_5,
        0,
        sdr_decim_24_5_coeffs,
        { s_dec_pstate_24_5[0], s_dec_pstate_24_5[1] },
        { 0, 0 },
        SDR_DECIM_24_5_NUM_TAPS,
        SDR_DECIM_STAGE_BLOCK(4u),
        0u,
    },
};

static const SdrDecimStage s_stage_384_90 = {
    0,
    s_iirdec_384_90,
    sdr_decim_384_90_iir_sos,
    { s_iir_state_384_90[0], s_iir_state_384_90[1] },
    { s_iir_scratch_384_90[0], s_iir_scratch_384_90[1] },
    0u,
    SDR_DECIM_STAGE_BLOCK(0u),
    SDR_DECIM_384_90_IIR_SECTIONS,
};

/* Selected variant of the first (384 -> 192 kHz) stage. */
static const SdrDecimStage *s_first_stage = &s_stages[0];

static const SdrDecimStage *stage_at(uint32_t index)
{
    return (index == 0u) ? s_first_stage : &s_stages[index];
}

static uint32_t s_start = 0;
static uint32_t s_count = 0;
static uint32_t s_src_hz = 0;
static uint32_t s_dst_hz = 0;
static volatile uint32_t s_size_errors;
static volatile uint32_t s_interp_size_errors;

_Static_assert(SDR_DECIM_RATE_COUNT == SDR_DECIM_STAGE_COUNT + 1u,
               "rate and stage tables disagree");

static bool rate_index(uint32_t rate_hz, uint32_t *index)
{
    for (uint32_t i = 0; i < SDR_DECIM_RATE_COUNT; ++i) {
        if (s_rates_hz[i] == rate_hz) {
            *index = i;
            return true;
        }
    }
    return false;
}

static void reset_stage(const SdrDecimStage *stage)
{
    for (uint32_t ch = 0; ch < 2u; ++ch) {
        if (stage->iir != 0) {
            Sdr_IirDecim_Reset(&stage->iir[ch]);
        } else {
            Sdr_HalfbandDecim_Reset(&stage->hb[ch]);
        }
    }
}

static void init_stage(const SdrDecimStage *stage)
{
    for (uint32_t ch = 0; ch < 2u; ++ch) {
        if (stage->iir != 0) {
            Sdr_IirDecim_Init(&stage->iir[ch], stage->coeffs,
                              stage->num_sections, stage->pstate[ch],
                              stage->scratch[ch]);
        } else {
            Sdr_HalfbandDecim_Init(&stage->hb[ch], stage->coeffs,
                                   stage->num_taps, stage->pstate[ch],
                                   stage->block_size);
        }
    }
}

void Sdr_Resample_Init(void)
{
    for (uint32_t i = 0; i < SDR_DECIM_STAGE_COUNT; ++i) {
        init_stage(&s_stages[i]);
    }
    init_stage(&s_stage_384_90);

    for (uint32_t ch = 0; ch < 2u; ++ch) {
        arm_fir_interpolate_init_f32(&s_firint_24_8[ch], SDR_INTERP_L,
                                     SDR_INTERP_48_9_NUM_TAPS,
                                     sdr_interp_48_9_coeffs,
                                     s_int_pstate_24_8[ch],
                                     SDR_INTERP_24_BLOCK_IN);
        arm_fir_interpolate_init_f32(&s_firint_12_5[ch], SDR_INTERP_L,
                                     SDR_INTERP_24_5_NUM_TAPS,
                                     sdr_interp_24_5_coeffs,
                                     s_int_pstate_12_5[ch],
                                     SDR_INTERP_12_BLOCK_IN);
    }

    s_first_stage = &s_stages[0];
    s_start       = 0u;
    s_count       = 0u;
}

void Sdr_Resample_SetRates(uint32_t src_rate_hz, uint32_t dst_rate_hz)
{
    uint32_t src_idx;
    uint32_t dst_idx;
    uint32_t start = 0u;
    uint32_t count = 0u;
    uint32_t key;
    bool     changed;

    if (!rate_index(src_rate_hz, &src_idx) ||
        !rate_index(dst_rate_hz, &dst_idx) ||
        src_rate_hz < dst_rate_hz) {
        LOG_ERROR("unsupported rates %u -> %u hz", src_rate_hz, dst_rate_hz);
    } else {
        start = src_idx;
        count = dst_idx - src_idx;
    }

    key     = Bsp_Irq_SaveLow();
    changed = (s_src_hz != src_rate_hz) ||
              (s_dst_hz != dst_rate_hz);

    /*
     * A direct 384 -> 192 kHz output uses the sharp elliptic IIR first stage;
     * any further decimation to 96/48/24/12 kHz uses the light intermediate
     * stage.
     */
    s_first_stage = ((src_rate_hz == SDR_MAX_RATE_HZ) &&
                     (dst_rate_hz == SDR_MAX_RATE_HZ / 2u))
                    ? &s_stage_384_90
                    : &s_stages[0];

    s_start       = start;
    s_count       = count;
    s_src_hz      = src_rate_hz;
    s_dst_hz      = dst_rate_hz;
    Bsp_Irq_RestoreLow(key);

    /*
     * The filter-state fill stays in the I2S DMA ISR but runs outside the
     * critical section: zeroing up to ~9 KB here must not hold off the FPGA
     * PE10 tick. The FPGA ISR never touches the decimation state. When the
     * call comes from radio_read_iq() the audio DMA ISR cannot preempt
     * itself, so the fill is uninterrupted; a main-context caller can be
     * preempted by audio DMA for its duration, which only risks one block of
     * transient output. Runtime changes use the deferred request path, so the
     * normal call arrives from the DMA ISR.
     */
    if (changed) {
        for (uint32_t i = 0; i < count; ++i) {
            reset_stage(stage_at(start + i));
        }
    }
}

Sdr_IQ_Data Sdr_Decimate_Process(Sdr_IQ_Data data)
{
    uint32_t    count = s_count;
    uint32_t    start = s_start;
    uint32_t    size;
    float      *in_i = data.i;
    float      *in_q = data.q;
    Sdr_IQ_Data out;

    if (count == 0u) {
        return data;
    }

    size = SDR_DECIM_STAGE_BLOCK(start);
    if (data.size != size) {
        s_size_errors++;
        out.i    = data.i;
        out.q    = data.q;
        out.size = 0u;
        return out;
    }

    for (uint32_t i = 0; i < count; ++i) {
        const SdrDecimStage *stage = stage_at(start + i);

        if (stage->iir != 0) {
            Sdr_IirDecim_Process(&stage->iir[0], in_i, s_out[0], size);
            Sdr_IirDecim_Process(&stage->iir[1], in_q, s_out[1], size);
        } else {
            Sdr_HalfbandDecim_Process(&stage->hb[0], in_i, s_out[0], size);
            Sdr_HalfbandDecim_Process(&stage->hb[1], in_q, s_out[1], size);
        }

        in_i = s_out[0];
        in_q = s_out[1];
        size >>= 1u;
    }

    out.i    = in_i;
    out.q    = in_q;
    out.size = size;
    return out;
}

/* Clear both interpolator histories. Called when in_freq changes so samples
 * from the previous rate do not leak into the new stream. */
static void reset_interp_states(void) {
    arm_fill_f32(0.0f, (float*)s_int_pstate_12_5, sizeof(s_int_pstate_12_5) / sizeof(float));
    arm_fill_f32(0.0f, (float*)s_int_pstate_24_8, sizeof(s_int_pstate_24_8) / sizeof(float));
}

static void sdr_interpolate_iq(const arm_fir_interpolate_instance_f32 *s, const float *i_src, const float *q_src,
                              float *i_dst, float *q_dst, uint32_t size)
{
    arm_fir_interpolate_f32(&s[0], i_src, i_dst, size);
    arm_fir_interpolate_f32(&s[1], q_src, q_dst, size);
}


/*
 * Interpolate one TX IQ block to 48 kHz into dst. 12 kHz input goes through
 * both x2 stages (12 -> 24 -> 48), with s_int_temp as the intermediate buffer;
 * 24 kHz input needs only the final 24 -> 48 stage. Unsupported in_freq or an
 * unexpected src.size leaves dst->size at 0 and counts an error.
 */
void Sdr_Interpolate_Process(Sdr_IQ_Data src, Sdr_IQ_Data *dst, uint32_t in_freq)
{
    dst->size = 0;

    if (in_freq != s_interp_in_freq) {
        s_interp_in_freq = in_freq;
        reset_interp_states();
    }

    if (in_freq == BOARD_AUDIO_SAMPLE_RATE_HZ / 4) {
        if (src.size == DRV_CODEC_BLOCK_FRAMES / 4) {
            /* 12 -> 24 kHz, then 24 -> 48 kHz. */
            sdr_interpolate_iq(s_firint_12_5, src.i, src.q, s_int_temp[0], s_int_temp[1], src.size);
            sdr_interpolate_iq(s_firint_24_8, s_int_temp[0], s_int_temp[1], dst->i, dst->q, src.size * 2);
            dst->size = DRV_CODEC_BLOCK_FRAMES;
        }
    } else if (in_freq == BOARD_AUDIO_SAMPLE_RATE_HZ / 2) {
        if (src.size == DRV_CODEC_BLOCK_FRAMES / 2) {
            /* 24 -> 48 kHz in one stage. */
            sdr_interpolate_iq(s_firint_24_8, src.i, src.q, dst->i, dst->q, src.size);
            dst->size = DRV_CODEC_BLOCK_FRAMES;
        }
    }

    if (dst->size == 0) {
        s_interp_size_errors++;
    }
}
