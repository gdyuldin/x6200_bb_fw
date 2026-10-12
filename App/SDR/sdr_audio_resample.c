#include "sdr_audio_resample.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "arm_math.h"

#include "board_config.h"
#include "bsp_irq.h"
#include "drv_codec.h"
#include "sdr_halfband_decim.h"
#include "sdr_resampling_filters.h"

#define SDR_AUDIO_INTERP_FACTOR 2u

#define SDR_AUDIO_DECIM_48_STATE_LEN \
    (SDR_DECIM_48_9_NUM_TAPS + DRV_CODEC_BLOCK_FRAMES - 1u)

#define SDR_AUDIO_INTERP_24_STATE_LEN \
    (SDR_AUDIO_BLOCK_24K + (SDR_INTERP_48_9_NUM_TAPS / SDR_AUDIO_INTERP_FACTOR) - 1u)
#define SDR_AUDIO_INTERP_12_STATE_LEN \
    (SDR_AUDIO_BLOCK_12K + (SDR_INTERP_24_5_NUM_TAPS / SDR_AUDIO_INTERP_FACTOR) - 1u)


/* Decimation: 48 -> 24 kHz stage */
static SdrHalfbandDecim s_decim_48_9;
static float s_state_decim_48_9[SDR_AUDIO_DECIM_48_STATE_LEN]
    __attribute__((aligned(32)));


/* Interpolation: 24 -> 48 kHz stage, plus 12 -> 24 kHz stage feeding it */
static arm_fir_interpolate_instance_f32 s_fir_interp_24_8;
static float s_state_interp_24_8[SDR_AUDIO_INTERP_24_STATE_LEN]
    __attribute__((aligned(32)));
static arm_fir_interpolate_instance_f32 s_fir_interp_12_5;
static float s_state_interp_12_5[SDR_AUDIO_INTERP_12_STATE_LEN]
    __attribute__((aligned(32)));

static volatile uint32_t s_decim_size_errors;
static volatile uint32_t s_interp_size_errors;

static struct {
    bool reset_decim_48;
    bool reset_interp_24;
    bool reset_interp_12;
} s_pending;

void Sdr_AudioResample_Init(void)
{
    Sdr_HalfbandDecim_Init(&s_decim_48_9, sdr_decim_48_9_coeffs,
                           SDR_DECIM_48_9_NUM_TAPS, s_state_decim_48_9,
                           DRV_CODEC_BLOCK_FRAMES);

    (void)arm_fir_interpolate_init_f32(&s_fir_interp_24_8,
                                       SDR_AUDIO_INTERP_FACTOR,
                                       SDR_INTERP_48_9_NUM_TAPS,
                                       sdr_interp_48_9_coeffs,
                                       s_state_interp_24_8,
                                       SDR_AUDIO_BLOCK_24K);
    (void)arm_fir_interpolate_init_f32(&s_fir_interp_12_5,
                                       SDR_AUDIO_INTERP_FACTOR,
                                       SDR_INTERP_24_5_NUM_TAPS,
                                       sdr_interp_24_5_coeffs,
                                       s_state_interp_12_5,
                                       SDR_AUDIO_BLOCK_12K);

}

void Sdr_AudioResample_Reset(void)
{
    uint32_t mask = Bsp_Irq_SaveLow();
    s_pending.reset_decim_48 = true;
    s_pending.reset_interp_12 = true;
    s_pending.reset_interp_24 = true;
    Bsp_Irq_RestoreLow(mask);
}

uint32_t Sdr_Audio_Decimate_48_24(const float *in, uint32_t in_size, float *out)
{
    if (in == NULL || out == NULL || in_size != DRV_CODEC_BLOCK_FRAMES) {
        s_decim_size_errors++;
        return 0u;
    }
    if (s_pending.reset_decim_48) {
        s_pending.reset_decim_48 = false;
        Sdr_HalfbandDecim_Reset(&s_decim_48_9);
    }

    Sdr_HalfbandDecim_Process(&s_decim_48_9, in, out, DRV_CODEC_BLOCK_FRAMES);
    return SDR_AUDIO_BLOCK_24K;
}

uint32_t Sdr_Audio_Interpolate_12_To_24(const float *in, uint32_t in_size, float *out)
{
    if (in == NULL || out == NULL || in_size != SDR_AUDIO_BLOCK_12K) {
        s_interp_size_errors++;
        return 0u;
    }
    if (s_pending.reset_interp_12) {
        s_pending.reset_interp_12 = false;
        memset(s_state_interp_12_5, 0, sizeof(s_state_interp_12_5));
    }

    arm_fir_interpolate_f32(&s_fir_interp_12_5, in, out, SDR_AUDIO_BLOCK_12K);
    return SDR_AUDIO_BLOCK_24K;
}

uint32_t Sdr_Audio_Interpolate_24_To_48(const float *in, uint32_t in_size, float *out)
{
    if (in == NULL || out == NULL || in_size != SDR_AUDIO_BLOCK_24K) {
        s_interp_size_errors++;
        return 0u;
    }
    if (s_pending.reset_interp_24) {
        s_pending.reset_interp_24 = false;
        memset(s_state_interp_24_8, 0, sizeof(s_state_interp_24_8));
    }

    arm_fir_interpolate_f32(&s_fir_interp_24_8, in, out, SDR_AUDIO_BLOCK_24K);
    return DRV_CODEC_BLOCK_FRAMES;
}
