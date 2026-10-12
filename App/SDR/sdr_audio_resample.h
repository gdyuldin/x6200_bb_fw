#pragma once

#include <stdint.h>

#include "drv_codec.h"

/* Audio-rate resampling for the RX/TX path */

#define SDR_AUDIO_BLOCK_24K (DRV_CODEC_BLOCK_FRAMES / 2u)
#define SDR_AUDIO_BLOCK_12K (DRV_CODEC_BLOCK_FRAMES / 4u)

void Sdr_AudioResample_Init(void);

void Sdr_AudioResample_Reset(void);

uint32_t Sdr_Audio_Decimate_48_24(const float *in, uint32_t in_size, float *out);

uint32_t Sdr_Audio_Interpolate_12_To_24(const float *in, uint32_t in_size, float *out);
uint32_t Sdr_Audio_Interpolate_24_To_48(const float *in, uint32_t in_size, float *out);
