#pragma once

#include <stdint.h>

#include "drv_codec.h"
#include "drv_fpga.h"
#include "sdr_common.h"

void Sdr_Resample_Init(void);

/*
 * Select the decimation chain. src and dst are in Hz and must come from the
 * family derived from BOARD_AUDIO_SAMPLE_RATE_HZ
 * ({384000, 192000, 96000, 48000, 24000, 12000}) with src >= dst, so the
 * chain is a contiguous run of x2 stages. Unsupported pairs are only logged
 * and leave the chain bypassed (count 0), so decimate returns its input.
 *
 * One I2S block spans DRV_CODEC_BLOCK_FRAMES board ticks: 512, 256, 128, 64,
 * 32 or 16 samples per channel at 384/192/96/48/24/12 kHz respectively.
 *
 * For src = 384 kHz the first 384 -> 192 kHz stage has two variants: a sharp
 * elliptic IIR when dst = 192 kHz, and the light sdr_decim_384_9 when the
 * signal is decimated further to 96/48/24/12 kHz.
 */
void Sdr_Resample_SetRates(uint32_t src_rate_hz, uint32_t dst_rate_hz);

/*
 * Decimate one I2S block along the chain selected by Sdr_Resample_SetRates.
 * data.size must equal the source-rate block size; a mismatch is counted and
 * returns size 0 without processing. The result is written to a shared static
 * buffer and remains valid until the next decimate call. RX path only.
 */
Sdr_IQ_Data Sdr_Decimate_Process(Sdr_IQ_Data data);

/*
 * Interpolate one IQ block from in_freq up to BOARD_AUDIO_SAMPLE_RATE_HZ
 * (48 kHz) for the TX path, writing the result into dst->i/q. Only 12 kHz (two
 * x2 stages, 12 -> 24 -> 48, input block DRV_CODEC_BLOCK_FRAMES/4) and 24 kHz
 * (one x2 stage, input block DRV_CODEC_BLOCK_FRAMES/2) are supported; any other
 * frequency or input size is counted as an error and leaves dst->size at 0.
 * On success dst->size is DRV_CODEC_BLOCK_FRAMES. The interpolator state is
 * reset whenever in_freq changes.
 */
void Sdr_Interpolate_Process(Sdr_IQ_Data src, Sdr_IQ_Data *dst, uint32_t in_freq);
