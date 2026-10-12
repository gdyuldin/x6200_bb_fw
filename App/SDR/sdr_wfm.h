#pragma once

#include "sdr_common.h"

/* Wideband FM (broadcast, mono), demodulation only.  The discriminator runs
 * at the 192 kHz IQ rate; the module decimates internally and outputs mono
 * audio at BOARD_AUDIO_SAMPLE_RATE_HZ. */

void Sdr_WFM_Init(void);

/* data: IQ at 192 kHz (SDR_WFM_BLOCK_SIZE complex samples).
 * audio: mono output at 48 kHz (DRV_CODEC_BLOCK_FRAMES samples). */
void Sdr_WFM_Demod(const Sdr_IQ_Data *data, float *audio);
