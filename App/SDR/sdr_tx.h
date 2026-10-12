#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdr_common.h"

/* TX modulation orchestration.  The modulators themselves do not change the
 * sample rate: Sdr_Tx_Process resamples the input audio to the rate the plan
 * asks for, runs the modulator, and resamples the IQ result back to 48 kHz.
 *
 * Filtering: one band-pass IIR on the 48 kHz input audio, shared by all audio
 * modes.  Coefficients are designed when the frequency changes (from the
 * setter, ISR/task context, see Sdr_Tx_SetFilterFreq).
 */

#define SDR_TX_RATE_12K  12000u
#define SDR_TX_RATE_24K  24000u
#define SDR_TX_RATE_48K  48000u

void Sdr_Tx_Init(void);

/* Store the TX audio band-pass edges.  Values are identical for all audio
 * modes; CW ignores them.  Redesigning the coefficients happens in the setter. */
void Sdr_Tx_SetFilterFreq(uint32_t low_hz, uint32_t high_hz);

/* Process one 48 kHz audio block into a 48 kHz IQ block.
 *
 * audio48  : DRV_CODEC_BLOCK_FRAMES mono samples, 48 kHz
 * n        : number of samples in audio48
 * mode     : current TX modulation
 * out_i/q  : destination buffers, DRV_CODEC_BLOCK_FRAMES samples each
 *
 * Unsupported or audio-less modes write zeros.  Both output buffers are always
 * filled with n samples.
 */
void Sdr_Tx_Process(const float *audio48, uint32_t n, SdrModulation mode,
                    float *out_i, float *out_q, float *monitor);

/* True when the mode has a TX modulator and can be selected for transmit. */
bool Sdr_Tx_ModeSupported(SdrModulation mode);
