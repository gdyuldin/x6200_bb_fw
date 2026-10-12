#pragma once

#include "sdr_common.h"

void Sdr_AM_Init(void);
void Sdr_AM_Demod(const Sdr_IQ_Data *data, float *audio);
void Sdr_SAM_Demod(const Sdr_IQ_Data *data, float *audio);

/* Linear carrier estimate (the DC the demod removes) for AM and SAM. Read by the
 * AGC in the same I2S DMA ISR that runs the demodulator. */
float Sdr_AM_GetCarrierLevel(void);

/* Squelch metric in dB for AM and SAM, updated once per demod block. More is
 * more signal-like; may be negative. 0 means no data (noise estimate invalid or
 * carrier not yet valid). */
float Sdr_AM_GetSignalMetric(void);

void Sdr_AM_Mod(const float *audio, Sdr_IQ_Data *data);
