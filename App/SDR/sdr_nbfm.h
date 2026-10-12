#pragma once

#include "sdr_common.h"

void Sdr_NBFM_Init(void);
void Sdr_NBFM_Demod(const Sdr_IQ_Data *data, float *audio);
void Sdr_NBFM_Mod(const float *audio, Sdr_IQ_Data *data);

/* Squelch metric in dB, updated once per demod block. More is more
 * signal-like; may be negative. 0 means no data (noise estimate invalid). */
float Sdr_NBFM_GetSignalMetric(void);
