#pragma once

#include <stdbool.h>

#include "sdr_common.h"

void Sdr_SSB_Init(void);

void Sdr_LSB_Demod(const Sdr_IQ_Data *data, float *audio);
void Sdr_USB_Demod(const Sdr_IQ_Data *data, float *audio);

void Sdr_LSB_Mod(const float *audio, Sdr_IQ_Data *data);
void Sdr_USB_Mod(const float *audio, Sdr_IQ_Data *data);

/* Squelch metric in dB, updated once per demod block. More is more
 * signal-like; may be negative. 0 means no data (noise estimate invalid). */
float Sdr_SSB_GetSignalMetric(void);

/* RX S-meter level in dB (more negative = weaker). In-band power after the
 * selective IIR pair, peak-held between reads. Holds the last value while no
 * block runs. Calibrated by SDR_SSB_LEVEL_DB_OFFSET. */
float Sdr_SSB_Get_Rx_Level_Db(void);
