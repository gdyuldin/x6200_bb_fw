#pragma once

#include <stdbool.h>

#include "sdr_common.h"

/*
 * One-sided complex channel FIR shared by the SSB and CW demodulators.
 *
 * `lower` selects the lower sideband (LSB/CWR), otherwise the upper sideband
 * (USB/CW). Demod returns the real part of the complex-filtered I/Q, i.e. the
 * demodulated audio before the mode's audio band-pass. Mod is its TX inverse:
 * it builds an analytic complex signal from real audio.
 *
 * The module owns the filter history, so callers must Reset() it on a mode or
 * rate change. RX and TX never run together.
 */

void Sdr_Sideband_Init(void);
void Sdr_Sideband_Reset(void);

void Sdr_Sideband_Demod(const Sdr_IQ_Data *in, float *audio, bool lower);
void Sdr_Sideband_Mod(const float *audio, Sdr_IQ_Data *out, bool lower);
