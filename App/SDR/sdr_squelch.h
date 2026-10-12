#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * RX squelch. Reads the per-mode signal metric published by the demodulators
 * (SSB/CW/AM/NBFM) in the same I2S DMA ISR pass, compares it against the
 * configured threshold with hysteresis and hold, and applies a gain ramp to the
 * 48 kHz audio block so mute/unmute does not click. It owns no demodulator
 * state and knows only the threshold.
 */

void Sdr_Squelch_Init(void);

/* Push the configuration from App_RadioState (main loop). threshold and
 * threshold_fm are the raw host values 0..100.
 * A threshold of 0 means "always open"; enable false bypasses the squelch
 * entirely (gain 1). Safe to call with interrupts running. */
void Sdr_Squelch_Configure(bool enable, uint8_t threshold, uint8_t threshold_fm);

/* Apply the squelch gain to one 48 kHz audio block. Called at the end of
 * Sdr_Rx_Process, in the I2S DMA ISR. */
void Sdr_Squelch_Process(float *audio48, uint32_t n);

/* Current decision for telemetry: true when the audio is being muted. */
bool Sdr_Squelch_IsMuted(void);
