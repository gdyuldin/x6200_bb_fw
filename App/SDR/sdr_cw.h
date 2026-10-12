#pragma once

#include <stdint.h>

#include "sdr_common.h"

typedef enum {
    SDR_CW_KEY_STRAIGHT = 0,
    SDR_CW_KEY_DUAL_LEFT,
    SDR_CW_KEY_DUAL_RIGHT
} SdrCwKeyMode;

typedef enum {
    SDR_CW_IAMBIC_A = 0,
    SDR_CW_IAMBIC_B
} SdrCwIambicMode;

/* Host keyer speed limits (cmd33 ks). */
#define SDR_CW_SPEED_WPM_MIN 5u
#define SDR_CW_SPEED_WPM_MAX 50u

void Sdr_CW_Init(void);

/* Keyer configuration from the host (cmd33). Speed is clamped to 5..50 WPM;
 * volume is the raw 0..63 host value (0 = silent, 30 = 0 dB, 1 dB per step). */
void Sdr_CW_SetKeySpeed(uint32_t wpm);
void Sdr_CW_SetKeyMode(SdrCwKeyMode mode);
void Sdr_CW_SetIambicMode(SdrCwIambicMode mode);
void Sdr_CW_SetSidetoneVolume(uint8_t cwvol);
void Sdr_CW_SetTraining(bool training);

/* Host CW timing (cmd34 qsktime_kr): qsk_ms is the TX hold-off after the last
 * element (0..1000 ms), dash_ratio_x10 is the dah/dot duration ratio in tenths
 * (25..45 = 2.5..4.5). Both are clamped to those bounds. */
void Sdr_CW_SetQskTimeAndRatio(uint32_t qsk_ms, uint32_t dash_ratio_x10);

/* One keyer tick per codec block, called from the codec block callback before
 * the RX/TX branch: advances the 48 kHz envelope timeline. Returns a pointer to
 * the sidetone block at 48 kHz, or NULL when CW is not selected. */
float * Sdr_CW_KeyerUpdate(void);

/* Paddle debounce and host-key poll tick, driven by the TIM7 update interrupt
 * (3 kHz via Bsp_Time_StartTimer); Sdr_CW_Init wires it. If TIM7 is not
 * running, Sdr_CW_KeyerUpdate falls back to once per codec block. */
void Sdr_CW_KeyTick(void);

void Sdr_CW_Demod(const Sdr_IQ_Data *data, float *audio);
void Sdr_CW_Mod(const float *audio, Sdr_IQ_Data *data);

/* Squelch metric in dB, updated once per demod block. More is more
 * signal-like; may be negative. 0 means no data (noise estimate invalid). */
float Sdr_CW_GetSignalMetric(void);

/* RX S-meter level in dB (more negative = weaker). In-band power from the
 * channel-filtered I/Q, peak-held between reads. Holds the last value while no
 * block runs. Calibrated by SDR_CW_LEVEL_DB_OFFSET. */
float Sdr_CW_Get_Rx_Level_Db(void);
