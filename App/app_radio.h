#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "app_ptt.h"
#include "sdr_common.h"

typedef enum {
    APP_RADIO_MODE_RX = 0,
    APP_RADIO_MODE_TX,
    APP_RADIO_MODE_OFF
} AppRadioMode;

int          App_Radio_Init(void);
int          App_Radio_SetRadioMode(AppRadioMode mode);
AppRadioMode App_Radio_GetRadioMode(void);

/* Frames (48 kHz) the CW keyer must still hold the start of the first element
 * after requesting TX: the relay/analog path settle window when the LPF band
 * actually changes, 0 otherwise (or once the window has elapsed). Safe to call
 * from the codec ISR. */
uint32_t App_Radio_TxSettleRemainingFrames(void);

/* Apply the speaker enable policy: headphones disconnected -> speaker on,
 * headphones connected -> speaker follows the host spmode setting. Debounces
 * HEADPHONE DETECT and only writes SPEAKER ENABLE when the result changes. */
void App_Radio_UpdateSpeaker(void);

/* Apply the microphone selection/gains and the RX audio route (normal RX or
 * voice_rec) received from hostlink to the codec. Called from
 * App_RadioState_Apply() after the staged edit commits. */
void App_Radio_ApplyAudioConfig(void);

/* Re-apply the microphone/line-in routing for the PTT source that is about to
 * key TX. Called at the start of the TX transition. */
void App_Radio_ApplyAudioForTx(void);

/*
 * TX signal publication for the future modulator. Converts count float I/Q
 * samples to Q1.31, appends the whole block to the TX spectrum and copies it
 * into the FPGA TX staging ring. Safe from the I2S DMA ISR. The driver may
 * accept fewer samples than offered when its ring is full; those are counted
 * and reported once per second by App_Radio_LogTxDrops().
 */
void App_Radio_TxPublish(const float *i, const float *q, uint32_t count);

/* Read and clear the TX staging overflow counter; logs a warning when it was
 * non-zero. Call once per second from housekeeping (main context). */
void App_Radio_LogTxDrops(void);

/*
 * Winner of the PTT arbitration, pushed by app_ptt whenever it changes (the
 * first source to request TX, held until every source is released). The radio
 * derives the TX audio input from it and inhibits the TX signal path for a
 * calibration request so the PA is keyed without a signal.
 */
void App_Radio_SetTxSource(AppPttSource source);

/*
 * Tune carrier: when the TX source is APP_PTT_SOURCE_ATU_TUNE or
 * APP_PTT_SOURCE_SWR_SCAN the TX branch generates a steady carrier
 * (I = envelope, Q = 0) with a ramp instead of the modulator. on = false runs
 * the envelope down (the blank used while ATU relays switch, so no RF is emitted
 * during a relay pulse). Safe from any context.
 */
void App_Radio_SetTuneCarrier(bool on);

/*
 * AIC frames accumulated since the last call (read-and-clear, like
 * Drv_Fpga_SampleCount). app_housekeeping compares them against the FPGA
 * tick-equivalent (samples / M) and warns when the two clock domains drift
 * apart.
 */
uint32_t App_Radio_GetAicSamples(void);
