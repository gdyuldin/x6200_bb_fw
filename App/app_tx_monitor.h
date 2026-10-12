#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * TX power/VSWR monitor. The ADC monitor DMA ISR feeds one raw scan per call;
 * this module keeps the power and VSWR branches independent and exposes a
 * lock-protected snapshot for telemetry. A VSWR event is one-shot: the DMA ISR
 * only sets a pending flag, App_Protect::evaluate() consumes it in thread
 * context and forces RX when the radio is transmitting.
 */

typedef struct {
    float fwd_power_peak; /* peak-hold forward power, fast decay, W */
    float fwd_power_pep;  /* peak-hold forward power, speech-rate decay, W */
    float fwd_power_inst; /* instantaneous forward power, W */
    float fwd_power_avg;  /* smoothed forward power, W */
    float ref_power;      /* instantaneous reflected power, W */
    float ref_power_fast; /* short-filter reflected power, W */
    float ref_power_avg;  /* smoothed reflected power, W */
    float swr;            /* latest SWR; 99.0f when invalid or no power */
    bool  swr_valid;      /* latest 10-scan block produced a finite ratio */
    uint16_t raw_fwd;     /* latest raw forward detector code */
    uint16_t raw_ref;     /* latest raw reflected detector code */
} AppTxMonitorSnapshot;

/* Zero state, precompute EMA coefficients, register the ADC scan hook. */
void App_TxMonitor_Init(void);

/* Copy the latest computed values under an IRQ lock. */
void App_TxMonitor_GetSnapshot(AppTxMonitorSnapshot *out);

/* Consume the one-shot VSWR trip event. */
bool App_TxMonitor_TakeSwrTrip(void);

/*
 * ATU fast window. The main snapshot path is smoothed over ~50 ms and is
 * useless for the 1-2 ms ATU measurement window. AtuArm() starts a fresh
 * accumulator that collects window_ms of raw detector scans; when the window
 * fills, the reflection coefficient gamma = V_ref / V_fwd is computed and
 * latched. Detector codes are converted to volts with the same calibration as
 * the main path, but below the calibrated floor the conversion extrapolates
 * linearly to (0, 0) instead of clamping, so the near-match trend is kept.
 * AtuTake() returns it once (true) and clears the flag, so a caller polls it
 * until it is ready.
 */
void App_TxMonitor_AtuArm(uint32_t window_ms);

/* Without a carrier the window latches this value, which compares worse than
 * any physical gamma. */
#define APP_TX_MON_ATU_GAMMA_INVALID 2.0f

/* Minimum mean raw forward code for a window to be considered valid. */
#define APP_TX_MON_ATU_MIN_FWD_RAW 64u

/* Return the freshly measured reflection coefficient (true) and clear the
 * latch, or false when no completed window is waiting. */
bool App_TxMonitor_AtuTake(float *gamma);
