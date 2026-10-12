#pragma once

#include <stdbool.h>

/*
 * TX/PTT arbiter. Combines every transmit source into one request and drives
 * the radio TX/RX transition from the main loop. The forced-RX latch keeps a
 * still-held source from re-entering TX after protection forced RX.
 */
typedef enum {
    APP_PTT_SOURCE_NONE = 0,
    APP_PTT_SOURCE_MTX,
    APP_PTT_SOURCE_IPTT,
    APP_PTT_SOURCE_CIVPTT,
    APP_PTT_SOURCE_HPTT,
    APP_PTT_SOURCE_CWKEY,
    APP_PTT_SOURCE_CALIB,
    APP_PTT_SOURCE_ATU_TUNE,
    APP_PTT_SOURCE_SWR_SCAN
} AppPttSource;

typedef struct {
    bool mtx;
    bool iptt;
    bool civptt;
    bool hptt;
    bool cwkey;
    bool calib;
    bool atu_tune;
    bool swr_scan;
    bool blocked; /* latch after a forced RX or a refused TX */
} AppPttStatus;

int  App_Ptt_Init(void);
void App_Ptt_SetMtx(bool on);
void App_Ptt_SetIptt(bool on);
void App_Ptt_SetCivPtt(bool on);

/* Calibration TX request, driven by the host `cal` flag and the console bias
 * mode. Keys TX with the signal path inhibited so the PA is driven without a
 * signal. */
void App_Ptt_SetCalibrate(bool on);

/* CW keyer TX request, driven from the keyer (ISR context, per debounce commit
 * and once per codec block). Includes the qsk tail; see sdr_cw.c. */
void App_Ptt_SetCwKey(bool on);

/* ATU tune-carrier TX request, driven by the ATU state machine (main loop). */
void App_Ptt_SetAtuTune(bool on);

/* SWR-scan TX request, driven by host `sple_atue_trx` bit 4. The radio keys an
 * unmodulated carrier at the scan power instead of the modulator. */
void App_Ptt_SetSwrScan(bool on);

/* True while the SWR-scan source owns TX and the host bit is still held. The
 * carrier and ALC tune mode are gated on this, not on the source latch alone.
 * ISR-safe (plain read of a volatile flag). */
bool App_Ptt_IsSwrScanActive(void);

/* Source that caused the current TX request. It is latched when the first
 * source becomes active and stays until every source is released. */
AppPttSource App_Ptt_GetSource(void);

/* Protection requested RX. If a source is active it stays latched: no TX
 * re-entry until every source is released. Drops the radio to RX when needed. */
void App_Ptt_ForceRx(void);

/* Snapshot of the source flags and the latch under the IRQ lock. */
void App_Ptt_GetStatus(AppPttStatus *out);
