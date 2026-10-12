#pragma once

#include <stdint.h>

#include "sdr_common.h"

/*
 * Configuration snapshots published on the event bus. A block bundles the
 * fields that several independent consumers read while making one decision, so
 * a subscriber never observes a half-updated slice. SdrRxCfg is committed by
 * sdr_rx (it owns the RX resample generation); SdrTxCfg and SdrKeyToneCfg are
 * committed by app_radio_state.
 */

typedef struct {
    SdrModulation rx_mode;
    int32_t       filter_low_hz;
    int32_t       filter_high_hz;
    uint8_t       fftdec;
} SdrRxCfg;

typedef struct {
    SdrModulation tx_mode;
} SdrTxCfg;

typedef struct {
    uint32_t key_tone_hz;
} SdrKeyToneCfg;
