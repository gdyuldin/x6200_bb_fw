#pragma once

#include <stdint.h>

#include "sdr_common.h"

typedef enum {
    SDR_AGC_OFF,
    SDR_AGC_SLOW,
    SDR_AGC_FAST,
    SDR_AGC_AUTO
} SdrAgcMode;

void Sdr_AGC_Init(void);
/* Direct setter for the AGC speed; the modulation is taken from SdrRxCfg. */
void Sdr_AGC_SetMode(SdrAgcMode mode);
void Sdr_AGC_ResetStats(void);
void Sdr_AGC_Process(float *samples, uint32_t size);
