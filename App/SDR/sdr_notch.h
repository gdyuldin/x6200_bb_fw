#pragma once

#include <stdint.h>

typedef enum {
    SDR_NOTCH_MODE_OFF = 0,
    SDR_NOTCH_MODE_MANUAL,
    SDR_NOTCH_MODE_AUTO,
} SdrNotchMode;


void Sdr_Notch_Init(void);

void Sdr_Notch_SetParams(SdrNotchMode mode, float center, float width);

void Sdr_Notch_Process(float *samples, uint32_t size);
