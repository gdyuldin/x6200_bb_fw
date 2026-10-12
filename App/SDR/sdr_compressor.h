#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float rms_in_db;
    float rms_out_db;
    float applied_db;
    float threshold;
    bool  enabled;
} SdrCompressorStats;

void Sdr_Compressor_Init(void);
void Sdr_Compressor_SetRatio(uint8_t ratio);
void Sdr_Compressor_SetThreshold(float db);
void Sdr_Compressor_ResetThreshold(void);
void Sdr_Compressor_Process(float *audio, uint32_t size);

void Sdr_Compressor_Enable(bool on);
void Sdr_Compressor_GetStats(SdrCompressorStats *out);
