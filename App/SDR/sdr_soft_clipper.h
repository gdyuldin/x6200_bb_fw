#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SDR_SOFTCLIPPER_OK           0
#define SDR_SOFTCLIPPER_ERR_TH       (-1)
#define SDR_SOFTCLIPPER_ERR_MAX_CEIL (-2)

typedef struct {
    float T;          /* Applying threshold */
    float M;          /* Max output level */
    float delta_sq;
    float denom_offs;
    bool neg_only;
} SdrSoftClipperConfig;

int Sdr_SoftClipper_Setup(SdrSoftClipperConfig *cfg, float threshold, float max_ceil, bool neg_only);

void Sdr_SoftClipper_Process(const SdrSoftClipperConfig *cfg, const float *src, float *dst, uint32_t n);
