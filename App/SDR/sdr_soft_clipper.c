#include "sdr_soft_clipper.h"

#include <math.h>

int Sdr_SoftClipper_Setup(SdrSoftClipperConfig *cfg, float threshold, float max_ceil, bool neg_only)
{
    if (threshold > max_ceil) {
        return SDR_SOFTCLIPPER_ERR_TH;
    }
    if (threshold <= 0.0f) {
        return SDR_SOFTCLIPPER_ERR_TH;
    }
    if (max_ceil <= 0.0f) {
        return SDR_SOFTCLIPPER_ERR_MAX_CEIL;
    }
    cfg->T = threshold;
    cfg->M = max_ceil;
    float delta = max_ceil - threshold;
    cfg->delta_sq = delta * delta;
    cfg->denom_offs = max_ceil - (2.0f * threshold);
    cfg->neg_only = neg_only;
    return SDR_SOFTCLIPPER_OK;
}

void Sdr_SoftClipper_Process(const SdrSoftClipperConfig *cfg, const float *src, float *dst, uint32_t n)
{
    const float T     = cfg->T;
    const float M     = cfg->M;
    const float dsq   = cfg->delta_sq;
    const float off   = cfg->denom_offs;
    const bool  neg   = cfg->neg_only;

    for (uint32_t i = 0; i < n; i++) {
        float v = src[i];
        if (neg && v > 0.0f) {
            dst[i] = v;
            continue;
        }
        float a = fabsf(v);
        dst[i] = (a <= T) ? v : copysignf(M - dsq / (a + off), v);
    }
}
