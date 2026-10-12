#pragma once

#include "sdr_common.h"

/*
 * Shared FM phase discriminator: atan2 -> phase unwrap (with a continuous
 * +/-2pi turn carry) -> FIR differentiator.  The output is the raw
 * discriminator signal in cycles per sample; the caller applies its own
 * deviation scaling, de-emphasis and audio filtering.
 *
 * The turn carry is recentred by turns_threshold (radians, a multiple of 2pi)
 * so it cannot grow without bound and lose float precision.  Smaller threshold
 * keeps the carry smaller but recentres (rewrites the FIR state) more often.
 */
typedef struct {
    arm_fir_instance_f32 diff;
    float   *state;
    uint32_t state_len;
    float    prev;
    float    offset;
    float    turns_threshold;
} Sdr_FmDisc;

/* state must hold block + taps - 1 floats. */
void Sdr_FmDisc_Init(Sdr_FmDisc *d, const float *coeffs, uint16_t taps,
                     float *state, uint32_t block, float turns_threshold);
void Sdr_FmDisc_Reset(Sdr_FmDisc *d);
void Sdr_FmDisc_Process(Sdr_FmDisc *d, const float *i, const float *q,
                        float *out, uint32_t size);
