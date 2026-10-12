#include "sdr_fm_disc.h"

#include <math.h>
#include <stdint.h>

#include "bsp_cordic.h"

void Sdr_FmDisc_Init(Sdr_FmDisc *d, const float *coeffs, uint16_t taps,
                     float *state, uint32_t block, float turns_threshold) {
    d->state           = state;
    d->state_len       = block + taps - 1u;
    d->turns_threshold = turns_threshold;
    arm_fir_init_f32(&d->diff, taps, coeffs, state, block);
    Sdr_FmDisc_Reset(d);
}

void Sdr_FmDisc_Reset(Sdr_FmDisc *d) {
    d->prev   = 0.0f;
    d->offset = 0.0f;
    arm_fill_f32(0.0f, d->state, d->state_len);
}

void Sdr_FmDisc_Process(Sdr_FmDisc *d, const float *i, const float *q,
                        float *out, uint32_t size) {
    float th = d->turns_threshold;

    /* Keep the accumulated turn carry bounded: shift the carry, the previous
     * phase and the FIR state by one threshold (phase-equivalent). */
    if (d->offset > th) {
        d->offset -= th;
        d->prev -= th;
        arm_offset_f32(d->state, -th, d->state, d->state_len);
    } else if (d->offset < -th) {
        d->offset += th;
        d->prev += th;
        arm_offset_f32(d->state, th, d->state, d->state_len);
    }

    Bsp_Cordic_Atan2(i, q, out, size);
    for (uint32_t n = 0; n < size; ++n) {
        out[n] += d->offset;
        if (out[n] - d->prev > (float)M_PI) {
            out[n] -= (float)M_TWOPI;
            d->offset -= (float)M_TWOPI;
        } else if (out[n] - d->prev < -(float)M_PI) {
            out[n] += (float)M_TWOPI;
            d->offset += (float)M_TWOPI;
        }
        d->prev = out[n];
    }

    arm_fir_f32(&d->diff, out, out, size);
}
