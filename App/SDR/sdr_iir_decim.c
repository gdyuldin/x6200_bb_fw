#include "sdr_iir_decim.h"

#include <stdint.h>

void Sdr_IirDecim_Init(SdrIirDecim *d, const float32_t *coeffs,
                       uint8_t num_sections, float *state, float *scratch)
{
    d->scratch = scratch;
    arm_biquad_cascade_df1_init_f32(&d->iir, num_sections, coeffs, state);
    Sdr_IirDecim_Reset(d);
}

void Sdr_IirDecim_Reset(SdrIirDecim *d)
{
    arm_fill_f32(0.0f, d->iir.pState, 4u * d->iir.numStages);
}

void Sdr_IirDecim_Process(SdrIirDecim *d, const float *in, float *out,
                          uint32_t in_size)
{
    arm_biquad_cascade_df1_f32(&d->iir, in, d->scratch, in_size);
    for (uint32_t n = 0; n < in_size / 2u; ++n) {
        out[n] = d->scratch[2u * n];
    }
}