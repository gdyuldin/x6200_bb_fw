#include "sdr_halfband_decim.h"

#include <stdint.h>
#include <string.h>

void Sdr_HalfbandDecim_Init(SdrHalfbandDecim *d, const float32_t *coeffs,
                            uint32_t numTaps, float32_t *state,
                            uint32_t blockSize)
{
    d->numTaps   = 0u;
    d->phase     = 0u;
    d->center    = 0u;
    d->pCoeffs   = coeffs;
    d->pState    = state;
    d->blockSize = blockSize;

    if (((numTaps & 1u) == 0u) || ((blockSize & 1u) != 0u) || (numTaps == 0u)) {
        return;
    }

    d->numTaps = numTaps;
    d->center  = (numTaps - 1u) / 2u;

    /* Non-zero off-centre taps share one index parity. */
    for (uint32_t i = 0u; i < numTaps; ++i) {
        if ((i != d->center) && (coeffs[i] != 0.0f)) {
            d->phase = i & 1u;
            break;
        }
    }
}

void Sdr_HalfbandDecim_Reset(SdrHalfbandDecim *d)
{
    if (d->numTaps == 0u) {
        return;
    }
    arm_fill_f32(0.0f, d->pState, d->numTaps + d->blockSize - 1u);
}

void Sdr_HalfbandDecim_Process(SdrHalfbandDecim *d, const float32_t *src,
                               float32_t *dst, uint32_t blockSize)
{
    const float32_t *coeffs  = d->pCoeffs;
    float32_t       *state   = d->pState;
    uint32_t         numTaps = d->numTaps;
    uint32_t         phase   = d->phase;
    uint32_t         center  = d->center;

    if ((numTaps == 0u) || (blockSize != d->blockSize)) {
        return;
    }

    /* Append the new block after the (numTaps - 1) history samples. */
    arm_copy_f32(src, state + numTaps - 1u, blockSize);

    float32_t *base = state;
    for (uint32_t n = 0u; n < blockSize / 2u; ++n) {
        const float32_t *s   = base;
        float32_t        acc = 0.0f;

        /* The centre tap has the opposite parity to the non-zero off-centre
         * taps, so it is added separately. */
        for (uint32_t k = phase; k < numTaps; k += 2u) {
            acc += coeffs[k] * s[k];
        }
        acc += coeffs[center] * s[center];

        dst[n] = acc;
        base  += 2u;
    }

    /* Keep the last (numTaps - 1) samples as the next block's history. The
     * source and destination overlap, so memmove is required. */
    memmove(state, state + blockSize, (numTaps - 1u) * sizeof(float32_t));
}