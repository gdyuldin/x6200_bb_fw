#pragma once

#include <stdint.h>

#include <arm_math.h>

/*
 * Block half-band FIR decimator (decimation by 2).
 *
 * A half-band lowpass has a centre tap of 0.5, and every tap an even distance
 * from the centre is zero. Decimation by 2 only needs the non-zero taps, so
 * the multiply-accumulate count is roughly halved compared with the full FIR.
 * coeffs are the time-reversed CMSIS-style array (zeros included), state holds
 * numTaps + blockSize - 1 floats, and blockSize must be even.
 *
 * The instance is caller-owned so independent consumers can each hold their own
 * state. The caller keeps the stopband edge at or below fs_out - f_pass so
 * aliasing folded by the halved rate stays outside the passband.
 */
typedef struct {
    const float32_t *pCoeffs;
    float32_t       *pState;
    uint32_t         numTaps;
    uint32_t         blockSize;
    uint32_t         phase;
    uint32_t         center;
} SdrHalfbandDecim;

void Sdr_HalfbandDecim_Init(SdrHalfbandDecim *d, const float32_t *coeffs,
                            uint32_t numTaps, float32_t *state,
                            uint32_t blockSize);
void Sdr_HalfbandDecim_Reset(SdrHalfbandDecim *d);
void Sdr_HalfbandDecim_Process(SdrHalfbandDecim *d, const float32_t *src,
                               float32_t *dst, uint32_t blockSize);