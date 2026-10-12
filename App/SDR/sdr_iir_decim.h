#pragma once

#include <stdint.h>

#include <arm_math.h>

/*
 * Block IIR decimator: a CMSIS DF1 biquad cascade run at the input rate,
 * emitting every second output sample. The filter is recursive, so every input
 * sample is processed; decimation by 2 only halves the output rate.
 *
 * The instance is caller-owned so independent consumers (RX frontend, spectrum)
 * can each hold their own state and coefficient set. coeffs are CMSIS DF1
 * {b0, b1, b2, a1, a2} per section, state holds 4 * sections floats, and
 * scratch must hold at least in_size floats of the full input rate.
 *
 * An IIR cascade has non-linear group delay and cannot fuse decimation, so the
 * whole input block is filtered and the output is sampled at every second
 * point. The caller keeps the stopband edge at or below fs_out - f_pass so
 * aliasing folded by the halved rate stays outside the passband.
 */
typedef struct {
    arm_biquad_casd_df1_inst_f32 iir;
    float                       *scratch;
} SdrIirDecim;

void Sdr_IirDecim_Init(SdrIirDecim *d, const float32_t *coeffs,
                       uint8_t num_sections, float *state, float *scratch);
void Sdr_IirDecim_Reset(SdrIirDecim *d);
void Sdr_IirDecim_Process(SdrIirDecim *d, const float *in, float *out,
                          uint32_t in_size);