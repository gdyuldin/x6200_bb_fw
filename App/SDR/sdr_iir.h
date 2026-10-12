#pragma once

#include <stdint.h>

#include <arm_math.h>

#include "sdr_iir_orders.h"

/* Chebyshev Type II and elliptic LPF/HPF design for CMSIS-DSP
 * arm_biquad_cascade_df1_f32.  The two families share the section-count keys,
 * the design pipeline and the response report; they differ only in the analog
 * prototype selected. */
typedef enum {
    SDR_IIR_TYPE_LPF = 0,
    SDR_IIR_TYPE_HPF = 1,
} SdrIirType;

/* Which point of the response f_edge_hz refers to. */
typedef enum {
    SDR_IIR_EDGE_PASSBAND = 0,  /* passband reference point (default) */
    SDR_IIR_EDGE_STOPBAND = 1,  /* start of the full stopband rejection */
    SDR_IIR_EDGE_NOISE_BW = 2,  /* type-dependent noise band, see below */
} SdrIirEdge;

/*
 * SDR_IIR_EDGE_PASSBAND is the prototype reference point: for the Chebyshev II
 * family its -9 dB point, for the elliptic family its passband ripple edge.
 */

/*
 * In SDR_IIR_EDGE_NOISE_BW the meaning of f_edge_hz follows the filter type:
 * LPF requests the one-sided equivalent noise bandwidth of the passband,
 * HPF requests the equivalent removed (blocked) bandwidth.  The integrand is
 * chosen by type, so an HPF can never be asked for a passband ENBW.
 */
typedef struct {
    SdrIirType   type;    /* LPF or HPF */
    SdrIirEdge   edge;    /* meaning of f_edge_hz */
    SdrIirStages stages;  /* biquad sections; picks the generated prototype */
    float        fs_hz;   /* sample rate */
    float        f_edge_hz; /* response edge selected by edge */
} SdrIirSpec;

/*
 * Response report for a designed filter.  Frequencies are in Hz, levels in dB.
 * noise_bw_hz is the LPF passband ENBW or the HPF blocked bandwidth, following
 * the filter type; for an HPF it is not a passband ENBW.
 */
typedef struct {
    float f_ref_hz;      /* prototype reference point (see SdrIirEdge) */
    float f_stop_hz;     /* start of the full stopband rejection */
    float noise_bw_hz;   /* LPF: passband ENBW; HPF: blocked bandwidth */
    float enbw_gain_db;  /* LPF: gain at noise_bw_hz; HPF: 0 */
    float level_hz;      /* boundary at the ENBW attenuation level */
    float level_gain_db; /* that level, signed dB; same for both types */
} SdrIirResponse;

/* Section counts are SDR_IIR_STAGES_* from the generated sdr_iir_orders.h,
 * so a dropped order is a compile error at the caller. */
#define SDR_IIR_COEFFS_PER_STAGE  5u

#define SDR_IIR_OK                0
#define SDR_IIR_ERR_ARG           (-1)
#define SDR_IIR_ERR_RANGE         (-2)

/*
 * Compute CMSIS DF1 coefficients for a Chebyshev Type II LPF/HPF.
 *
 * The analog prototype is fixed at 100 dB stopband rejection and normalized to
 * its -9 dB point at 1 rad/s; spec->stages selects which generated prototype,
 * and an unsupported count is rejected.  f_edge_hz selects the passband
 * reference point, the start of the stopband rejection, or -- in NOISE_BW --
 * the type-dependent noise band.  The frequency scaling and bilinear transform
 * are applied for the requested Fs/f_edge_hz.  coeffs[] receives 5*num_stages
 * floats in CMSIS order: {b0, b1, b2, a1, a2} per stage.
 *
 * When info is not 0 it receives the response report; passing 0 skips the
 * measurement work.
 *
 * Returns SDR_IIR_OK on success, a negative error code otherwise.
 */
int Sdr_Iir_DesignCheby2Ex(const SdrIirSpec *spec, float32_t *coeffs,
                           uint8_t *num_stages, SdrIirResponse *info);

/* Same as Sdr_Iir_DesignCheby2Ex without the response report. */
int Sdr_Iir_DesignCheby2(const SdrIirSpec *spec, float32_t *coeffs,
                         uint8_t *num_stages);

/*
 * Compute CMSIS DF1 coefficients for an elliptic (Cauer) LPF/HPF.
 *
 * Same interface and semantics as Sdr_Iir_DesignCheby2Ex, but the prototype is
 * elliptic: 0.1 dB passband ripple and 80 dB stopband rejection, normalized to
 * the passband ripple edge at 1 rad/s.  For a given order the transition band
 * is the narrowest of the generated families, at the cost of group-delay
 * peaking near the band edge.  spec->stages selects the elliptic prototype,
 * accepted for SDR_IIR_STAGES_4 (order 8) and SDR_IIR_STAGES_5 (order 10); any
 * other count is rejected.
 *
 * Returns SDR_IIR_OK on success, a negative error code otherwise.
 */
int Sdr_Iir_DesignEllipEx(const SdrIirSpec *spec, float32_t *coeffs,
                          uint8_t *num_stages, SdrIirResponse *info);

/* Same as Sdr_Iir_DesignEllipEx without the response report. */
int Sdr_Iir_DesignEllip(const SdrIirSpec *spec, float32_t *coeffs,
                        uint8_t *num_stages);