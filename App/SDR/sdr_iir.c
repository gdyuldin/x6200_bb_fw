#include "sdr_iir.h"

#include <math.h>
#include <stddef.h>

#include "sdr_iir_prototypes.h"
#include "sdr_common.h"

/* Noise-band grid: at 12 kHz Fs this is well under 1 Hz per point, which keeps
 * the narrowest channel filters resolved, while 8192 points still fits the
 * occasional redesign budget. */
#define SDR_IIR_NOISE_GRID 8192u

/* Secant budget and relative band tolerance for SDR_IIR_EDGE_NOISE_BW. */
#define SDR_IIR_NOISE_ITERS 6u
#define SDR_IIR_NOISE_TOL   1e-3

static const float *prototype_select(SdrIirFamily family, SdrIirType type,
                                     uint8_t stages, float *stop_edge,
                                     float *noise_norm, float *level_att_db)
{
    const uint32_t count = (uint32_t)(sizeof(sdr_iir_protos) /
                                      sizeof(sdr_iir_protos[0]));
    const SdrIirProto *entry = 0;
    for (uint32_t index = 0u; index < count; ++index) {
        if (sdr_iir_protos[index].family == family &&
            sdr_iir_protos[index].stages == stages) {
            entry = &sdr_iir_protos[index];
            break;
        }
    }
    if (entry == 0) {
        return 0;
    }

    *level_att_db = entry->level_att_db;
    if (type == SDR_IIR_TYPE_LPF) {
        *stop_edge = entry->stop_edge_lpf;
        *noise_norm = entry->noise_norm_lpf;
        return entry->lpf;
    }
    *stop_edge = entry->stop_edge_hpf;
    *noise_norm = entry->noise_norm_hpf;
    return entry->hpf;
}

static int design_sections(const float *proto, uint8_t stages, double omega_c,
                           double fs, float32_t *coeffs)
{
    const double k_bilinear = 2.0 * fs;
    const double omega_c2 = omega_c * omega_c;
    const double k2 = k_bilinear * k_bilinear;

    for (uint8_t stage = 0u; stage < stages; ++stage) {
        const float *section = proto + stage * 6u;
        const double b0 = (double)section[0];
        const double b1 = (double)section[1];
        const double b2 = (double)section[2];
        const double a0 = (double)section[3];
        const double a1 = (double)section[4];
        const double a2 = (double)section[5];

        /* H(s/omega_c): multiply by omega_c^2 to keep integer powers. */
        const double B0 = b0 * omega_c2;
        const double B1 = b1 * omega_c;
        const double B2 = b2;
        const double A0 = a0 * omega_c2;
        const double A1 = a1 * omega_c;
        const double A2 = a2;

        /* Bilinear transform s = 2*Fs*(z-1)/(z+1). */
        const double N2 = B0 + B1 * k_bilinear + B2 * k2;
        const double N1 = 2.0 * B0 - 2.0 * B2 * k2;
        const double N0 = B0 - B1 * k_bilinear + B2 * k2;
        const double D2 = A0 + A1 * k_bilinear + A2 * k2;
        const double D1 = 2.0 * A0 - 2.0 * A2 * k2;
        const double D0 = A0 - A1 * k_bilinear + A2 * k2;

        if (D2 == 0.0) {
            return SDR_IIR_ERR_RANGE;
        }

        const double inv_d2 = 1.0 / D2;
        float32_t *out = coeffs + stage * SDR_IIR_COEFFS_PER_STAGE;
        out[0] = (float32_t)(N2 * inv_d2);
        out[1] = (float32_t)(N1 * inv_d2);
        out[2] = (float32_t)(N0 * inv_d2);
        /* CMSIS DF1 expects negated denominator coefficients. */
        out[3] = (float32_t)(-D1 * inv_d2);
        out[4] = (float32_t)(-D0 * inv_d2);
    }

    return SDR_IIR_OK;
}

/* Cascade |H|^2 at z^-1 = cos(w) - j*sin(w), from the stored coefficients. */
static double cascade_power(const float32_t *coeffs, uint8_t stages,
                            double cw, double sw)
{
    const double c2 = cw * cw - sw * sw;
    const double s2 = 2.0 * cw * sw;
    double num = 1.0;
    double den = 1.0;

    for (uint8_t stage = 0u; stage < stages; ++stage) {
        const float32_t *section = coeffs + stage * SDR_IIR_COEFFS_PER_STAGE;
        const double b0 = (double)section[0];
        const double b1 = (double)section[1];
        const double b2 = (double)section[2];
        const double a1 = (double)section[3];
        const double a2 = (double)section[4];

        const double nr = b0 + b1 * cw + b2 * c2;
        const double ni = -(b1 * sw + b2 * s2);
        const double dr = 1.0 - a1 * cw - a2 * c2;
        const double di = a1 * sw + a2 * s2;

        num *= nr * nr + ni * ni;
        den *= dr * dr + di * di;
    }

    return den > 0.0 ? num / den : 0.0;
}

static double response_power_at(const float32_t *coeffs, uint8_t stages,
                                float fs_hz, float f_hz)
{
    const double w = 2.0 * M_PI * (double)f_hz / (double)fs_hz;
    return cascade_power(coeffs, stages, cos(w), sin(w));
}

/*
 * Digital noise band over [0, Fs/2] from the coefficients, so bilinear warping
 * is included.  LPF accumulates |H|^2/|H(0)|^2 (passband ENBW); HPF accumulates
 * 1 - |H|^2/|H(Fs/2)|^2 (blocked band), which converges where the analog ENBW
 * of a high-pass does not.
 */
static int noise_band_digital(SdrIirType type, const float32_t *coeffs,
                              uint8_t stages, float fs_hz, float *band_hz)
{
    const double step = M_PI / (double)SDR_IIR_NOISE_GRID;
    const double cos_step = cos(step);
    const double sin_step = sin(step);
    const double ref_pow = type == SDR_IIR_TYPE_LPF
                               ? cascade_power(coeffs, stages, 1.0, 0.0)
                               : cascade_power(coeffs, stages, -1.0, 0.0);
    if (!(ref_pow > 0.0)) {
        return SDR_IIR_ERR_RANGE;
    }

    double sum = 0.0;
    double cw = 1.0;
    double sw = 0.0;
    for (uint32_t k = 0u; k <= SDR_IIR_NOISE_GRID; ++k) {
        /* Periodic re-normalization keeps the rotating phasor accurate. */
        if ((k & 1023u) == 0u) {
            const double angle = step * (double)k;
            cw = cos(angle);
            sw = sin(angle);
        }
        const double power = cascade_power(coeffs, stages, cw, sw);
        double value = type == SDR_IIR_TYPE_LPF
                           ? power / ref_pow
                           : 1.0 - power / ref_pow;
        if (value < 0.0) {
            value = 0.0;
        }
        const double weight = (k == 0u || k == SDR_IIR_NOISE_GRID) ? 0.5 : 1.0;
        sum += weight * value;

        const double next_cw = cw * cos_step - sw * sin_step;
        const double next_sw = sw * cos_step + cw * sin_step;
        cw = next_cw;
        sw = next_sw;
    }

    *band_hz = (float)(0.5 * (double)fs_hz * sum / (double)SDR_IIR_NOISE_GRID);
    return SDR_IIR_OK;
}

/* Frequency where |H|, relative to the type's passband reference, reaches
 * level_db (negative).  LPF falls through the level, HPF rises through it. */
static float response_level_hz(SdrIirType type, const float32_t *coeffs,
                               uint8_t stages, float fs_hz, float level_db)
{
    const double target = pow(10.0, (double)level_db / 20.0);
    const double ref_pow = type == SDR_IIR_TYPE_LPF
                               ? cascade_power(coeffs, stages, 1.0, 0.0)
                               : cascade_power(coeffs, stages, -1.0, 0.0);
    double lo = 0.0;
    double hi = 0.5 * (double)fs_hz;

    if (!(ref_pow > 0.0)) {
        return 0.0f;
    }

    for (uint8_t i = 0u; i < 32u; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double ratio = sqrt(response_power_at(coeffs, stages, fs_hz,
                                                     (float)mid) / ref_pow);
        if (type == SDR_IIR_TYPE_LPF ? ratio > target : ratio < target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    return (float)(0.5 * (lo + hi));
}

static int fill_response(SdrIirType type, const float32_t *coeffs,
                         uint8_t stages, float fs_hz, double omega_c,
                         double stop_edge, float level_att_db,
                         float noise_bw_hz, SdrIirResponse *info)
{
    const double fs = (double)fs_hz;

    info->f_ref_hz = (float)(fs / M_PI * atan(omega_c / (2.0 * fs)));
    info->f_stop_hz = (float)(fs / M_PI * atan(stop_edge * omega_c / (2.0 * fs)));
    info->noise_bw_hz = noise_bw_hz;
    info->level_gain_db = -level_att_db;

    if (type == SDR_IIR_TYPE_LPF) {
        const double ref_pow = cascade_power(coeffs, stages, 1.0, 0.0);
        const double band_pow = response_power_at(coeffs, stages, fs_hz,
                                                  noise_bw_hz);
        info->enbw_gain_db = (ref_pow > 0.0 && band_pow > 0.0)
                                 ? Sdr_Lin2Db((float)sqrt(band_pow / ref_pow))
                                 : 0.0f;
    } else {
        info->enbw_gain_db = 0.0f;
    }

    info->level_hz = response_level_hz(type, coeffs, stages, fs_hz,
                                       info->level_gain_db);
    return SDR_IIR_OK;
}

static int noise_band_eval(const float *proto, uint8_t stages, SdrIirType type,
                           float fs_hz, double log_omega, float32_t *coeffs,
                           float *band_hz)
{
    int rc = design_sections(proto, stages, exp(log_omega), (double)fs_hz,
                             coeffs);
    if (rc != SDR_IIR_OK) {
        return rc;
    }
    return noise_band_digital(type, coeffs, stages, fs_hz, band_hz);
}

/* Solve the digital noise band for target_hz by secant steps on log(omega_c).
 * The analog prototype constant is only the starting guess; the objective is
 * the measured digital band. */
static int solve_noise_bw(const float *proto, uint8_t stages, SdrIirType type,
                          float fs_hz, double target_hz, double const_norm,
                          float32_t *coeffs, double *omega_c, float *band_hz)
{
    const double guess = 2.0 * M_PI * target_hz / const_norm;
    if (!(guess > 0.0) || !isfinite(guess)) {
        return SDR_IIR_ERR_RANGE;
    }
    const double lo = log(guess) - log(16.0);
    const double hi = log(guess) + log(16.0);

    double x0 = log(guess);
    double x1 = x0 + log(1.1);
    float b0 = 0.0f;
    float b1 = 0.0f;
    int rc = noise_band_eval(proto, stages, type, fs_hz, x0, coeffs, &b0);
    if (rc != SDR_IIR_OK) {
        return rc;
    }
    rc = noise_band_eval(proto, stages, type, fs_hz, x1, coeffs, &b1);
    if (rc != SDR_IIR_OK) {
        return rc;
    }

    double f0 = (double)b0 - target_hz;
    double f1 = (double)b1 - target_hz;
    for (uint8_t iter = 0u; iter < SDR_IIR_NOISE_ITERS; ++iter) {
        if (fabs(f1) <= SDR_IIR_NOISE_TOL * target_hz) {
            break;
        }
        if (f1 == f0) {
            break;
        }
        double x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
        if (x2 < lo) {
            x2 = lo;
        }
        if (x2 > hi) {
            x2 = hi;
        }
        x0 = x1;
        f0 = f1;
        x1 = x2;
        rc = noise_band_eval(proto, stages, type, fs_hz, x1, coeffs, &b1);
        if (rc != SDR_IIR_OK) {
            return rc;
        }
        f1 = (double)b1 - target_hz;
    }

    if (fabs(f1) > SDR_IIR_NOISE_TOL * target_hz) {
        return SDR_IIR_ERR_RANGE;
    }

    *omega_c = exp(x1);
    *band_hz = b1;
    return SDR_IIR_OK;
}

static int design_proto(SdrIirFamily family, const SdrIirSpec *spec,
                        float32_t *coeffs, uint8_t *num_stages,
                        SdrIirResponse *info)
{
    if (spec == 0 || coeffs == 0 || num_stages == 0) {
        return SDR_IIR_ERR_ARG;
    }
    if (spec->type != SDR_IIR_TYPE_LPF && spec->type != SDR_IIR_TYPE_HPF) {
        return SDR_IIR_ERR_ARG;
    }
    if (spec->edge != SDR_IIR_EDGE_PASSBAND &&
        spec->edge != SDR_IIR_EDGE_STOPBAND &&
        spec->edge != SDR_IIR_EDGE_NOISE_BW) {
        return SDR_IIR_ERR_ARG;
    }
    if (!(spec->fs_hz > 0.0f) || !(spec->f_edge_hz > 0.0f)) {
        return SDR_IIR_ERR_ARG;
    }
    /* tan(pi*f_edge/Fs) must remain finite; reject f_edge >= Fs/2. */
    if (spec->f_edge_hz >= 0.5f * spec->fs_hz) {
        return SDR_IIR_ERR_RANGE;
    }

    uint8_t stages = (uint8_t)spec->stages;
    float stop_edge = 0.0f;
    float noise_norm = 0.0f;
    float level_att_db = 0.0f;
    const float *proto = prototype_select(family, spec->type, stages, &stop_edge,
                                          &noise_norm, &level_att_db);
    if (proto == 0) {
        return SDR_IIR_ERR_ARG;
    }

    const double fs = (double)spec->fs_hz;
    const double f_edge = (double)spec->f_edge_hz;
    double omega_c = 0.0;
    float noise_bw_hz = 0.0f;

    if (spec->edge == SDR_IIR_EDGE_NOISE_BW) {
        int rc = solve_noise_bw(proto, stages, spec->type, spec->fs_hz, f_edge,
                                (double)noise_norm, coeffs, &omega_c,
                                &noise_bw_hz);
        if (rc != SDR_IIR_OK) {
            return rc;
        }
    } else {
        omega_c = 2.0 * fs * tan(M_PI * f_edge / fs);
        if (spec->edge == SDR_IIR_EDGE_STOPBAND) {
            omega_c /= (double)stop_edge;
        }
        int rc = design_sections(proto, stages, omega_c, fs, coeffs);
        if (rc != SDR_IIR_OK) {
            return rc;
        }
        if (info != 0) {
            rc = noise_band_digital(spec->type, coeffs, stages, spec->fs_hz,
                                    &noise_bw_hz);
            if (rc != SDR_IIR_OK) {
                return rc;
            }
        }
    }

    *num_stages = stages;
    if (info != 0) {
        return fill_response(spec->type, coeffs, stages, spec->fs_hz, omega_c,
                             (double)stop_edge, level_att_db, noise_bw_hz, info);
    }
    return SDR_IIR_OK;
}

int Sdr_Iir_DesignCheby2Ex(const SdrIirSpec *spec, float32_t *coeffs,
                           uint8_t *num_stages, SdrIirResponse *info)
{
    return design_proto(SDR_IIR_FAMILY_CHEBY2, spec, coeffs, num_stages, info);
}

int Sdr_Iir_DesignCheby2(const SdrIirSpec *spec, float32_t *coeffs,
                         uint8_t *num_stages)
{
    return Sdr_Iir_DesignCheby2Ex(spec, coeffs, num_stages, 0);
}

int Sdr_Iir_DesignEllipEx(const SdrIirSpec *spec, float32_t *coeffs,
                          uint8_t *num_stages, SdrIirResponse *info)
{
    return design_proto(SDR_IIR_FAMILY_ELLIP, spec, coeffs, num_stages, info);
}

int Sdr_Iir_DesignEllip(const SdrIirSpec *spec, float32_t *coeffs,
                        uint8_t *num_stages)
{
    return Sdr_Iir_DesignEllipEx(spec, coeffs, num_stages, 0);
}