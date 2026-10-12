#pragma once

#include <math.h>
#include <arm_math.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float    *i;
    float    *q;
    uint32_t  size;
} Sdr_IQ_Data;


typedef enum {
    SDR_MOD_LSB,
    SDR_MOD_LSB_D,
    SDR_MOD_USB,
    SDR_MOD_USB_D,
    SDR_MOD_CW,
    SDR_MOD_CWR,
    SDR_MOD_AM,
    SDR_MOD_SAM,
    SDR_MOD_NFM,
    SDR_MOD_WFM,
} SdrModulation;


/*
 * Quadrature NCO
 */

typedef struct {
    float    real;
    float    imag;
    float    step_real;
    float    step_imag;
    uint32_t sample_cnt;
} Sdr_Nco;


static inline void Sdr_Nco_Init(Sdr_Nco *nco)
{
    nco->real       = 1.0f;
    nco->imag       = 0.0f;
    nco->step_real  = 1.0f;
    nco->step_imag  = 0.0f;
    nco->sample_cnt = 0u;
}


static inline void Sdr_Nco_SetStep(Sdr_Nco *nco, float re, float im)
{
    nco->step_real = re;
    nco->step_imag = im;
}

/* Advance one sample. */
static inline void Sdr_Nco_Step(Sdr_Nco *nco)
{
    float next_real = nco->real * nco->step_real - nco->imag * nco->step_imag;
    nco->imag = nco->real * nco->step_imag + nco->imag * nco->step_real;
    nco->real = next_real;
    nco->sample_cnt = (nco->sample_cnt + 1) & (2048u - 1u);
    if (nco->sample_cnt == 0u) {
        float inv_len = 1.0f / sqrtf(nco->real * nco->real + nco->imag * nco->imag);
        nco->real *= inv_len;
        nco->imag *= inv_len;
    }
}

/**
 * DC filter
 */

typedef struct {
    float alpha;
    float val;
    bool invalid;
} Sdr_DC_Filter;


static inline float Sdr_DC_Filter_Apply(Sdr_DC_Filter *dc, float val) {
    if (!dc->invalid) {
        dc->val += (val - dc->val) * dc->alpha;
        val -= dc->val;
    } else {
        dc->invalid = false;
        dc->val = val;
        val = 0.0f;
    }
    return val;
}

static inline void Sdr_DC_Filter_Reset(Sdr_DC_Filter *dc) {
    dc->invalid = true;
}

/*
 * Helper functions
 */

static inline float Sdr_Lin2Db(float val) {
    /* 20 * log10(x) = 20 * ln(x) / ln(10) = 8.685889638 * ln(x) */
    return 8.685889638f * logf(val > FLT_MIN ? val : FLT_MIN);
}

static inline float Sdr_Pow2Db(float val) {
    /* 10 * log10(x) = 10 * ln(x) / ln(10) = 4.342944819032518 * ln(x) */
    return 4.342944819032518f * logf(val > FLT_MIN ? val : FLT_MIN);
}


static inline float Sdr_Db2Lin(float val) {
    /* 10^(x/20) = exp(x * ln(10) / 20) = exp(x * 0.1151292546) */
    return expf(val * 0.1151292546f);
}
