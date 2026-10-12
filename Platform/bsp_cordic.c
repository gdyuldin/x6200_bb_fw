#include "bsp_cordic.h"

#include <stdbool.h>
#include <arm_math.h>
#include "stm32h7xx_ll_cordic.h"

#define PRECISION (CORDIC_PRECISION_5CYCLES)
#define Q31_SCALE (2147483648.0f)

#define FIX_INF(v) (isinff(v) ? copysignf(FLT_MAX, v) : v)
#define FIX_NAN(v) (isnanf(v) ? 0.0f : v)


static inline int32_t angle_float_to_q31(float val) {
    val = FIX_NAN(val);
    val = FIX_INF(val);
    while (val > (float)M_PI) {
        val -= (float)M_TWOPI;
    }
    while (val < -(float)M_PI) {
        val += (float)M_TWOPI;
    }
    return clip_q63_to_q31((q63_t)(val / (float)M_PI * Q31_SCALE));
}

static inline void xy_float_to_q31(float x_src, float y_src, int32_t *x_dst, int32_t *y_dst) {
    x_src = FIX_NAN(x_src);
    x_src = FIX_INF(x_src);
    y_src = FIX_NAN(y_src);
    y_src = FIX_INF(y_src);
    float x_abs = fabsf(x_src);
    float y_abs = fabsf(y_src);
    float max_val = MAX(x_abs, y_abs);
    if (max_val == 0.0f) {
        *x_dst = Q31_MAX;
        *y_dst = 0;
    } else {
        float k = Q31_SCALE / max_val;
        *x_dst = clip_q63_to_q31((q63_t)(x_src * k));
        *y_dst = clip_q63_to_q31((q63_t)(y_src * k));
    }
}


void Bsp_Cordic_SinCos(const float *angles_rad, float *out_sin, float *out_cos, uint32_t size)
{
    int32_t angle_q31, cos_q31, sin_q31;

    if (!angles_rad || !out_sin || !out_cos || size == 0) {
        return;
    }

    // Setup CORDIC
    CORDIC->CSR = (CORDIC_FUNCTION_COSINE) |
                 (PRECISION) |
                 (CORDIC_SCALE_0) |
                 (CORDIC_NBWRITE_2) |
                 (CORDIC_NBREAD_2) |
                 (CORDIC_INSIZE_32BITS) |
                 (CORDIC_OUTSIZE_32BITS);

    // Put first sample
    float angle_0 = angles_rad[0];
    CORDIC->WDATA = angle_float_to_q31(angle_0);
    CORDIC->WDATA = Q31_MAX;

    for (int i = 1; i < size; i++)
    {
        // Prepare next sample
        float next_angle = angles_rad[i];
        angle_q31 = angle_float_to_q31(next_angle);

        // Send next sample
        CORDIC->WDATA = angle_q31;
        CORDIC->WDATA = Q31_MAX;

        while (!(CORDIC->CSR & CORDIC_CSR_RRDY));

        // Read data
        cos_q31 = CORDIC->RDATA;
        sin_q31 = CORDIC->RDATA;


        out_cos[i - 1] = (float)cos_q31 / Q31_SCALE;
        out_sin[i - 1] = (float)sin_q31 / Q31_SCALE;
    }

    // Wait last sample
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY));

    cos_q31 = CORDIC->RDATA;
    sin_q31 = CORDIC->RDATA;

    out_cos[size - 1] = (float)cos_q31 / Q31_SCALE;
    out_sin[size - 1] = (float)sin_q31 / Q31_SCALE;
}


void Bsp_Cordic_Atan2(const float *x_arr, const float *y_arr, float *out_atan2, uint32_t size)
{
    int32_t x, y, angle_q31;

    if (!x_arr || !y_arr || !out_atan2 || size == 0) {
        return;
    }

    // Setup CORDIC
    CORDIC->CSR = (CORDIC_FUNCTION_PHASE) |
                 (PRECISION) |
                 (CORDIC_SCALE_0) |
                 (CORDIC_NBWRITE_2) |
                 (CORDIC_NBREAD_1) |
                 (CORDIC_INSIZE_32BITS) |
                 (CORDIC_OUTSIZE_32BITS);

    // Put first sample
    xy_float_to_q31(x_arr[0], y_arr[0], &x, &y);
    CORDIC->WDATA = x;
    CORDIC->WDATA = y;

    for (int i = 1; i < size; i++) {
        xy_float_to_q31(x_arr[i], y_arr[i], &x, &y);

        // Send next sample
        CORDIC->WDATA = x;
        CORDIC->WDATA = y;

        while (!(CORDIC->CSR & CORDIC_CSR_RRDY));

        // Read phase (angle)
        angle_q31 = CORDIC->RDATA;

        // Convert to radians
        out_atan2[i - 1] = ((float)angle_q31 / Q31_SCALE) * (float)M_PI;
    }

    // Wait last sample
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY));
    angle_q31 = CORDIC->RDATA;
    out_atan2[size - 1] = ((float)angle_q31 / Q31_SCALE) * (float)M_PI;
}
