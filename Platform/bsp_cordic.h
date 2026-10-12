#pragma once

#include "stdint.h"

void Bsp_Cordic_SinCos(const float *angles_rad, float *out_sin, float *out_cos, uint32_t size);
void Bsp_Cordic_Atan2(const float *x_arr, const float *y_arr, float *out_atan2, uint32_t size);
