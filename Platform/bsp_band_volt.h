#pragma once

#include <stdint.h>

/*
 * Band-voltage output on PA3 / TIM2_CH4: a 5 kHz PWM whose duty sets an analog
 * level through an external RC. The channel full scale is fixed by the board
 * RC/divider; callers pass the target level in millivolts and this module maps
 * it onto the timer compare.
 */

/* Start the PWM channel at its configured idle pulse. 0 = success, negative =
 * error. */
int Bsp_BandVolt_Init(void);

/* Set the output level in millivolts; values outside the channel span clamp to
 * the ends. Rewrites the compare only when the level actually changes. */
void Bsp_BandVolt_SetMillivolts(uint16_t mv);
