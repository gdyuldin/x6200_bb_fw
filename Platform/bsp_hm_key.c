#include "bsp_hm_key.h"

#include "stm32h7xx_hal.h"
#include "tim.h"

static struct {
    BspHmKeyEdgeFn fn;
    void          *ctx;
} s_edge;

/*
 * TIM4 runs as PWM input: the slave controller resets the counter on each
 * TI1FP1 rising edge, so CCR1 holds the full cell period. CCR2 is the indirect
 * capture of the same pin on the falling edge, so at the rising-edge callback
 * it already holds the high phase of the cell that just ended.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM4) {
        return;
    }
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) {
        return;
    }
    if (s_edge.fn == 0) {
        return;
    }
    s_edge.fn(htim->Instance->CCR1, htim->Instance->CCR2, s_edge.ctx);
}

int Bsp_HmKey_Start(BspHmKeyEdgeFn fn, void *ctx) {
    if (fn == 0) {
        return -1;
    }

    s_edge.fn  = fn;
    s_edge.ctx = ctx;

    if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK) {
        s_edge.fn  = 0;
        s_edge.ctx = 0;
        return -1;
    }
    /* Channel 2 shares the pin and only feeds CCR2; its interrupt is enabled
     * so HAL clears the capture flag. The decoder uses channel 1 only. */
    if (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2) != HAL_OK) {
        (void)HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
        s_edge.fn  = 0;
        s_edge.ctx = 0;
        return -1;
    }
    return 0;
}