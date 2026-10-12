#include "bsp_band_volt.h"

#include "bsp_log.h"
#include "stm32h7xx_hal.h"
#include "tim.h"

#define LOG_TAG "bsp.band_volt"

/* Channel full scale in millivolts: the RC-filtered PWM reaches this level at
 * 100% duty (compare = period + 1). */
#define BSP_BAND_VOLT_SPAN_MV 6600u

#define BSP_BAND_VOLT_NONE    0xFFFFu

static uint16_t s_applied_mv = BSP_BAND_VOLT_NONE;

int Bsp_BandVolt_Init(void) {
    s_applied_mv = BSP_BAND_VOLT_NONE;

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK) {
        LOG_ERROR("pwm start failed");
        return -1;
    }
    LOG_INFO("band volt pwm started");
    return 0;
}

void Bsp_BandVolt_SetMillivolts(uint16_t mv) {
    uint32_t period = htim2.Init.Period;
    uint32_t ccr;

    if (mv > BSP_BAND_VOLT_SPAN_MV) {
        mv = BSP_BAND_VOLT_SPAN_MV;
    }
    if (mv == s_applied_mv) {
        return;
    }
    ccr = ((period + 1u) * mv) / BSP_BAND_VOLT_SPAN_MV;
    if (ccr > period) {
        ccr = period;
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr);
    s_applied_mv = mv;
}
