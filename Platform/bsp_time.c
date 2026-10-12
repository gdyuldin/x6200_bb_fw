#include "bsp_time.h"

#include "bsp_prof.h"
#include "stm32h7xx_hal.h"
#include "tim.h"

static struct {
    BspTimeTimerCallback callback;
    void                *ctx;
} s_timer_cb[BSP_TIME_TIMER_COUNT];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        if (s_timer_cb[BSP_TIME_TIMER_PROTECT].callback != 0) {
            BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_PROTECT, prof_start);
            s_timer_cb[BSP_TIME_TIMER_PROTECT].callback(
                s_timer_cb[BSP_TIME_TIMER_PROTECT].ctx);
            BSP_PROF_ISR_END(BSP_PROF_ISR_PROTECT, prof_start);
        }
        return;
    }
    if (htim->Instance == TIM7) {
        if (s_timer_cb[BSP_TIME_TIMER_CW_KEY].callback != 0) {
            BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_CW_KEY, prof_start);
            s_timer_cb[BSP_TIME_TIMER_CW_KEY].callback(
                s_timer_cb[BSP_TIME_TIMER_CW_KEY].ctx);
            BSP_PROF_ISR_END(BSP_PROF_ISR_CW_KEY, prof_start);
        }
    }
}

void Bsp_Time_Init(void) {
    /* Enable the trace unit and the free-running cycle counter. The counter is
     * clocked by the Cortex-M7 CPU clock (SYSCLK = 480 MHz), not HCLK. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55u; /* unlock the DWT (Cortex-M7 lock access) */
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Bsp_Time_GetCpuHz(void) {
    return HAL_RCC_GetSysClockFreq();
}

uint32_t Bsp_Time_GetMs(void) {
    return HAL_GetTick();
}

void Bsp_Time_DelayMs(uint32_t ms) {
    HAL_Delay(ms);
}

int Bsp_Time_StartTimer(BspTimeTimer timer, BspTimeTimerCallback callback,
                        void *ctx) {
    TIM_HandleTypeDef *htim;

    if (callback == 0) {
        return -1;
    }

    switch (timer) {
    case BSP_TIME_TIMER_PROTECT:
        htim = &htim6;
        break;
    case BSP_TIME_TIMER_CW_KEY:
        htim = &htim7;
        break;
    default:
        return -1;
    }

    s_timer_cb[timer].callback = callback;
    s_timer_cb[timer].ctx      = ctx;
    if (HAL_TIM_Base_Start_IT(htim) != HAL_OK) {
        s_timer_cb[timer].callback = 0;
        s_timer_cb[timer].ctx      = 0;
        return -1;
    }
    return 0;
}
