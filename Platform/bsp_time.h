#pragma once

#include <stdint.h>

#include "stm32h7xx_hal.h"

/* Board timer roles; Core owns the HAL handles and runs MX_TIMx_Init(). */
typedef enum {
    BSP_TIME_TIMER_PROTECT = 0, /* TIM6: 10 ms housekeeping/protection tick */
    BSP_TIME_TIMER_CW_KEY,      /* TIM7: ~3 kHz CW paddle debounce tick */
    BSP_TIME_TIMER_COUNT
} BspTimeTimer;

/* Called from the timer update ISR. Keep it short and non-blocking. */
typedef void (*BspTimeTimerCallback)(void *ctx);

/* Register a handler and start a board timer in interrupt mode.
 * 0 = success, negative = error. */
int      Bsp_Time_StartTimer(BspTimeTimer timer, BspTimeTimerCallback callback,
                             void *ctx);

uint32_t Bsp_Time_GetMs(void);
void     Bsp_Time_DelayMs(uint32_t ms);

/*
 * Free-running HSE-referenced cycle counter (DWT CYCCNT). On STM32H7 the DWT
 * counts the Cortex-M7 CPU clock (SYSCLK = 480 MHz here, from the 25 MHz HSE
 * via PLL1), NOT HCLK (240 MHz) — using HCLK halves the measured rate. Used as
 * an independent time base for the AIC/FPGA clock-sync check.
 * Bsp_Time_Init() enables the trace unit; call it early, before any
 * Bsp_Time_GetCycles() consumer. The counter is 32-bit and wraps at 2^32 CPU
 * cycles (~8.9 s at 480 MHz): always take differences (now - prev) as uint32_t,
 * which is wrap-safe for a single wrap.
 */
void     Bsp_Time_Init(void);
uint32_t Bsp_Time_GetCpuHz(void);

__STATIC_FORCEINLINE uint32_t Bsp_Time_GetCycles(void) {
    return DWT->CYCCNT;
}
