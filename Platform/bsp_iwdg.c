#include "bsp_iwdg.h"

#include "stm32h7xx_hal.h"

/* RM0468 §50.4.1: 0xAAAA reloads the counter down from RLR. */
#define IWDG_RELOAD_KEY 0xAAAAu

void Bsp_Iwdg_Kick(void) {
    IWDG1->KR = IWDG_RELOAD_KEY;
    __DSB();
}
