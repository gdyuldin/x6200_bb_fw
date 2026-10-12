#include "bsp_init.h"

#include "bsp_gpio.h"
#include "bsp_prof.h"
#include "bsp_time.h"

void Bsp_Init(void) {
    Bsp_Time_Init();
    Bsp_Prof_Reset();
}
