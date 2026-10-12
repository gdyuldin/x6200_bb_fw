#pragma once

#include <stdint.h>

#include "stm32h7xx_hal.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} BspPin;

void          Bsp_Gpio_Write(BspPin pin, GPIO_PinState state);
GPIO_PinState Bsp_Gpio_Read(BspPin pin);
void          Bsp_Gpio_Toggle(BspPin pin);
