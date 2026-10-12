#pragma once

#include <stdint.h>

#include "stm32h7xx_hal.h"

/*
 * EXTI dispatch for external interrupt lines (FPGA host bus, PTT, CW paddles).
 *
 * The generated EXTI handlers call HAL_GPIO_EXTI_Callback(), which is a weak
 * HAL symbol. This module owns it and routes the callback to the handler
 * registered for the pin, so a device driver can service its line without
 * touching Core/. Unregistered lines are ignored.
 *
 * Bsp_Exti_Register() is Platform API; the callers are in Devices/App, so no
 * Platform -> Devices dependency is introduced.
 */

typedef void (*BspExtiHandlerFn)(void *ctx);

int  Bsp_Exti_Register(uint16_t pin, BspExtiHandlerFn fn, void *ctx);
int  Bsp_Exti_Unregister(uint16_t pin);
