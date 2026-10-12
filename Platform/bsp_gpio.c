#include "bsp_gpio.h"

void Bsp_Gpio_Write(BspPin pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(pin.port, pin.pin, state);
}

GPIO_PinState Bsp_Gpio_Read(BspPin pin) {
    return HAL_GPIO_ReadPin(pin.port, pin.pin);
}

void Bsp_Gpio_Toggle(BspPin pin) {
    HAL_GPIO_TogglePin(pin.port, pin.pin);
}
