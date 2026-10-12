#include "bsp_board.h"

#include "bsp_gpio.h"
#include "main.h"

/* The power key is wired active low with a pull-up. */
bool Bsp_Board_IsPowerButtonPressed(void) {
    BspPin const pin = {POWER_BUTTON_GPIO_Port, POWER_BUTTON_Pin};
    return Bsp_Gpio_Read(pin) == GPIO_PIN_RESET;
}

/* EXT MIC PTT is wired active low with a pull-up. */
bool Bsp_Board_IsExtMicPttPressed(void) {
    BspPin const pin = {EXT_MIC_PTT_GPIO_Port, EXT_MIC_PTT_Pin};
    return Bsp_Gpio_Read(pin) == GPIO_PIN_RESET;
}

/* CW_HOST is wired active low with a pull-up. */
bool Bsp_Board_IsCwHostKeyDown(void) {
    BspPin const pin = {CW_HOST_GPIO_Port, CW_HOST_Pin};
    return Bsp_Gpio_Read(pin) == GPIO_PIN_RESET;
}

/* EXT MIC DETECT is wired active low with a pull-up. */
bool Bsp_Board_IsExtMicConnected(void) {
    BspPin const pin = {EXT_MIC_DETECT_GPIO_Port, EXT_MIC_DETECT_Pin};
    return Bsp_Gpio_Read(pin) == GPIO_PIN_RESET;
}

/* LINUX_POWER and POWER_HOLD are active high. */
void Bsp_Board_SetLinuxPower(bool enabled) {
    Bsp_Gpio_Write((BspPin){LINUX_POWER_GPIO_Port, LINUX_POWER_Pin},
                   enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_Board_SetPowerHold(bool enabled) {
    Bsp_Gpio_Write((BspPin){POWER_HOLD_GPIO_Port, POWER_HOLD_Pin},
                   enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* The status LEDs are open-drain active low: 0 = on. */
void Bsp_Board_SetStatusLedGreen(bool on) {
    Bsp_Gpio_Write((BspPin){STATUS_LED_GREEN_GPIO_Port, STATUS_LED_GREEN_Pin},
                   on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Bsp_Board_SetStatusLedRed(bool on) {
    Bsp_Gpio_Write((BspPin){STATUS_LED_RED_GPIO_Port, STATUS_LED_RED_Pin},
                   on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Bsp_Board_SetAccTrx(bool on) {
    Bsp_Gpio_Write((BspPin){ACC_TRX_GPIO_Port, ACC_TRX_Pin},
                   on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_Board_SetPABias(bool on) {
    Bsp_Gpio_Write((BspPin){PA_BIAS_EN_GPIO_Port, PA_BIAS_EN_Pin},
                   on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_Board_ConnectRxADC(bool on) {
    Bsp_Gpio_Write((BspPin){RF_RX_EN_GPIO_Port, RF_RX_EN_Pin},
                   on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_Board_SetTxRelay(bool on) {
    Bsp_Gpio_Write((BspPin){RF_TX_RELAY_GPIO_Port, RF_TX_RELAY_Pin},
                   on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Bsp_Board_FinalPowerCut(void) {
    __disable_irq();
    Bsp_Board_SetLinuxPower(false);
    Bsp_Board_SetPowerHold(false);
    Bsp_Board_SetStatusLedGreen(false);
    Bsp_Board_SetStatusLedRed(false);
    NVIC_SystemReset();
    for (;;) {
    }
}
