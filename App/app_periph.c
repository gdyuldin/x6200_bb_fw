#include "app_periph.h"

#include "main.h"

#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_gpio.h"
#include "bsp_log.h"
#include "drv_adc_mon.h"
#include "drv_pca9535.h"

#define LOG_TAG "app.periph"

/* Strapping pins, active-low with pull-up: a low pin sets its bit. Order is
 * PF6->bit0, PF7->bit1, PF8->bit2. */
static BspPin const s_detect[3] = {
    { BOARD_DETECT_0_GPIO_Port, BOARD_DETECT_0_Pin },
    { BOARD_DETECT_1_GPIO_Port, BOARD_DETECT_1_Pin },
    { BOARD_DETECT_2_GPIO_Port, BOARD_DETECT_2_Pin },
};

static uint32_t s_button_mask;
static uint8_t  s_hw_rev;

static void periph_poll(void) {
    Drv_Pca9535_ReadButtons(&s_button_mask);
}

static uint8_t periph_read_hw_rev(void) {
    uint8_t n = 0;
    for (unsigned i = 0; i < 3; i++) {
        if (Bsp_Gpio_Read(s_detect[i]) == GPIO_PIN_RESET) {
            n |= (uint8_t)(1u << i);
        }
    }
    return (uint8_t)(n & 0x7u);
}

int App_Periph_Init(void) {
    s_hw_rev = periph_read_hw_rev();
    LOG_INFO("Hardware version: %u", s_hw_rev);
    if (Drv_Pca9535_Init() != 0) {
        return -1;
    }
    if (Drv_AdcMon_Init() != 0) {
        return -1;
    }
    return App_Scheduler_Add(periph_poll, BOARD_BUTTONS_PERIOD_MS);
}

uint32_t App_Periph_GetButtonMask(void) {
    return s_button_mask;
}

uint8_t App_Periph_HwRev(void) {
    return s_hw_rev;
}
