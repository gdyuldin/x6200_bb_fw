#include "bsp_exti.h"

#include <stddef.h>

#include "bsp_log.h"
#include "bsp_prof.h"

#define LOG_TAG "bsp.exti"

#define BSP_EXTI_MAX_HANDLERS 8u

typedef struct {
    uint16_t         pin;
    BspExtiHandlerFn fn;
    void            *ctx;
} BspExtiSlot;

static BspExtiSlot          s_slots[BSP_EXTI_MAX_HANDLERS];

int Bsp_Exti_Register(uint16_t pin, BspExtiHandlerFn fn, void *ctx) {
    size_t free_slot = BSP_EXTI_MAX_HANDLERS;

    if (pin == 0u || fn == 0) {
        return -1;
    }
    for (size_t i = 0; i < BSP_EXTI_MAX_HANDLERS; ++i) {
        if (s_slots[i].pin == pin) {
            s_slots[i].fn  = fn;
            s_slots[i].ctx = ctx;
            return 0;
        }
        if (s_slots[i].pin == 0u && free_slot == BSP_EXTI_MAX_HANDLERS) {
            free_slot = i;
        }
    }
    if (free_slot == BSP_EXTI_MAX_HANDLERS) {
        LOG_WARN("exti handler table full, pin 0x%04X dropped", (unsigned)pin);
        return -1;
    }
    s_slots[free_slot].pin = pin;
    s_slots[free_slot].fn  = fn;
    s_slots[free_slot].ctx = ctx;
    return 0;
}

int Bsp_Exti_Unregister(uint16_t pin) {
    for (size_t i = 0; i < BSP_EXTI_MAX_HANDLERS; ++i) {
        if (s_slots[i].pin == pin) {
            s_slots[i].pin = 0u;
            s_slots[i].fn  = 0;
            s_slots[i].ctx = 0;
            return 0;
        }
    }
    return -1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_FPGA_TICK, prof_start);

    for (size_t i = 0; i < BSP_EXTI_MAX_HANDLERS; ++i) {
        if (s_slots[i].pin == GPIO_Pin) {
            s_slots[i].fn(s_slots[i].ctx);
            break;
        }
    }

    BSP_PROF_ISR_END(BSP_PROF_ISR_FPGA_TICK, prof_start);
}
