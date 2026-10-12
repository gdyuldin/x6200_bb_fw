#pragma once

#include <stdint.h>

#include "stm32h7xx_hal.h"

static inline uint32_t Bsp_Irq_Save(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void Bsp_Irq_Restore(uint32_t primask) {
    __set_PRIMASK(primask);
}

/*
 * Lower-priority critical section. Masks every interrupt with preemption
 * priority >= 1 and leaves priority 0 running. The FPGA tick (PE10/EXTI15_10)
 * is priority 0; the ADC monitor DMA is priority 1 and is masked here. A PE10
 * edge is still serviced while this section is held. Use it for data that the
 * FPGA ISR does not touch; keep Bsp_Irq_Save() for state shared with the FPGA
 * ISR itself.
 *
 * HAL initialises NVIC_PRIORITYGROUP_4 and __NVIC_PRIO_BITS is 4, so the
 * BASEPRI encoding of priority level 1 is 1 << 4 = 0x10. Priority levels
 * numerically >= 0x10 are masked; level 0 (0x00) stays enabled.
 */
#define BSP_IRQ_LOW_BASEPRI (1u << (8u - __NVIC_PRIO_BITS))

static inline uint32_t Bsp_Irq_SaveLow(void) {
    uint32_t basepri = __get_BASEPRI();
    __set_BASEPRI(BSP_IRQ_LOW_BASEPRI);
    __DSB();
    __ISB();
    return basepri;
}

static inline void Bsp_Irq_RestoreLow(uint32_t basepri) {
    __set_BASEPRI(basepri);
    __DSB();
    __ISB();
}
