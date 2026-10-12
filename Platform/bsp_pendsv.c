#include "bsp_pendsv.h"

#include "stm32h7xx_hal.h"

static BspPendSvHandlerFn s_pendsv_fn;

void Bsp_PendSV_Register(BspPendSvHandlerFn fn) {
    s_pendsv_fn = fn;
}

void Bsp_PendSV_Unregister(void) {
    s_pendsv_fn = 0;
}

void Bsp_PendSV_Trigger(void) {
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB();
    __ISB();
}

/*
 * Used for the FPGA FFT-window notification so the heavy FFT work runs outside
 * the 48 kHz ISR. The startup file declares a weak PendSV_Handler; this
 * definition overrides it. If CubeMX regenerates the .ioc and restores the
 * handler in Core/, the duplicate strong symbol must be resolved by removing it
 * from Core/.
 */
void PendSV_Handler(void) {
    if (s_pendsv_fn != 0) {
        s_pendsv_fn();
    }
}
