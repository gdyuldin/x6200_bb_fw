#include "bsp_boot.h"

#include "bsp_time.h"
#include "bsp_uart.h"

#include "stm32h7xx_hal.h"

/* Bootloader vector table base. */
#define BSP_BOOT_BASE  0x08000000UL
#define BSP_BOOT_MSP   (BSP_BOOT_BASE + 0x00UL)
#define BSP_BOOT_RESET (BSP_BOOT_BASE + 0x04UL)

#define BSP_BOOT_LOG_DRAIN_MS     100u
#define BSP_BOOT_HANDOFF_DELAY_MS 1000u

/*
 * Hand-off breadcrumbs "iap[N]" on the DEBUG UART (polling TX). Off by
 * default; set to 1 here or via -DBSP_BOOT_IAP_TRACE=1 to localise a hang
 * during IAP entry. When off, DEBUG is not restored after the RSTR pulse -
 * the bootloader reconfigures USART1 itself.
 */
#ifndef BSP_BOOT_IAP_TRACE
#define BSP_BOOT_IAP_TRACE 0u
#endif

#if BSP_BOOT_IAP_TRACE
static void boot_trace(uint32_t step) {
    char msg[] = "iap[0]\r\n";
    msg[4] = (char)('0' + step);
    (void)Bsp_Uart_Write(BSP_UART_DEBUG, (const uint8_t *)msg, sizeof msg - 1u);
}
#else
#define boot_trace(step) ((void)0)
#endif

/* Reset-mask bits unnamed in stm32h725xx.h. */
#define BSP_BOOT_AHB3RSTR_RESERVED (0x3u << 22) /* bits 22..23 */
#define BSP_BOOT_AHB2RSTR_RESERVED (0x3u << 4)  /* bits 4..5 (CRYP/HASH pos.) */

/*
 * Peripheral reset pulse masks, composed from the CMSIS bit names. AHB4RSTR is
 * deliberately absent: it holds the GPIO banks, and
 * resetting them would cut board power/logic.
 */
#define BSP_BOOT_AHB3RSTR_MASK                                            \
    (RCC_AHB3RSTR_MDMARST | RCC_AHB3RSTR_DMA2DRST | RCC_AHB3RSTR_FMCRST | \
     RCC_AHB3RSTR_OSPI1RST | RCC_AHB3RSTR_SDMMC1RST |                     \
     RCC_AHB3RSTR_OSPI2RST | RCC_AHB3RSTR_IOMNGRRST |                     \
     BSP_BOOT_AHB3RSTR_RESERVED)

#define BSP_BOOT_AHB1RSTR_MASK                                              \
    (RCC_AHB1RSTR_DMA1RST | RCC_AHB1RSTR_DMA2RST | RCC_AHB1RSTR_ADC12RST | \
     RCC_AHB1RSTR_ETH1MACRST | RCC_AHB1RSTR_USB1OTGHSRST)

#define BSP_BOOT_AHB2RSTR_MASK                                              \
    (RCC_AHB2RSTR_DCMI_PSSIRST | BSP_BOOT_AHB2RSTR_RESERVED |               \
     RCC_AHB2RSTR_RNGRST | RCC_AHB2RSTR_SDMMC2RST | RCC_AHB2RSTR_FMACRST | \
     RCC_AHB2RSTR_CORDICRST)

#define BSP_BOOT_APB3RSTR_MASK (RCC_APB3RSTR_LTDCRST)

#define BSP_BOOT_APB1LRSTR_MASK                                               \
    (RCC_APB1LRSTR_TIM2RST | RCC_APB1LRSTR_TIM3RST | RCC_APB1LRSTR_TIM4RST | \
     RCC_APB1LRSTR_TIM5RST | RCC_APB1LRSTR_TIM6RST | RCC_APB1LRSTR_TIM7RST | \
     RCC_APB1LRSTR_TIM12RST | RCC_APB1LRSTR_TIM13RST |                       \
     RCC_APB1LRSTR_TIM14RST | RCC_APB1LRSTR_LPTIM1RST |                      \
     RCC_APB1LRSTR_SPI2RST | RCC_APB1LRSTR_SPI3RST |                         \
     RCC_APB1LRSTR_SPDIFRXRST | RCC_APB1LRSTR_USART2RST |                    \
     RCC_APB1LRSTR_USART3RST | RCC_APB1LRSTR_UART4RST |                      \
     RCC_APB1LRSTR_UART5RST | RCC_APB1LRSTR_I2C1RST |                        \
     RCC_APB1LRSTR_I2C2RST | RCC_APB1LRSTR_I2C3RST | RCC_APB1LRSTR_I2C5RST | \
     RCC_APB1LRSTR_CECRST | RCC_APB1LRSTR_DAC12RST |                         \
     RCC_APB1LRSTR_UART7RST | RCC_APB1LRSTR_UART8RST)

#define BSP_BOOT_APB1HRSTR_MASK                                                \
    (RCC_APB1HRSTR_CRSRST | RCC_APB1HRSTR_SWPMIRST | RCC_APB1HRSTR_OPAMPRST | \
     RCC_APB1HRSTR_MDIOSRST | RCC_APB1HRSTR_FDCANRST |                        \
     RCC_APB1HRSTR_TIM23RST | RCC_APB1HRSTR_TIM24RST)

#define BSP_BOOT_APB2RSTR_MASK                                               \
    (RCC_APB2RSTR_TIM1RST | RCC_APB2RSTR_TIM8RST | RCC_APB2RSTR_USART1RST |  \
     RCC_APB2RSTR_USART6RST | RCC_APB2RSTR_UART9RST |                        \
     RCC_APB2RSTR_USART10RST | RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_SPI4RST | \
     RCC_APB2RSTR_TIM15RST | RCC_APB2RSTR_TIM16RST |                         \
     RCC_APB2RSTR_TIM17RST | RCC_APB2RSTR_SPI5RST | RCC_APB2RSTR_SAI1RST |   \
     RCC_APB2RSTR_DFSDM1RST)

#define BSP_BOOT_APB4RSTR_MASK                                                 \
    (RCC_APB4RSTR_SYSCFGRST | RCC_APB4RSTR_LPUART1RST | RCC_APB4RSTR_SPI6RST | \
     RCC_APB4RSTR_I2C4RST | RCC_APB4RSTR_LPTIM2RST |                           \
     RCC_APB4RSTR_LPTIM3RST | RCC_APB4RSTR_LPTIM4RST |                         \
     RCC_APB4RSTR_LPTIM5RST | RCC_APB4RSTR_COMP12RST |                         \
     RCC_APB4RSTR_VREFRST | RCC_APB4RSTR_SAI4RST | RCC_APB4RSTR_DTSRST)

/* DMA1 + DMA2 (AHB1), MDMA (AHB3). */
#define BSP_BOOT_DMA_CLK_MASK  (RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN)
#define BSP_BOOT_MDMA_CLK_MASK (RCC_AHB3ENR_MDMAEN)

#define BSP_BOOT_NVIC_WORDS 8u

static void (*s_pre_handoff_hook)(void);

void Bsp_Boot_SetPreHandoffHook(void (*hook)(void)) {
    s_pre_handoff_hook = hook;
}

void Bsp_Boot_EnterIap(void) {
    /* Drain the asynchronous DEBUG log while IRQs are still on. */
    Bsp_Time_DelayMs(BSP_BOOT_LOG_DRAIN_MS);

    /*
     * Drop the PMIC outputs through the application-registered hook, if any,
     * before the reset flags are cleared: a rail glitch could re-latch
     * CPURSTF and the bootloader would fall into "Wait Power KEY..." instead
     * of IAP. The parallel I2C host-command path (0x800A, register 0xfffe)
     * must call Bsp_Boot_EnterIap() as well. Runs unconditionally, always
     * before the RMVF write below.
     */
    if (s_pre_handoff_hook != 0) {
        s_pre_handoff_hook();
    }
    boot_trace(1);

    /* AUX (USART2) is the request side and comes down first; DEBUG (USART1)
     * is left running. */
    Bsp_Uart_Shutdown(BSP_UART_AUX);
    boot_trace(2);

    /* Gate the DMA clocks so the set/way cache maintenance below cannot hang
     * the bus behind a live DMA transfer. */
    RCC->AHB1ENR &= ~BSP_BOOT_DMA_CLK_MASK;
    RCC->AHB3ENR &= ~BSP_BOOT_MDMA_CLK_MASK;
    boot_trace(3);

    /* Hand-off settle pause. SysTick-based: must complete before __disable_irq. */
    Bsp_Time_DelayMs(BSP_BOOT_HANDOFF_DELAY_MS);

    SCB_DisableDCache();
    SCB_DisableICache();
    boot_trace(4);

    /* Reset-like CPU state. The FPU is left alone. */
    HAL_MPU_Disable();
    SysTick->CTRL = 0u;
    SysTick->LOAD = 0u;
    SysTick->VAL  = 0u;
    boot_trace(5);

    __disable_irq();
    boot_trace(6);

    /* Peripheral reset pulse (mask, then 0). No clock tree is reproduced
     * here - the bootloader reconfigures clocks and peripherals itself. */
    RCC->AHB3RSTR  = BSP_BOOT_AHB3RSTR_MASK;
    RCC->AHB3RSTR  = 0u;
    RCC->AHB1RSTR  = BSP_BOOT_AHB1RSTR_MASK;
    RCC->AHB1RSTR  = 0u;
    RCC->AHB2RSTR  = BSP_BOOT_AHB2RSTR_MASK;
    RCC->AHB2RSTR  = 0u;
    RCC->APB3RSTR  = BSP_BOOT_APB3RSTR_MASK;
    RCC->APB3RSTR  = 0u;
    RCC->APB1LRSTR = BSP_BOOT_APB1LRSTR_MASK;
    RCC->APB1LRSTR = 0u;
    RCC->APB1HRSTR = BSP_BOOT_APB1HRSTR_MASK;
    RCC->APB1HRSTR = 0u;
    RCC->APB2RSTR  = BSP_BOOT_APB2RSTR_MASK;
    RCC->APB2RSTR  = 0u;
    RCC->APB4RSTR  = BSP_BOOT_APB4RSTR_MASK;
    RCC->APB4RSTR  = 0u;

    /* The APB2 pulse includes USART1RST, so DEBUG is back in its reset
     * state; restore it from the handle config for the trailing breadcrumbs
     * only. GPIO survived the pulse (AHB4RSTR untouched). */
#if BSP_BOOT_IAP_TRACE
    (void)Bsp_Uart_Reinit(BSP_UART_DEBUG);
#endif
    boot_trace(7);

    /* Clear the reset flags unconditionally, after the RSTR pulse and after
     * the PMIC step, as the last flag action before the vector table switch. */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    for (uint32_t i = 0u; i < BSP_BOOT_NVIC_WORDS; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }
    boot_trace(8);

    /* Bootloader vector table, initial stack pointer, privileged MSP-based
     * thread mode. */
    SCB->VTOR = BSP_BOOT_BASE;
    __set_MSP(*(volatile uint32_t *)BSP_BOOT_MSP);
    __set_CONTROL(0);
    __ISB();

    /*
     * Keep interrupts ON across the jump. The bootloader keeps time from its
     * SysTick handler only, so a masked PRIMASK would spin its first busy-wait
     * delay forever, watchdog kick would never run, and IWDG1 would reset the
     * MCU before the bootloader started.
     */
    __enable_irq();

    boot_trace(9);

    /* Bootloader Reset_Handler: its own .data/.bss init, SystemInit, main. */
    ((void (*)(void)) * (volatile uint32_t *)BSP_BOOT_RESET)();

    __builtin_unreachable();
}
