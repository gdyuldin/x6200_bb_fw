#pragma once

/*
 * Placement helpers for large, non-DMA buffers. The linker script maps
 * .ram_d1_bss to RAM_D1 (AXI SRAM), keeping DTCM free for the stack and the
 * latency-critical DSP state. Buffers in this section are zeroed by the
 * startup code before main() runs.
 *
 * Do not use for DMA targets: DMA buffers belong in .dma_d2 (RAM_D2), which is
 * configured non-cacheable in the MPU.
 */
#define BSP_RAM_D1_BSS __attribute__((section(".ram_d1_bss")))
#define BSP_RAM_D1_BSS_32 __attribute__((section(".ram_d1_bss"), aligned(32)))
