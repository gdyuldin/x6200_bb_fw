#pragma once

/*
 * Hand off to the bootloader in IAP mode. Never returns.
 *
 * Single entry point for the hand-off; the future I2C host-command path
 * (0x800A, register 0xfffe) must call this too.
 */
void Bsp_Boot_EnterIap(void) __attribute__((noreturn));

/*
 * Register a callback invoked from Bsp_Boot_EnterIap() immediately before the
 * reset flags are cleared and control jumps to the bootloader. The application
 * uses it to drop the PMIC outputs, so that a rail glitch cannot re-latch
 * CPURSTF and the bootloader's IAP gate stays valid. Pass NULL to clear.
 */
void Bsp_Boot_SetPreHandoffHook(void (*hook)(void));
