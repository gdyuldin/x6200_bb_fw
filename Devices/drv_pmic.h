#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * AXP223 PMIC driver (I2C4).
 *
 * Drv_Pmic_Init() runs the ordered rail bring-up: it drops the outputs,
 * programs the rail voltages and then enables the rails the application
 * uses (DCDC1/2/3/5, ALDO1, DLDO1). It returns 0 on success or the negative
 * error code of the first failed register write; it does not abort the boot.
 *
 * Drv_Pmic_DropOutputs() leaves only the base CPU rail enabled. Register it
 * with Bsp_Boot_SetPreHandoffHook() so the outputs are dropped before the
 * bootloader reset flags are cleared. It is best-effort and returns no
 * status (the system is leaving).
 */
int  Drv_Pmic_Init(void);
void Drv_Pmic_DropOutputs(void);
