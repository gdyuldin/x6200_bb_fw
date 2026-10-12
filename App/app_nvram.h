#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drv_nvram.h"

/*
 * Application NVRAM boot integration (external AT24C256C EEPROM).
 *
 * App_Nvram_Init() reads the three regions (settings -> calib -> NVPARAM) after
 * the PMIC rails are up. A read error is logged but does not abort the boot; an
 * invalid NVPARAM leaves the charger enabled (chge = 1).
 */

int App_Nvram_Init(void);

/*
 * Charger-enable preference from the NVPARAM record. The boot read defaults to
 * enabled when the record is absent or invalid (magic/CRC/read error). The RAM
 * copy changes immediately; App_Nvram_ChargeSave() persists it at power-off.
 */
bool App_Nvram_ChargeEnabled(void);
void App_Nvram_SetChargeEnabled(bool enabled);

/*
 * Persist the current charger-enable preference to NVPARAM (magic + chge +
 * CRC-8). Blocking EEPROM write; called from the TIM6 power-off sequence.
 */
int App_Nvram_ChargeSave(void);

/*
 * Calibration RAM copy read at boot. Exposed so the host save path
 * (App_HostCmd_CalibSave) can hand the current block to the EEPROM writer.
 */
const DrvNvramCalib *App_Nvram_Calib(void);

/*
 * TX permission for a frequency from the current calibration RAM copy.
 * fullband_txe allows every frequency; otherwise the frequency must fall inside
 * one of the TX bands whose txe is set (both edges inclusive). The copy is read
 * on every call so host updates take effect without a reboot.
 */
bool App_Nvram_TxAllowed(uint32_t freq_hz);

/*
 * Write bytes into the calibration RAM copy at an intra-region offset
 * (0..DRV_NVRAM_CALIB_LEN-1). src/len must fit inside the region; the caller
 * applies the wrap for the host protocol's 0xF000 window. Returns 0 on success,
 * -1 on bad arguments. Safe to call from an ISR: the copy is done under a
 * short critical section so a concurrent EEPROM save cannot observe a torn
 * block. The change is volatile until App_HostCmd_CalibSave() persists it.
 */
int App_Nvram_CalibWrite(uint16_t off, const uint8_t *src, size_t len);

/*
 * Update the two PA bias fields in the calibration RAM copy without touching
 * the EEPROM. The host bias command (0xA4) uses this so the DAC driver can be
 * updated with the same values in one step. The change is volatile until
 * App_HostCmd_CalibSave() persists it.
 */
int App_Nvram_CalibSetBias(uint16_t bias_drive, uint16_t bias_final);

/*
 * FPGA-frontend per-unit calibration bytes inside the calib region. The type
 * and layout live in the FPGA driver (drv_fpga.h); this module owns only the
 * pointer into its calib copy. Values are per-unit and come from this board's
 * EEPROM — never hard-coded.
 */
const DrvFpgaCalib *App_Nvram_FpgaCalib(void);
