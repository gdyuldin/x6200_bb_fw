#pragma once

#include <stdint.h>

#include "drv_fpga.h"

/*
 * FPGA host-bus application glue. App_Fpga_Init() runs after the clock is up
 * (the FPGA NCO needs 122.88 MHz) and after NVRAM is loaded, and starts the
 * driver plus the periodic read-status request. calib points to the per-unit
 * FPGA calibration bytes from the EEPROM calib region; it is copied into the
 * driver (see Drv_Fpga_Init). A driver failure is logged and the boot
 * continues.
 *
 * The Set* wrappers hand the value to the driver, which computes words and
 * arms a flag so the frame leaves on the next PE10 tick.
 */

int  App_Fpga_Init(const DrvFpgaCalib *calib);
void App_Fpga_SetRxFrequency(uint32_t freq_hz);
void App_Fpga_SetTxFrequency(uint32_t freq_hz);
void App_Fpga_TxMute(uint32_t ms);
void App_Fpga_SetSampleRate(uint8_t fftdec);
