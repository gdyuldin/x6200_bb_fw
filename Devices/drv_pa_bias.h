#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * PA bias DAC driver (DAC1 channels 1/2 on PA4/PA5).
 *
 * Behaviour:
 *   - bias_drive (calib 0x119) -> DAC1 ch1, bias_final (calib 0x11b) -> ch2;
 *   - both channels are 12-bit right-aligned, output buffer on, normal mode;
 *   - on boot the calibration values are written and the channels enabled;
 *   - a host update rewrites both channels immediately;
 *   - dw_sple_atue_trx bits 21/22 force the matching channel to 0 (OFF).
 *
 * Revision policy for this firmware: bias voltage may only be set when the
 * board revision is 0. On every other revision the driver refuses all DAC
 * writes and logs the refusal; the calibration RAM copy is still owned by the
 * caller and may be updated, but the DAC stays untouched.
 *
 * The driver never reads the ADC and does not implement ALC: bias_drive is
 * open-loop here. bias_final is never touched by ALC either.
 *
 * bias_drive/bias_final passed to Init/SetBias are the base calibration values
 * owned by NVRAM/host. Either DAC channel receives base + temperature offset,
 * where the offset is a signed DAC-code correction supplied by the application
 * and clamped so bias never rises above the base. The offsets are not persisted
 * and start at 0 after reset.
 */

int  Drv_PaBias_Init(uint8_t hw_rev, uint16_t bias_drive, uint16_t bias_final);
int  Drv_PaBias_SetBias(uint16_t bias_drive, uint16_t bias_final);
int  Drv_PaBias_SetOff(bool drive_off, bool final_off);
int  Drv_PaBias_SetOffset(int16_t drive_offset, int16_t final_offset);
bool Drv_PaBias_IsEnabled(void);
