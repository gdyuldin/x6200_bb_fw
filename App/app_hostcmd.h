#pragma once

#include <stddef.h>
#include <stdint.h>

int App_HostCmd_Init(void);

/*
 * Host calibration commands. Command 0x8003 dumps and 0x8005 saves the
 * calibration block; the calibration RAM shadow windows are 0x0100..0x02FA
 * (read) and 0xf000..0xF1FA (write).
 *
 * CalibSave() persists the current calibration block to the EEPROM.
 * CalibDump() and the window accessors are declarations only: their bodies
 * depend on the I2C3 host protocol, which is out of scope here.
 */
int App_HostCmd_CalibSave(void);
int App_HostCmd_CalibDump(void);
int App_HostCmd_CalibWindowRead(uint16_t addr, uint8_t *dst, size_t len);
int App_HostCmd_CalibWindowWrite(uint16_t addr, const uint8_t *src, size_t len);
