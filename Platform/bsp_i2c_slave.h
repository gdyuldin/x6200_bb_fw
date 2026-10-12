#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * I2C3 slave transport for the host protocol (7-bit address 0x72).
 *
 * This module owns hi2c3, the listen/sequence HAL calls and the register
 * address latch. It carries no protocol semantics: the register map and the
 * meaning of every payload live in the caller (App/app_hostlink). The
 * callbacks run in the I2C3 ISR, so they must not block (no Bsp_Time_DelayMs,
 * no EEPROM access).
 *
 * Wire format:
 *   write [0x72+W][reg 2 B big-endian][data]
 *   read  [0x72+W][reg 2 B big-endian][Sr][0x72+R][data]
 */

/*
 * Called once per completed write transaction. reg is the big-endian register
 * address; data/len is the payload after the two address bytes. A write that
 * carried only the register address (the first phase of a read transaction)
 * does not call this hook.
 */
typedef void (*BspI2cSlaveWriteFn)(uint16_t reg, const uint8_t *data, size_t len);

/*
 * Called when the master addresses the slave for a read. The handler fills dst
 * (at most cap bytes) and returns the number of bytes to send. Called in the
 * ISR, so keep the copy short.
 */
typedef size_t (*BspI2cSlaveReadFn)(uint16_t reg, uint8_t *dst, size_t cap);

/*
 * Register the handlers and arm the listen. Returns 0 on success, negative on
 * failure. Both handlers must be non-NULL.
 */
int Bsp_I2cSlave_Init(BspI2cSlaveWriteFn on_write, BspI2cSlaveReadFn on_read);
