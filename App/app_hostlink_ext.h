#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Extended host-protocol page on top of the I2C3 slave transport, carrying the
 * command set the stock firmware does not support. Write-only: nothing is read
 * back from this page. Lives outside the OEM register map so the two do not
 * collide.
 *
 * Register map (write only):
 *   0x4000..0x4007  magic "R2RFE_FW", opens the page
 *   0x4008          layout version (32-bit)
 *   0x4100..0x41FF  command block, auto-increment, word at idx*4
 *
 * The command block is ignored until the magic matches; the magic write must
 * come first. The page is write-only, so the transport read path is untouched.
 */

int App_HostLinkExt_Init(void);

/*
 * Called from the I2C3 ISR write hook. Returns true when reg belongs to the
 * extended page and was captured; false leaves the address to the OEM map.
 */
bool App_HostLinkExt_Write(uint16_t reg, const uint8_t *data, size_t len);
