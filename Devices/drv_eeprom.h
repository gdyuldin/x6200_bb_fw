#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * AT24C256C external I2C EEPROM driver (I2C1 master bus).
 *
 *   - 32 KiB, 7-bit address 0x50, 16-bit (2-byte) memory address MSB->LSB,
 *     64-byte write page, ~10 ms write cycle.
 *
 * Drv_Eeprom_Read()  : one address-then-read transaction; the device keeps its
 *                      internal address pointer across the STOP, so the BSP's
 *                      transmit+receive pair reads from the requested offset.
 * Drv_Eeprom_Write() : splits the range on 64-byte page boundaries, waits the
 *                      write cycle and kicks the IWDG after every page.
 *
 * Return codes: 0 success, -1 argument/range/NULL, negative BSP transport
 * code (-2/-3) on a failed transfer. Call only from housekeeping context
 * (never from an ISR) — the write path blocks on the write-cycle delay.
 */

#define DRV_EEPROM_ADDR7         0x50u   /* 7-bit device address */
#define DRV_EEPROM_SIZE          32768u  /* 32 KiB */
#define DRV_EEPROM_PAGE_SIZE     64u     /* write page */
#define DRV_EEPROM_MEM_ADDR_SIZE 2u      /* 16-bit memory address */
#define DRV_EEPROM_WRITE_CYCLE_MS 10u    /* tWR between page writes */

#define DRV_EEPROM_TIMEOUT_BASE_MS     5u
#define DRV_EEPROM_TIMEOUT_PER_BYTE_MS 1u

int Drv_Eeprom_Read(uint16_t offset, uint8_t *buf, size_t len);
int Drv_Eeprom_Write(uint16_t offset, const uint8_t *buf, size_t len);
