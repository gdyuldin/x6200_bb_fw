#pragma once

/*
 * Host protocol semantics on top of the I2C3 slave transport
 * (Platform/bsp_i2c_slave). Owns the register map: base-info, the command
 * block shadow and the host command codes. The transport callbacks run in the
 * I2C3 ISR, so this module only copies and latches flags there; blocking work
 * (calib save, power-off, IAP) runs in hostlink_task on the main loop.
 *
 * Register map:
 *   write 0x0000..0x00D7  command block, auto-increment, word at cmd*4
 *   write 0xF000..0xF1FA  calib RAM copy, wrap mod 0x1FB
 *   write 0xFFFE          host command, 16-bit
 *   read  0x0000..0x0080  base-info (129 B)
 *   read  0x0100..0x02FA  calib RAM copy (507 B)
 *   read  0x2000          ready = 1
 *   read  others          one zero byte
 */

int App_HostLink_Init(void);
