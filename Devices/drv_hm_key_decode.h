#pragma once

#include <stdint.h>

/*
 * Hand-mic key frame decoder. Pure state machine over pulse-cell measurements,
 * with no hardware or timing dependencies, so it can be exercised on the host.
 *
 * The mic sends one burst per press on the shared key line:
 *   seven short cells, one long start cell, 39 data bits, one long stop cell.
 * Bits 0..19 and 20..38 carry the same 20-bit word (the second copy has no top
 * bit), and the buttons are encoded as a matrix row/column nibble pair.
 */

void    Drv_HmKeyDecode_Reset(void);

/* Feed one cell: full period and high phase in timer ticks, current time in ms.
 * Returns the latest decoded code (unchanged while a frame is incomplete). */
uint8_t Drv_HmKeyDecode_Edge(uint32_t period, uint32_t high, uint32_t now_ms);

/* Latest decoded code, or 0 once the code lifetime has expired. */
uint8_t Drv_HmKeyDecode_Get(uint32_t now_ms);