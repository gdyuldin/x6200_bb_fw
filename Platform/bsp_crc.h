#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * Hardware CRC-32/MPEG-2 on the STM32 CRC unit (this module is the only user).
 * The unit is configured in Core/Src/crc.c with the reset defaults: polynomial
 * 0x04C11DB7, init 0xFFFFFFFF, no input/output inversion and
 * CRC_INPUTDATA_FORMAT_WORDS. Feeding 32-bit words MSB-first with no final xor
 * matches the host algorithm, which also XORs whole 32-bit words and shifts 8
 * bits per step: within each
 * little-endian word the bytes are consumed as b3,b2,b1,b0. A byte-format unit
 * instead walks the raw byte stream and yields a different value, so the Word
 * format is required for host compatibility (548 B = 137 words, no tail).
 *
 * len is a byte count and must be a multiple of 4. The module owns no state
 * beyond the shared unit, so Bsp_Crc32_Calc() is not reentrant.
 */

int Bsp_Crc32_Calc(const void *data, size_t len, uint32_t *out);

/*
 * Check the CRC unit against known vectors to catch a configuration regression
 * (e.g. the .ioc InputDataFormat reverting to bytes):
 *   - 137 zero words        -> 0xF54D034F
 *   - words 0,1,...,136     -> 0x0A133F4F
 * The ramped vector is not symmetric under the per-word byte reversal that
 * distinguishes Word from Byte format, so it catches a format change; an
 * all-zero vector cannot.
 * Returns 0 on match, -1 on bad arguments, -2 on mismatch.
 */
int Bsp_Crc32_SelfTest(void);
