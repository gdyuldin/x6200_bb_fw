#pragma once

#include <stdint.h>

/*
 * Hand-mic key receiver. The mic sends one burst per press on the shared key
 * line as a preamble, a start cell and a 39-bit data frame; the decoded frame
 * is the key matrix code published to the host as the telemetry hkey field.
 * A code is held for a short window after the last edge and then reads as 0,
 * which the host treats as "no key".
 */

int     Drv_HmKey_Init(void);

/* Latest decoded key code, or 0 when no code arrived recently. */
uint8_t Drv_HmKey_GetCode(void);