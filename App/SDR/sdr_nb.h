#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdr_common.h"

/*
 * Impulse noise blanker on the raw RX I/Q, before the channel decimation.
 *
 * Sdr_NB_Process() runs in the I2S DMA ISR and is allocation-, log- and
 * divide-free: it only reads the parameters derived by Sdr_NB_SetParams().
 * Disabled by default; width and threshold follow the host settings.
 */
void Sdr_NB_Init(void);

/* nbw, nbthr and nbe arrive in one host command word, so all three are set at
 * once. Called from the main loop. */
void Sdr_NB_SetParams(bool enabled, uint8_t width, uint8_t threshold);

void Sdr_NB_Process(const Sdr_IQ_Data *iq);
