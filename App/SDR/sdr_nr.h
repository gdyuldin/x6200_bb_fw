#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Spectral noise reduction for the common RX audio chain.
 *
 * Sdr_NR_Process() runs in the I2S DMA ISR: it only pushes input samples and
 * pulls output samples through the ring buffers. The heavy FFT work runs in the
 * separate task, triggered  once a full NR window of input samples is ready.
 * Disabled by default.
 */
void Sdr_NR_Init(void);
void Sdr_NR_SetThreshold(float nrthr);
void Sdr_NR_SetEnabled(bool enabled);
void Sdr_NR_Process(float *samples, uint32_t n);
