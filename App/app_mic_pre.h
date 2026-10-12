#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Hand-mic line conditioning, applied to the microphone before the TX
 * modulator mixes it with the host input. Runs in the codec block ISR and
 * works in place; no blocking calls.
 *
 * The peaking cuts remove the hand-mic line tones without the infinite depth
 * of a true notch.
 */
void App_MicPre_Init(void);

void App_MicPre_Reset(void);

void App_MicPre_SetEnabled(bool enabled);

void App_MicPre_Process(float *samples, uint32_t n);