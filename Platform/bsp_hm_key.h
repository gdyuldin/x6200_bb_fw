#pragma once

#include <stdint.h>

/*
 * Input capture for the external hand-mic key line (PD12 / TIM4_CH1). The
 * decoder is a device protocol and lives above Platform; this module only
 * measures one pulse-coded cell: on every rising edge it reports the full cell
 * period and the high phase, both in TIM4 ticks.
 */

typedef void (*BspHmKeyEdgeFn)(uint32_t period, uint32_t high, void *ctx);

/* Register the edge handler and start TIM4 capture. 0 = success, negative =
 * error. */
int Bsp_HmKey_Start(BspHmKeyEdgeFn fn, void *ctx);