#pragma once

/*
 * Software-IRQ (PendSV) hook.
 *
 * A device ISR raises PendSV and the registered callback runs in the
 * software-IRQ context, so deferred work stays out of the hard IRQ. One
 * handler slot: the last registration wins. API for Devices/App callers.
 */

typedef void (*BspPendSvHandlerFn)(void);

void Bsp_PendSV_Register(BspPendSvHandlerFn fn);
void Bsp_PendSV_Unregister(void);
void Bsp_PendSV_Trigger(void);
