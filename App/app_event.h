#pragma once

#include <stdint.h>

#include "app_radio.h"
#include "sdr_common.h"
#include "sdr_config.h"

/*
 * Central dispatcher for state transitions (RX/TX) and configuration blocks
 * (RX config, TX config, CW key tone). Publishers push a new state; subscribers
 * compare it against their own previous value and decide whether to reset. The
 * dispatcher does not interpret the state and does not touch hardware or
 * app_radio_state.
 *
 * Publish is ISR-safe: the event lands in a per-type ring and the scheduler is
 * notified. Callbacks run in the main loop from App_Event_Process(), never in
 * an ISR. Subscriptions are registered once at init and live forever; subscribe
 * during dispatch is rejected.
 */

#define APP_EVENT_MAX_SUBSCRIBERS 16u
#define APP_EVENT_QUEUE_DEPTH     4u

typedef struct {
    AppRadioMode mode;
} AppEventRxTxEvent;

typedef void (*AppEventRxTxFn)(const AppEventRxTxEvent *evt, void *user);
typedef void (*AppEventRxCfgFn)(const SdrRxCfg *cfg, void *user);
typedef void (*AppEventTxCfgFn)(const SdrTxCfg *cfg, void *user);
typedef void (*AppEventKeyToneFn)(const SdrKeyToneCfg *cfg, void *user);

int  App_Event_Init(void);
int  App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user);
int  App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user);
int  App_Event_SubscribeTxCfg(AppEventTxCfgFn cb, void *user);
int  App_Event_SubscribeKeyTone(AppEventKeyToneFn cb, void *user);
int  App_Event_NotifyRxTx(AppRadioMode mode);
int  App_Event_NotifyRxCfg(const SdrRxCfg *cfg);
int  App_Event_NotifyTxCfg(const SdrTxCfg *cfg);
int  App_Event_NotifyKeyTone(const SdrKeyToneCfg *cfg);
void App_Event_Process(void);
