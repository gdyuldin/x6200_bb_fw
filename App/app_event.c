#include "app_event.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "app_scheduler.h"
#include "bsp_irq.h"
#include "bsp_log.h"

#define LOG_TAG "app.event"

typedef union {
    AppEventRxTxEvent rxtx;
    SdrRxCfg          rx_cfg;
    SdrTxCfg          tx_cfg;
    SdrKeyToneCfg     key_tone;
} AppEventPayload;

typedef struct {
    AppEventPayload buf[APP_EVENT_QUEUE_DEPTH];
    uint8_t         head;
    uint8_t         tail;
    uint8_t         count;
    uint32_t        dropped;
} AppEventQueue;

static AppEventQueue s_rxtx_q;
static AppEventQueue s_rx_cfg_q;
static AppEventQueue s_tx_cfg_q;
static AppEventQueue s_key_tone_q;

static AppEventRxTxFn       s_rxtx_cb[APP_EVENT_MAX_SUBSCRIBERS];
static void                *s_rxtx_user[APP_EVENT_MAX_SUBSCRIBERS];
static size_t               s_rxtx_count;

static AppEventRxCfgFn      s_rx_cfg_cb[APP_EVENT_MAX_SUBSCRIBERS];
static void                *s_rx_cfg_user[APP_EVENT_MAX_SUBSCRIBERS];
static size_t               s_rx_cfg_count;

static AppEventTxCfgFn      s_tx_cfg_cb[APP_EVENT_MAX_SUBSCRIBERS];
static void                *s_tx_cfg_user[APP_EVENT_MAX_SUBSCRIBERS];
static size_t               s_tx_cfg_count;

static AppEventKeyToneFn    s_key_tone_cb[APP_EVENT_MAX_SUBSCRIBERS];
static void                *s_key_tone_user[APP_EVENT_MAX_SUBSCRIBERS];
static size_t               s_key_tone_count;

static bool s_dispatching;

static int event_push(AppEventQueue *q, const AppEventPayload *payload) {
    uint32_t key = Bsp_Irq_SaveLow();

    if (q->count == APP_EVENT_QUEUE_DEPTH) {
        /* State events carry the newest value only; evict the oldest. */
        q->tail = (uint8_t)((q->tail + 1u) % APP_EVENT_QUEUE_DEPTH);
        q->count--;
        q->dropped++;
    }
    q->buf[q->head] = *payload;
    q->head = (uint8_t)((q->head + 1u) % APP_EVENT_QUEUE_DEPTH);
    q->count++;
    Bsp_Irq_RestoreLow(key);

    App_Scheduler_Notify(APP_SCHED_EV_APP_EVENT);
    return 0;
}

/* Copy the oldest event out under the IRQ mask so a concurrent ISR publish
 * cannot overwrite the slot while a callback reads it. */
static bool event_pop(AppEventQueue *q, AppEventPayload *out) {
    uint32_t key = Bsp_Irq_SaveLow();

    if (q->count == 0u) {
        Bsp_Irq_RestoreLow(key);
        return false;
    }
    *out = q->buf[q->tail];
    q->tail = (uint8_t)((q->tail + 1u) % APP_EVENT_QUEUE_DEPTH);
    q->count--;
    Bsp_Irq_RestoreLow(key);
    return true;
}

static uint32_t event_take_dropped(void) {
    uint32_t key = Bsp_Irq_SaveLow();
    uint32_t total = s_rxtx_q.dropped + s_rx_cfg_q.dropped +
                     s_tx_cfg_q.dropped + s_key_tone_q.dropped;

    s_rxtx_q.dropped     = 0u;
    s_rx_cfg_q.dropped   = 0u;
    s_tx_cfg_q.dropped   = 0u;
    s_key_tone_q.dropped = 0u;
    Bsp_Irq_RestoreLow(key);
    return total;
}

int App_Event_Init(void) {
    memset(&s_rxtx_q, 0, sizeof(s_rxtx_q));
    memset(&s_rx_cfg_q, 0, sizeof(s_rx_cfg_q));
    memset(&s_tx_cfg_q, 0, sizeof(s_tx_cfg_q));
    memset(&s_key_tone_q, 0, sizeof(s_key_tone_q));
    s_rxtx_count     = 0u;
    s_rx_cfg_count   = 0u;
    s_tx_cfg_count   = 0u;
    s_key_tone_count = 0u;
    s_dispatching    = false;

    return App_Scheduler_AddEvent(App_Event_Process, APP_SCHED_EV_APP_EVENT);
}

int App_Event_SubscribeRxTx(AppEventRxTxFn cb, void *user) {
    if (cb == 0 || s_dispatching ||
        s_rxtx_count >= APP_EVENT_MAX_SUBSCRIBERS) {
        return -1;
    }
    s_rxtx_cb[s_rxtx_count]   = cb;
    s_rxtx_user[s_rxtx_count] = user;
    s_rxtx_count++;
    return 0;
}

int App_Event_SubscribeRxCfg(AppEventRxCfgFn cb, void *user) {
    if (cb == 0 || s_dispatching ||
        s_rx_cfg_count >= APP_EVENT_MAX_SUBSCRIBERS) {
        return -1;
    }
    s_rx_cfg_cb[s_rx_cfg_count]   = cb;
    s_rx_cfg_user[s_rx_cfg_count] = user;
    s_rx_cfg_count++;
    return 0;
}

int App_Event_SubscribeTxCfg(AppEventTxCfgFn cb, void *user) {
    if (cb == 0 || s_dispatching ||
        s_tx_cfg_count >= APP_EVENT_MAX_SUBSCRIBERS) {
        return -1;
    }
    s_tx_cfg_cb[s_tx_cfg_count]   = cb;
    s_tx_cfg_user[s_tx_cfg_count] = user;
    s_tx_cfg_count++;
    return 0;
}

int App_Event_SubscribeKeyTone(AppEventKeyToneFn cb, void *user) {
    if (cb == 0 || s_dispatching ||
        s_key_tone_count >= APP_EVENT_MAX_SUBSCRIBERS) {
        return -1;
    }
    s_key_tone_cb[s_key_tone_count]   = cb;
    s_key_tone_user[s_key_tone_count] = user;
    s_key_tone_count++;
    return 0;
}

int App_Event_NotifyRxTx(AppRadioMode mode) {
    AppEventPayload payload;
    payload.rxtx.mode = mode;
    return event_push(&s_rxtx_q, &payload);
}

int App_Event_NotifyRxCfg(const SdrRxCfg *cfg) {
    AppEventPayload payload;
    payload.rx_cfg = *cfg;
    return event_push(&s_rx_cfg_q, &payload);
}

int App_Event_NotifyTxCfg(const SdrTxCfg *cfg) {
    AppEventPayload payload;
    payload.tx_cfg = *cfg;
    return event_push(&s_tx_cfg_q, &payload);
}

int App_Event_NotifyKeyTone(const SdrKeyToneCfg *cfg) {
    AppEventPayload payload;
    payload.key_tone = *cfg;
    return event_push(&s_key_tone_q, &payload);
}

static void dispatch_rx_cfg(void) {
    AppEventPayload payload;

    while (event_pop(&s_rx_cfg_q, &payload)) {
        for (size_t i = 0; i < s_rx_cfg_count; i++) {
            s_rx_cfg_cb[i](&payload.rx_cfg, s_rx_cfg_user[i]);
        }
    }
}

static void dispatch_tx_cfg(void) {
    AppEventPayload payload;

    while (event_pop(&s_tx_cfg_q, &payload)) {
        for (size_t i = 0; i < s_tx_cfg_count; i++) {
            s_tx_cfg_cb[i](&payload.tx_cfg, s_tx_cfg_user[i]);
        }
    }
}

static void dispatch_key_tone(void) {
    AppEventPayload payload;

    while (event_pop(&s_key_tone_q, &payload)) {
        for (size_t i = 0; i < s_key_tone_count; i++) {
            s_key_tone_cb[i](&payload.key_tone, s_key_tone_user[i]);
        }
    }
}

static void dispatch_rxtx(void) {
    AppEventPayload payload;

    while (event_pop(&s_rxtx_q, &payload)) {
        for (size_t i = 0; i < s_rxtx_count; i++) {
            s_rxtx_cb[i](&payload.rxtx, s_rxtx_user[i]);
        }
    }
}

void App_Event_Process(void) {
    uint32_t dropped;

    dropped = event_take_dropped();
    s_dispatching = true;

    /* Configuration before activation: block changes reach subscribers before
     * an RX/TX transition. */
    dispatch_key_tone();
    dispatch_tx_cfg();
    dispatch_rx_cfg();
    dispatch_rxtx();

    s_dispatching = false;

    if (dropped != 0u) {
        LOG_WARN("%u event(s) dropped", (unsigned)dropped);
    }
}
