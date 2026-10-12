#pragma once

#include <stddef.h>
#include <stdint.h>

typedef void (*AppSchedulerTaskFn)(void);
typedef void (*AppSchedulerEventFn)(void);

/*
 * Events: one event, one handler. App_Scheduler_Notify() is safe from any
 * context (ISR included); handlers run in the main loop before periodic
 * tasks. Edge semantics: notifies before dispatch coalesce into one run,
 * there is no queue.
 */
typedef enum {
    APP_SCHED_EV_SPECTRUM_CALC = 0,
    APP_SCHED_EV_SPECTRUM_READY,
    APP_SCHED_EV_NR_CALC,
    APP_SCHED_EV_PTT,
    APP_SCHED_EV_APP_EVENT,
    APP_SCHED_EV_HOSTLINK,
    APP_SCHED_EV_HOSTLINK_EXT,
    APP_SCHED_EV_COUNT
} AppSchedulerEvent;

typedef struct {
    AppSchedulerTaskFn fn;
    uint32_t           period_ms;
    uint32_t           last_ms;
} AppSchedulerTask;

int  App_Scheduler_Add(AppSchedulerTaskFn fn, uint32_t period_ms);
int  App_Scheduler_AddEvent(AppSchedulerEventFn fn, AppSchedulerEvent ev);
void App_Scheduler_Notify(AppSchedulerEvent ev);
void App_Scheduler_Run(void);
