#include "app_scheduler.h"

#include <stdbool.h>

#include "bsp_irq.h"
#include "bsp_prof.h"
#include "bsp_time.h"

#define APP_SCHEDULER_MAX_TASKS 16

static AppSchedulerTask    s_tasks[APP_SCHEDULER_MAX_TASKS];
static size_t              s_task_count;
static AppSchedulerEventFn s_event_fn[APP_SCHED_EV_COUNT];
static volatile uint32_t   s_event_pending;

int App_Scheduler_Add(AppSchedulerTaskFn fn, uint32_t period_ms) {
    if (s_task_count >= APP_SCHEDULER_MAX_TASKS || fn == 0) {
        return -1;
    }
    s_tasks[s_task_count].fn        = fn;
    s_tasks[s_task_count].period_ms = period_ms;
    s_tasks[s_task_count].last_ms   = Bsp_Time_GetMs();
    s_task_count++;
    return 0;
}

int App_Scheduler_AddEvent(AppSchedulerEventFn fn, AppSchedulerEvent ev) {
    if (fn == 0 || ev >= APP_SCHED_EV_COUNT || s_event_fn[ev] != 0) {
        return -1;
    }
    s_event_fn[ev] = fn;
    return 0;
}

void App_Scheduler_Notify(AppSchedulerEvent ev) {
    uint32_t key;

    if (ev >= APP_SCHED_EV_COUNT) {
        return;
    }
    key = Bsp_Irq_SaveLow();
    s_event_pending |= (1u << ev);
    Bsp_Irq_RestoreLow(key);
}

static bool scheduler_run_events(void) {
    uint32_t key = Bsp_Irq_SaveLow();
    uint32_t pending = s_event_pending;
    bool     ran = false;

    s_event_pending = 0u;
    Bsp_Irq_RestoreLow(key);

    for (size_t ev = 0; ev < APP_SCHED_EV_COUNT; ev++) {
        if ((pending & (1u << ev)) != 0u && s_event_fn[ev] != 0) {
            s_event_fn[ev]();
            ran = true;
        }
    }
    return ran;
}

void App_Scheduler_Run(void) {
    for (;;) {
        uint32_t now;
        bool     any_work;

#if BSP_PROF_ENABLE
        uint32_t prof_start = Bsp_Prof_LoopBegin();
#endif

        any_work = scheduler_run_events();

        now = Bsp_Time_GetMs();
        for (size_t i = 0; i < s_task_count; i++) {
            if ((now - s_tasks[i].last_ms) >= s_tasks[i].period_ms) {
                s_tasks[i].last_ms += s_tasks[i].period_ms;
                s_tasks[i].fn();
                any_work = true;
            }
        }

#if BSP_PROF_ENABLE
        Bsp_Prof_LoopEnd(prof_start, any_work);
#endif
    }
}
