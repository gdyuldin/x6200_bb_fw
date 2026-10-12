#include "app_prof.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_console.h"
#include "bsp_log.h"
#include "bsp_prof.h"
#include "bsp_time.h"

#define LOG_TAG "app.prof"

#if BSP_PROF_ENABLE

static const char *const s_isr_names[BSP_PROF_ISR_COUNT] = {
    "fpga", "adc", "codec", "cwkey", "protect", "usart2", "usart1",
};

static const char *const s_section_names[BSP_PROF_SECTION_COUNT] = {
    "spectrum", "nr",
};

static void prof_slot_log(const char *name, const BspProfStats *st,
                          uint64_t wall, uint32_t cpu_hz) {
    uint32_t pct;
    uint64_t rate;
    uint64_t avg_us;
    uint64_t max_us;

    if (st->calls == 0u || wall == 0u) {
        return;
    }
    pct    = (uint32_t)((uint64_t)st->cycles * 10000u / wall);
    rate   = (uint64_t)st->calls * cpu_hz / wall;
    avg_us = (uint64_t)st->cycles * 1000000u / ((uint64_t)cpu_hz * st->calls);
    max_us = (uint64_t)st->max_cycles * 1000000u / cpu_hz;
    LOG_INFO("%s %u.%02u%% %u/s avg %uus max %uus", name,
             (unsigned)(pct / 100u), (unsigned)(pct % 100u),
             (unsigned)rate, (unsigned)avg_us, (unsigned)max_us);
}

static void prof_report(void) {
    uint64_t wall = Bsp_Prof_WallCycles();
    uint64_t idle = Bsp_Prof_IdleCycles();
    uint32_t cpu_hz;
    uint32_t busy_pct;
    uint32_t idle_pct;
    uint32_t isr_pct;

    if (wall == 0u) {
        return;
    }
    cpu_hz    = Bsp_Time_GetCpuHz();
    if (idle > wall) {
        idle = wall;
    }
    busy_pct  = (uint32_t)((wall - idle) * 10000u / wall);
    idle_pct  = (uint32_t)(idle * 10000u / wall);
    isr_pct   = (uint32_t)((uint64_t)Bsp_Prof_IsrTotalCycles() * 10000u / wall);

    LOG_INFO("cpu %u.%02u%% idle %u.%02u%% isr %u.%02u%%",
             (unsigned)(busy_pct / 100u), (unsigned)(busy_pct % 100u),
             (unsigned)(idle_pct / 100u), (unsigned)(idle_pct % 100u),
             (unsigned)(isr_pct / 100u), (unsigned)(isr_pct % 100u));

    for (size_t i = 0; i < BSP_PROF_ISR_COUNT; i++) {
        BspProfStats st;

        Bsp_Prof_IsrStats((BspProfIsr)i, &st);
        prof_slot_log(s_isr_names[i], &st, wall, cpu_hz);
    }
    for (size_t i = 0; i < BSP_PROF_SECTION_COUNT; i++) {
        BspProfStats st;

        Bsp_Prof_SectionStats((BspProfSection)i, &st);
        prof_slot_log(s_section_names[i], &st, wall, cpu_hz);
    }
}

static void prof_help(void) {
    LOG_INFO("usage: p print, r reset window, q exit");
}

static void prof_enter(void) {
    Bsp_Prof_Reset();
    prof_help();
}

static void prof_exit(void) {
}

static void prof_on_key(char c) {
    switch (c) {
    case 'r':
        Bsp_Prof_Reset();
        LOG_INFO("window reset");
        break;
    case 'p':
        break;
    default:
        LOG_WARN("unknown key '%c'", c);
        break;
    }
}

static void prof_tick(void) {
    /* Skip a partial window right after entering the mode; the counters still
     * reset so the next report covers a full tick. */
    if (Bsp_Prof_WallCycles() * 2u >= Bsp_Time_GetCpuHz()) {
        prof_report();
    }
    Bsp_Prof_Reset();
}

static const AppConsoleMode s_mode = {
    .key    = 'u',
    .name   = "usage",
    .enter  = prof_enter,
    .exit   = prof_exit,
    .on_key = prof_on_key,
    .report = prof_report,
    .help   = prof_help,
    .tick   = prof_tick,
};

int App_Prof_Init(void) {
    return App_Console_RegisterMode(&s_mode);
}

#else

int App_Prof_Init(void) {
    return 0;
}

#endif