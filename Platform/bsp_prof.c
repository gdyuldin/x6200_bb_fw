#include "bsp_prof.h"

#include <stddef.h>

#include "bsp_irq.h"
#include "bsp_time.h"

static BspProfStats s_isr[BSP_PROF_ISR_COUNT];
static BspProfStats s_section[BSP_PROF_SECTION_COUNT];

/* Mutated from ISR context only. */
static volatile uint32_t s_isr_depth;
static volatile uint32_t s_isr_total;

/* Mutated from the main loop only. */
static uint64_t s_wall_cycles;
static uint64_t s_idle_cycles;
static uint32_t s_loop_last;
static uint32_t s_loop_isr_start;

static void stats_add(BspProfStats *st, uint32_t cycles) {
    st->calls++;
    st->cycles += cycles;
    if (cycles > st->max_cycles) {
        st->max_cycles = cycles;
    }
}

void Bsp_Prof_Reset(void) {
    uint32_t key = Bsp_Irq_Save();

    for (size_t i = 0; i < BSP_PROF_ISR_COUNT; i++) {
        s_isr[i].calls      = 0u;
        s_isr[i].cycles     = 0u;
        s_isr[i].max_cycles = 0u;
    }
    for (size_t i = 0; i < BSP_PROF_SECTION_COUNT; i++) {
        s_section[i].calls      = 0u;
        s_section[i].cycles     = 0u;
        s_section[i].max_cycles = 0u;
    }

    s_isr_depth       = 0u;
    s_isr_total       = 0u;
    s_wall_cycles     = 0u;
    s_idle_cycles     = 0u;
    s_loop_last       = Bsp_Time_GetCycles();
    s_loop_isr_start  = 0u;

    Bsp_Irq_Restore(key);
}

uint32_t Bsp_Prof_IsrEnter(BspProfIsr isr) {
    uint32_t key = Bsp_Irq_Save();
    uint32_t start;

    (void)isr;
    s_isr_depth++;
    start = Bsp_Time_GetCycles();
    Bsp_Irq_Restore(key);
    return start;
}

void Bsp_Prof_IsrExit(BspProfIsr isr, uint32_t start) {
    uint32_t key = Bsp_Irq_Save();
    uint32_t delta = Bsp_Time_GetCycles() - start;

    if (isr < BSP_PROF_ISR_COUNT) {
        stats_add(&s_isr[isr], delta);
    }
    if (s_isr_depth > 0u) {
        s_isr_depth--;
    }
    if (s_isr_depth == 0u) {
        s_isr_total += delta;
    }
    Bsp_Irq_Restore(key);
}

uint32_t Bsp_Prof_SectionEnter(BspProfSection section) {
    (void)section;
    return Bsp_Time_GetCycles();
}

void Bsp_Prof_SectionExit(BspProfSection section, uint32_t start) {
    if (section < BSP_PROF_SECTION_COUNT) {
        stats_add(&s_section[section], Bsp_Time_GetCycles() - start);
    }
}

void Bsp_Prof_IsrStats(BspProfIsr isr, BspProfStats *out) {
    uint32_t key;

    if (out == 0) {
        return;
    }
    if (isr >= BSP_PROF_ISR_COUNT) {
        *out = (BspProfStats){0};
        return;
    }
    key = Bsp_Irq_Save();
    *out = s_isr[isr];
    Bsp_Irq_Restore(key);
}

void Bsp_Prof_SectionStats(BspProfSection section, BspProfStats *out) {
    uint32_t key;

    if (out == 0) {
        return;
    }
    if (section >= BSP_PROF_SECTION_COUNT) {
        *out = (BspProfStats){0};
        return;
    }
    key = Bsp_Irq_Save();
    *out = s_section[section];
    Bsp_Irq_Restore(key);
}

uint32_t Bsp_Prof_IsrTotalCycles(void) {
    return s_isr_total;
}

uint64_t Bsp_Prof_WallCycles(void) {
    return s_wall_cycles;
}

uint64_t Bsp_Prof_IdleCycles(void) {
    return s_idle_cycles;
}

uint32_t Bsp_Prof_LoopBegin(void) {
    uint32_t now = Bsp_Time_GetCycles();

    s_wall_cycles += (uint32_t)(now - s_loop_last);
    s_loop_last = now;
    s_loop_isr_start = s_isr_total;
    return now;
}

void Bsp_Prof_LoopEnd(uint32_t start, bool any_work) {
    uint32_t elapsed;
    uint32_t isr;

    if (any_work) {
        return;
    }
    elapsed = Bsp_Time_GetCycles() - start;
    isr = s_isr_total - s_loop_isr_start;
    if (isr < elapsed) {
        s_idle_cycles += (uint32_t)(elapsed - isr);
    }
}