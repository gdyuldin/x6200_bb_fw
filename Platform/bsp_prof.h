#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Cycle-count profiling on the DWT cycle counter (Bsp_Time_GetCycles).
 *
 * ISR slots measure the application-level callback bodies, not the HAL IRQ
 * dispatch or vector entry. Section slots measure a region of the main loop.
 * Per-ISR counters include time spent in higher-priority ISRs that preempted
 * them, so a slot percentage is inclusive; the window total counts only the
 * outermost ISR and is what the overall load uses. Section slots likewise
 * include ISR time that preempted them.
 *
 * BSP_PROF_ENABLE is the build switch: when 0 the instrumentation macros
 * expand to nothing and the scheduler loop hooks are compiled out.
 */
#ifndef BSP_PROF_ENABLE
#define BSP_PROF_ENABLE 1
#endif

typedef enum {
    BSP_PROF_ISR_FPGA_TICK = 0,
    BSP_PROF_ISR_ADC_MON,
    BSP_PROF_ISR_CODEC,
    BSP_PROF_ISR_CW_KEY,
    BSP_PROF_ISR_PROTECT,
    BSP_PROF_ISR_USART2,
    BSP_PROF_ISR_USART1,
    BSP_PROF_ISR_COUNT
} BspProfIsr;

typedef enum {
    BSP_PROF_SECTION_SPECTRUM = 0,
    BSP_PROF_SECTION_NR,
    BSP_PROF_SECTION_COUNT
} BspProfSection;

typedef struct {
    uint32_t calls;
    uint32_t cycles;
    uint32_t max_cycles;
} BspProfStats;

void Bsp_Prof_Reset(void);

uint32_t Bsp_Prof_IsrEnter(BspProfIsr isr);
void     Bsp_Prof_IsrExit(BspProfIsr isr, uint32_t start);
uint32_t Bsp_Prof_SectionEnter(BspProfSection section);
void     Bsp_Prof_SectionExit(BspProfSection section, uint32_t start);

void Bsp_Prof_IsrStats(BspProfIsr isr, BspProfStats *out);
void Bsp_Prof_SectionStats(BspProfSection section, BspProfStats *out);

uint32_t Bsp_Prof_IsrTotalCycles(void);
uint64_t Bsp_Prof_WallCycles(void);
uint64_t Bsp_Prof_IdleCycles(void);

/*
 * Called once per App_Scheduler_Run() iteration around the event/task
 * dispatch. LoopEnd() classifies the iteration as idle only when no handler
 * ran, then removes the ISR cycles that occurred inside it: the remainder is
 * main-loop idle spin. Everything not idle (tasks, ISRs, loop overhead) is
 * busy, so the overall load needs no per-task instrumentation.
 */
uint32_t Bsp_Prof_LoopBegin(void);
void     Bsp_Prof_LoopEnd(uint32_t start, bool any_work);

#if BSP_PROF_ENABLE
#define BSP_PROF_ISR_BEGIN(isr, tok)   uint32_t tok = Bsp_Prof_IsrEnter(isr)
#define BSP_PROF_ISR_END(isr, tok)     Bsp_Prof_IsrExit(isr, tok)
#define BSP_PROF_SECTION_BEGIN(sec, tok) uint32_t tok = Bsp_Prof_SectionEnter(sec)
#define BSP_PROF_SECTION_END(sec, tok)   Bsp_Prof_SectionExit(sec, tok)
#else
#define BSP_PROF_ISR_BEGIN(isr, tok)   uint32_t tok = 0u
#define BSP_PROF_ISR_END(isr, tok)     ((void)(tok))
#define BSP_PROF_SECTION_BEGIN(sec, tok) uint32_t tok = 0u
#define BSP_PROF_SECTION_END(sec, tok)   ((void)(tok))
#endif