#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Power domain: PMIC rail bring-up and the software power-off sequence.
 * The button scan, the power-off sequence itself and the auto-off countdown
 * run in the 10 ms TIM6 protection tick (App_Power_Tick), so power-off stays
 * reachable when the main loop is stuck.
 */

int App_Power_Init(void);

/*
 * Request a software power-off. Sources: AUX 'S'/'s', power-button long
 * press (BOARD_POWER_KEY_OFF_MS), the auto-off countdown (host-link idle or
 * battery warning) and host command 0x8009 on that link. One-shot: later
 * calls are ignored once the sequence has started. May be called from the
 * main loop or an ISR; the sequence itself runs in the TIM6 tick.
 */
void App_PowerOff_Request(void);

/* 10 ms tick from the TIM6 protection callback (ISR context). */
void App_Power_Tick(void);

/* Host-link activity ("host alive"): re-arms the auto-off idle window. Called
 * from the per-frame handler of the I2C host protocol. */
void App_Power_NotifyHostFrame(void);

/* Remaining auto-off countdown in seconds (0..BOARD_AUTO_OFF_COUNTDOWN_S);
 * reads the counter driven by auto_off_tick. */
uint16_t App_Power_GetCountdownS(void);

/* True while the auto-off arming condition holds (no host frame for
 * BOARD_AUTO_OFF_IDLE_MS or the battery warning), including the grace phase.
 * False once the power-off sequence has started. */
bool App_Power_IsAutoOffActive(void);
