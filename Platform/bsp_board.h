#pragma once

#include <stdbool.h>

/*
 * Named board-level discrete signals. The port/pin mapping and active levels
 * stay in Platform; App callers see only the logical state.
 */

bool Bsp_Board_IsPowerButtonPressed(void);
bool Bsp_Board_IsExtMicPttPressed(void);
/* CW_HOST is the host-driven virtual CW key, active low with a pull-up. */
bool Bsp_Board_IsCwHostKeyDown(void);
/* EXT MIC DETECT is active low: true means the external/hand mic is plugged. */
bool Bsp_Board_IsExtMicConnected(void);

void Bsp_Board_SetLinuxPower(bool enabled);
void Bsp_Board_SetPowerHold(bool enabled);
void Bsp_Board_SetStatusLedGreen(bool on);
void Bsp_Board_SetStatusLedRed(bool on);

/* Control ACC TRX pin for external PA */
void Bsp_Board_SetAccTrx(bool on);

/* Enable PA biases (turn transistors ON) */
void Bsp_Board_SetPABias(bool on);

/* Connect RF ADC */
void Bsp_Board_ConnectRxADC(bool on);
/* Set input/output relay to TX */
void Bsp_Board_SetTxRelay(bool on);

/* Final power cut: disable interrupts, drop LINUX_POWER/POWER_HOLD and the
 * status LEDs, then reset the MCU. Never returns. */
void Bsp_Board_FinalPowerCut(void) __attribute__((noreturn));
