#pragma once

/*
 * Console calibration modes. Each registers one mode with App/app_console: its
 * own keys, status report and tick. The calibration TX request ("TX without
 * signal") drives radio/PTT directly from each mode.
 */

int App_CalBias_Init(void);
int App_CalAlc_Init(void);
int App_CalComp_Init(void);
