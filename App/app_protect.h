#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drv_charger.h"
#include "drv_gauge.h"

/*
 * Power/temperature protection core. App_Protect_Init()
 * runs one sample burst and evaluate() in the caller's thread, claims I2C2 and
 * starts the 10 ms TIM6 tick; from then on the tick runs the burst every
 * BOARD_PROTECT_SAMPLE_DIV ticks and compares thresholds every tick against
 * the cached snapshot. Charge is inhibited (and TX cut) on any trip; unknown
 * I2C data also inhibits charge. The snapshot is the only read-only view other
 * modules may take; I2C2 is owned by the tick (see bsp_i2c_bus.h).
 */

typedef enum {
    APP_PROTECT_FLAG_BATT_TEMP_HIGH = 1u << 0,
    APP_PROTECT_FLAG_BATT_TEMP_LOW  = 1u << 1,
    APP_PROTECT_FLAG_BATT_TEMP_COLD = 1u << 2,
    APP_PROTECT_FLAG_CELL_OV        = 1u << 3,
    APP_PROTECT_FLAG_CELL_UV        = 1u << 4,
    APP_PROTECT_FLAG_PACK_LOW       = 1u << 5,
    APP_PROTECT_FLAG_SOC_LOW        = 1u << 6,
    APP_PROTECT_FLAG_PA_TEMP_HIGH   = 1u << 7,
    APP_PROTECT_FLAG_PA_FAULT       = 1u << 8,
    APP_PROTECT_FLAG_DC_IN_LOW      = 1u << 9,
    APP_PROTECT_FLAG_DATA_INVALID   = 1u << 10,
    APP_PROTECT_FLAG_CHARGE_INHIBIT = 1u << 11,
    APP_PROTECT_FLAG_TX_CUT         = 1u << 12
} AppProtectFlags;

/* External DC is considered present when the input voltage reaches this level. */
#define APP_PROTECT_DC_PRESENT_MV 9000u

typedef struct {
    DrvGaugeMeas     gauge;
    DrvChargerStatus charger;
    uint16_t         vext_mv;     /* ADC1 IN1, mV */
    uint16_t         vext_raw;    /* ADC1 IN1 raw code (telemetry) */
    uint16_t         pa_ntc_raw;  /* ADC1 IN5 PA NTC raw code; falls as temp rises */
    int32_t          pa_temp_mc;
    uint32_t         flags;       /* AppProtectFlags mask */
    bool             adapter_present; /* derived from vext_mv */
    bool             data_valid;  /* false = gauge/charger data unknown */
    bool             pa_valid;    /* false = ADC/PA NTC reading invalid */
} AppProtectSnapshot;

int  App_Protect_Init(void);

/* Copy the latest snapshot under an IRQ lock. */
void App_Protect_GetSnapshot(AppProtectSnapshot *out);

/* True while the PA TX-inhibit latch is set (PA over-temperature or an invalid
 * NTC reading); App_Radio_SetRadioMode(TX) must refuse. Released only by a
 * valid NTC reading at or below BOARD_PA_TEMP_HIGH_RELEASE_MC. */
bool App_Protect_IsTxInhibited(void);

/* True while the low-battery warning condition holds (critical pack state or
 * unknown gauge data while running from the battery, no adapter); arms the
 * auto-off countdown in App_Power_Tick. */
bool App_Protect_IsBattWarning(void);

/*
 * User charger-enable preference (NVPARAM chge / host cmd21 bit 4). False
 * inhibits charging even when protection allows it; true only removes this
 * veto and never overrides a protection trip. The next TIM6 tick applies the
 * change, because I2C2 is owned by the tick.
 */
void App_Protect_SetChargeEnabled(bool enabled);
