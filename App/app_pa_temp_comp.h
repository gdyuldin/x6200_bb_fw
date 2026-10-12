#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Open-loop PA bias temperature compensation. Reads the PA NTC every 20 ms,
 * smooths it with a 1/8 EMA and moves a non-positive DAC-code offset on both
 * the drive and the final channel every 40 ms. The base bias remains owned by
 * NVRAM/host; this module only supplies the offsets to Drv_PaBias. An invalid
 * NTC freezes the last applied correction. Protection keeps using its own raw
 * thresholds and does not consume the EMA.
 */

typedef struct {
    int32_t ema_mc;          /* smoothed PA temperature, millidegrees C */
    int32_t target_final_q8; /* target final-channel correction, Q8 DAC codes */
    int32_t applied_final_q8;/* final actuator position, Q8 DAC codes */
    int32_t target_drive_q8; /* target drive-channel correction, Q8 DAC codes */
    int32_t applied_drive_q8;/* drive actuator position, Q8 DAC codes */
    bool    valid;           /* last NTC reading was valid */
    bool    seeded;          /* EMA has been initialized from a valid reading */
} AppPaTempCompStatus;

/* Zero state and seed from the current NTC reading when valid. */
int  App_PaTempComp_Init(void);

/* Call once per 10 ms TIM6 tick; the module applies its own 20/40 ms dividers. */
void App_PaTempComp_Tick10ms(void);

/* Copy the diagnostic state under an IRQ lock. */
void App_PaTempComp_GetStatus(AppPaTempCompStatus *out);
