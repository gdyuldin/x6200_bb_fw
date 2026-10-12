#pragma once

#include <stdint.h>

/*
 * Renesas 5P49V6965 (VersaClock 6E) clock generator on I2C1.
 *
 * Drv_ClockGen_Init() programs a single output: OUT1 = 122.88 MHz LVDS
 * 3.3 V from the 26 MHz reference crystal; FOD2-4 and OUT2-4 stay
 * disabled. Register fields and the apply/lock sequence are derived from
 * the 5P49V6965 datasheet and the VersaClock 6E Register Descriptions and
 * Programming Guide. The Trim RAM 0x01-0x0F, the test RAM 0x6A-0x6F and
 * factory-reserved fields are never written (they break VCO
 * auto-calibration).
 *
 * Returns 0 on success, -ENODEV when the part does not answer at 0x68 or
 * 0x6A, or the negative error code of the first failed transfer. The boot
 * is never aborted.
 */

/* Reference crystal and target output (board contract). */
#define DRV_CLOCKGEN_FREF_HZ         26000000u
#define DRV_CLOCKGEN_OUT1_HZ         122880000u

/* PLL: F_VCO = F_REF / prediv * M = 26 MHz / 2 * 222 = 2886 MHz. */
#define DRV_CLOCKGEN_PLL_PREDIV      2u
#define DRV_CLOCKGEN_PLL_FB_INT      222u

/* FOD1: F_OUT1 = (F_VCO / 2) / (N_INT + N_FRAC / 2^24), exact for
 * 122.88 MHz: N = 11 + 761/1024, N_FRAC = 761 * 2^14. */
#define DRV_CLOCKGEN_FOD1_N_INT      11u
#define DRV_CLOCKGEN_FOD1_N_FRAC     0x0BE4000u

#define DRV_CLOCKGEN_ADDR7           0x68u
#define DRV_CLOCKGEN_ADDR7_ALT       0x6Au
#define DRV_CLOCKGEN_XFER_TIMEOUT_MS 100u

int Drv_ClockGen_Init(void);
