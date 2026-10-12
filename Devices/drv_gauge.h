#pragma once

#include <stdint.h>

/*
 * MAX172xx ModelGauge m5 fuel gauge (I2C2, inside the 4-pin battery pack).
 *
 * Drv_Gauge_Init() probes the gauge with a single read; a missing gauge (no
 * battery) is reported as a negative bus code and must not stop the boot. The
 * gauge configuration lives in the pack's non-volatile memory (nPackCfg) and is
 * never read or written here. Drv_Gauge_Shutdown() sets Config.SHDN so the gauge
 * stops draining the pack and belongs to the power-off path.
 *
 * Drv_Gauge_ReadMeasurements() reads the runtime registers and converts them
 * to engineering units (register LSBs from the MAX172xx datasheet). Cell1/
 * Cell2 are only meaningful when the pack provisioned nPackCfg.ChEn; a pack
 * without per-cell wiring reports zeros there.
 *
 * Returns 0 on success or a negative bus error code.
 */
typedef struct {
    int32_t  temp_mc;   /* Temp 008h, 1/256 °C  -> 0.001 °C */
    uint32_t batt_mv;   /* Batt 0DAh, 1.25 mV   -> 1 mV */
    uint32_t cell1_mv;  /* Cell1 0D8h, 78.125 uV -> 1 mV */
    uint32_t cell2_mv;  /* Cell2 0D7h, 78.125 uV -> 1 mV */
    int32_t  curr_ma;   /* AvgCurrent 00Bh, 0.15625 mA @ 10 mOhm -> 1 mA */
    uint16_t soc_d_pct; /* RepSOC 006h, 1/256 % -> 0.1 % */
} DrvGaugeMeas;

int Drv_Gauge_Init(void);
int Drv_Gauge_Shutdown(void);
int Drv_Gauge_ReadMeasurements(DrvGaugeMeas *meas);
