#include "drv_gauge.h"

#include <stdint.h>

#include "bsp_i2c_bus.h"

/*
 * Register/bit values from the MAX172xx datasheet: Config (01Dh) bit7 = SHDN;
 * once set, the gauge enters shutdown after its ShdnTimer delay. RepCap (005h)
 * is read at probe to confirm the gauge responds. Words travel little-endian
 * ([reg, lo, hi]); the pointer-register read is a write-then-read pair, the same
 * shape the device helpers use for word accesses. Measurement register LSBs:
 * Temp 1/256 °C, Batt 1.25 mV, Cell1/Cell2 78.125 uV, AvgCurrent 156.25 uA at
 * the 10 mOhm pack sense resistor (0.15625 mA), RepSOC 1/256 %.
 */

#define MAX172XX_I2C_ADDR 0x36u /* 7-bit (8-bit write address 0x6c) */
#define MAX172XX_REG_REPCAP 0x005u
#define MAX172XX_REG_REPSOC 0x006u
#define MAX172XX_REG_TEMP 0x008u
#define MAX172XX_REG_AVGCURRENT 0x00Bu
#define MAX172XX_REG_CONFIG 0x1Du
#define MAX172XX_REG_CELL2 0x0D7u
#define MAX172XX_REG_CELL1 0x0D8u
#define MAX172XX_REG_BATT 0x0DAu
#define MAX172XX_CONFIG_SHDN (1u << 7)
/* Bounded per-transfer timeout: the burst runs in the TIM6 ISR, so a stuck
 * bus must fail quickly rather than block the ISR. */
#define MAX172XX_XFER_TIMEOUT_MS 5u

static int gauge_read_word(uint8_t reg, uint16_t *value) {
    uint8_t word[2];
    int rc = Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_2, MAX172XX_I2C_ADDR, &reg,
                                      1u, word, 2u, MAX172XX_XFER_TIMEOUT_MS);
    if (rc != 0) {
        return rc;
    }
    *value = (uint16_t)word[0] | ((uint16_t)word[1] << 8);
    return 0;
}

static int gauge_write_word(uint8_t reg, uint16_t value) {
    const uint8_t tx[3] = {reg, (uint8_t)(value & 0xFFu),
                           (uint8_t)(value >> 8)};
    return Bsp_I2c_WriteTimeout(BSP_I2C_BUS_2, MAX172XX_I2C_ADDR, tx, 3u,
                                MAX172XX_XFER_TIMEOUT_MS);
}

int Drv_Gauge_Init(void) {
    uint16_t probe;
    return gauge_read_word(MAX172XX_REG_REPCAP, &probe);
}

int Drv_Gauge_Shutdown(void) {
    uint16_t config;
    int rc = gauge_read_word(MAX172XX_REG_CONFIG, &config);
    if (rc != 0) {
        return rc;
    }
    config |= MAX172XX_CONFIG_SHDN;
    return gauge_write_word(MAX172XX_REG_CONFIG, config);
}

int Drv_Gauge_ReadMeasurements(DrvGaugeMeas *meas) {
    uint16_t temp;
    uint16_t batt;
    uint16_t cell1;
    uint16_t cell2;
    uint16_t avg_curr;
    uint16_t soc;
    int rc;

    if (meas == 0) {
        return -1;
    }
    rc = gauge_read_word(MAX172XX_REG_TEMP, &temp);
    if (rc != 0) {
        return rc;
    }
    rc = gauge_read_word(MAX172XX_REG_BATT, &batt);
    if (rc != 0) {
        return rc;
    }
    rc = gauge_read_word(MAX172XX_REG_CELL1, &cell1);
    if (rc != 0) {
        return rc;
    }
    rc = gauge_read_word(MAX172XX_REG_CELL2, &cell2);
    if (rc != 0) {
        return rc;
    }
    rc = gauge_read_word(MAX172XX_REG_AVGCURRENT, &avg_curr);
    if (rc != 0) {
        return rc;
    }
    rc = gauge_read_word(MAX172XX_REG_REPSOC, &soc);
    if (rc != 0) {
        return rc;
    }

    /* LSB conversions; multiplication keeps the 16-bit registers in 32-bit. */
    meas->temp_mc = ((int32_t)(int16_t)temp * 125) / 32;
    meas->batt_mv = ((uint32_t)batt * 5u) / 4u;
    meas->cell1_mv = ((uint32_t)cell1 * 5u) / 64u;
    meas->cell2_mv = ((uint32_t)cell2 * 5u) / 64u;
    meas->curr_ma = ((int32_t)(int16_t)avg_curr * 5) / 32;
    meas->soc_d_pct = (uint16_t)(((uint32_t)soc * 5u) / 128u);
    return 0;
}
