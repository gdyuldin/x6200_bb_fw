#include "drv_pmic.h"

#include <stddef.h>

#include "bsp_i2c_bus.h"
#include "bsp_log.h"
#include "bsp_time.h"

#define LOG_TAG "drv.pmic"

/*
 * AXP223 rail bring-up, derived from the AXP223 datasheet (V1.0, section 10).
 * Register addresses, bit positions and output voltage codes are all named or
 * computed from the datasheet formulas so nothing is copied from another
 * firmware. The driver talks to the part only through the I2C BSP.
 */

/* AXP223 7-bit I2C address; the BSP shifts it left when framing the bus. */
#define AXP223_I2C_ADDR 0x34u

/* Output control 10H: DCDC1/2/3/4/5 & ALDO1/2 enable. */
#define AXP223_REG_OUTPUT_CTRL1 0x10u
#define AXP223_CTRL1_DCDC1_EN   (1u << 1)
#define AXP223_CTRL1_DCDC2_EN   (1u << 2)
#define AXP223_CTRL1_DCDC3_EN   (1u << 3)
#define AXP223_CTRL1_DCDC5_EN   (1u << 5)
#define AXP223_CTRL1_ALDO1_EN   (1u << 6)

/* Output control 12H: ELDO1/2/3 & DLDO1/2/3/4 & DC1SW enable. */
#define AXP223_REG_OUTPUT_CTRL2 0x12u
#define AXP223_CTRL2_DLDO1_EN   (1u << 3)

/* ALDO3 control 13H: enable is bit 7, bits [6:0] are reserved (stay 1). */
#define AXP223_REG_ALDO3_CTRL   0x13u
#define AXP223_ALDO3_RESERVED   0x01u

/* Output voltage registers. */
#define AXP223_REG_DCDC1_VOUT   0x21u
#define AXP223_REG_DCDC2_VOUT   0x22u
#define AXP223_REG_DCDC3_VOUT   0x23u
#define AXP223_REG_DCDC5_VOUT   0x25u
#define AXP223_REG_ALDO1_VOUT   0x28u
#define AXP223_REG_DLDO1_VOUT   0x15u

/*
 * Output voltage codes from the datasheet linear formulas (mV):
 *   DCDC1 : V = 1.6  V + 0.1  V * N, N = [0, 31]
 *   DCDC2 : V = 0.6  V + 0.02 V * N, N = [0, 63]
 *   DCDC3 : V = 0.6  V + 0.02 V * N, N = [0, 63]
 *   DCDC5 : V = 1.0  V + 0.05 V * N, N = [0, 31]  (DC5SET)
 *   ALDO1 : V = 0.7  V + 0.1  V * N, N = [0, 31]
 *   DLDO1 : V = 0.7  V + 0.1  V * N, N = [0, 31]
 */
#define AXP223_DCDC1_CODE_MV(mv) ((uint8_t)(((mv) - 1600) / 100))
#define AXP223_DCDC2_CODE_MV(mv) ((uint8_t)(((mv) - 600) / 20))
#define AXP223_DCDC3_CODE_MV(mv) ((uint8_t)(((mv) - 600) / 20))
#define AXP223_DCDC5_CODE_MV(mv) ((uint8_t)(((mv) - 1000) / 50))
#define AXP223_ALDO1_CODE_MV(mv) ((uint8_t)(((mv) - 700) / 100))
#define AXP223_DLDO1_CODE_MV(mv) ((uint8_t)(((mv) - 700) / 100))

/* Target output voltages (mV). */
#define AXP223_DCDC1_MV 3300u
#define AXP223_DCDC2_MV 1200u
#define AXP223_DCDC3_MV 1800u
#define AXP223_DCDC5_MV 2500u
#define AXP223_ALDO1_MV 3300u
#define AXP223_DLDO1_MV 1800u

#define AXP223_PMIC_WRITE_TIMEOUT_MS 100u

typedef struct {
    uint8_t reg;
    uint8_t val;
} PmicWrite;

/* Outputs down: only the base CPU rail (DCDC1) stays enabled. */
static const PmicWrite s_pmic_outputs_off[] = {
    { AXP223_REG_OUTPUT_CTRL1, AXP223_CTRL1_DCDC1_EN },
    { AXP223_REG_OUTPUT_CTRL2, 0 },
    { AXP223_REG_ALDO3_CTRL, AXP223_ALDO3_RESERVED },
};

/* Program the rail voltages before enabling the outputs. */
static const PmicWrite s_pmic_voltage_set[] = {
    { AXP223_REG_DCDC1_VOUT, AXP223_DCDC1_CODE_MV(AXP223_DCDC1_MV) },
    { AXP223_REG_DCDC2_VOUT, AXP223_DCDC2_CODE_MV(AXP223_DCDC2_MV) },
    { AXP223_REG_DCDC3_VOUT, AXP223_DCDC3_CODE_MV(AXP223_DCDC3_MV) },
    { AXP223_REG_DCDC5_VOUT, AXP223_DCDC5_CODE_MV(AXP223_DCDC5_MV) },
    { AXP223_REG_ALDO1_VOUT, AXP223_ALDO1_CODE_MV(AXP223_ALDO1_MV) },
    { AXP223_REG_DLDO1_VOUT, AXP223_DLDO1_CODE_MV(AXP223_DLDO1_MV) },
};

/* Enable the application rails: DCDC1/2/3/5, ALDO1 (control 10H) and DLDO1
 * (control 12H). DCDC4 is left at its power-on state. */
static const PmicWrite s_pmic_enable[] = {
    { AXP223_REG_OUTPUT_CTRL1,
      AXP223_CTRL1_DCDC1_EN | AXP223_CTRL1_DCDC2_EN | AXP223_CTRL1_DCDC3_EN |
          AXP223_CTRL1_DCDC5_EN | AXP223_CTRL1_ALDO1_EN },
    { AXP223_REG_OUTPUT_CTRL2, AXP223_CTRL2_DLDO1_EN },
};

static int pmic_write_reg(uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = { reg, val };
    return Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_4, AXP223_I2C_ADDR, buf,
                                    sizeof(buf), 0, 0,
                                    AXP223_PMIC_WRITE_TIMEOUT_MS);
}

static int pmic_write_seq(const PmicWrite *seq, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        int rc = pmic_write_reg(seq[i].reg, seq[i].val);
        if (rc != 0) {
            return rc;
        }
    }
    return 0;
}

int Drv_Pmic_Init(void) {
    int rc = pmic_write_seq(s_pmic_outputs_off,
                            sizeof(s_pmic_outputs_off) /
                                sizeof(s_pmic_outputs_off[0]));
    if (rc != 0) {
        return rc;
    }

    rc = pmic_write_seq(s_pmic_voltage_set,
                        sizeof(s_pmic_voltage_set) /
                            sizeof(s_pmic_voltage_set[0]));
    if (rc != 0) {
        return rc;
    }

    Bsp_Time_DelayMs(10u);

    rc = pmic_write_seq(s_pmic_enable,
                        sizeof(s_pmic_enable) / sizeof(s_pmic_enable[0]));
    if (rc == 0) {
        LOG_DEBUG("init ok");
    }
    return rc;
}

void Drv_Pmic_DropOutputs(void) {
    (void)pmic_write_seq(s_pmic_outputs_off,
                         sizeof(s_pmic_outputs_off) /
                             sizeof(s_pmic_outputs_off[0]));
}
