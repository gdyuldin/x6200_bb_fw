#include "drv_charger.h"

#include <stdbool.h>
#include <stdint.h>

#include "bsp_i2c_bus.h"
#include "bsp_time.h"
#include "bsp_log.h"

#define LOG_TAG "drv.charger"

/*
 * Register/bit values from the BQ24725 datasheet. 16-bit registers travel
 * SMBus little-endian ([reg, lo, hi]); the pointer-register read is a
 * write-then-read pair. ChargeOption bit0 = Charge Inhibit (1 = charging
 * inhibited); ChargeOption bit4 = AC adapter indication (read-only, 1 =
 * ACDET > 2.4 V); ChargeOption bits [14:13] = watchdog timeout (00 =
 * disabled). ChargeCurrent bits [12:6], step 64 mA at a 10 mOhm sense
 * resistor. ChargeVoltage bits [14:4], step 16 mV.
 */

#define BQ24725_I2C_ADDR 0x09u /* 7-bit (8-bit write address 0x12) */
#define BQ24725_REG_CHARGE_OPTION 0x12u
#define BQ24725_REG_CHARGE_CURRENT 0x14u
#define BQ24725_REG_CHARGE_VOLTAGE 0x15u
#define BQ24725_REG_INPUT_CURRENT 0x3Fu
#define BQ24725_REG_MANUFACTURER_ID 0xFEu
#define BQ24725_REG_DEVICE_ID 0xFFu

#define BQ24725_OPTION_CHARGE_INHIBIT (1u << 0)
#define BQ24725_OPTION_AC_PRESENT (1u << 4)
#define BQ24725_OPTION_WATCHDOG_MASK (0x3u << 13)
#define BQ24725_OPTION_WATCHDOG_44S (0x1u << 13)

#define BQ24725_CHG_CURRENT_CODE_MA(ma) ((uint16_t)((ma) / 64u))
#define BQ24725_CHG_CURRENT_SHIFT 6u
#define BQ24725_CHG_VOLTAGE_CODE_MV(mv) ((uint16_t)((mv) / 16u))
#define BQ24725_CHG_VOLTAGE_SHIFT 4u

/* Bounded per-transfer timeout: a burst runs in the TIM6 ISR, so a stuck bus
 * must fail quickly. At ~100 kHz a word transfer is ~1 ms. */
#define BQ24725_XFER_TIMEOUT_MS 5u

static DrvChargerRegs s_regs;

static int charger_read_word(uint8_t reg, uint16_t *value) {
    uint8_t word[2];
    int rc = Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_2, BQ24725_I2C_ADDR, &reg, 1u,
                                      word, 2u, BQ24725_XFER_TIMEOUT_MS);
    if (rc != 0) {
        return rc;
    }
    *value = (uint16_t)word[0] | ((uint16_t)word[1] << 8);
    return 0;
}

static int charger_write_word(uint8_t reg, uint16_t value) {
    const uint8_t tx[3] = {reg, (uint8_t)(value & 0xFFu),
                           (uint8_t)(value >> 8)};
    return Bsp_I2c_WriteTimeout(BSP_I2C_BUS_2, BQ24725_I2C_ADDR, tx, 3u,
                                BQ24725_XFER_TIMEOUT_MS);
}

int Drv_Charger_Init(void) {
    uint16_t code;
    int rc;

    rc = charger_read_word(BQ24725_REG_CHARGE_OPTION, &s_regs.charge_option);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_CHARGE_CURRENT, &s_regs.charge_current);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_CHARGE_VOLTAGE, &s_regs.charge_voltage);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_INPUT_CURRENT, &s_regs.input_current);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_MANUFACTURER_ID,
                           &s_regs.manufacturer_id);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_DEVICE_ID, &s_regs.device_id);
    if (rc != 0) {
        return rc;
    }

    /*
     * Program the SMBus watchdog (bits [14:13] = 01), then read back. If the
     * MCU hangs and stops refreshing ChargeCurrent/ChargeVoltage, the charger
     * suspends on its own. The bootloader's charge loop feeds it at 1 Hz, so
     * off-state charging still works.
     */
    code = (uint16_t)((s_regs.charge_option &
                       (uint16_t)~BQ24725_OPTION_WATCHDOG_MASK) |
                      BQ24725_OPTION_WATCHDOG_44S);
    rc = charger_write_word(BQ24725_REG_CHARGE_OPTION, code);
    if (rc != 0) {
        return rc;
    }
    Bsp_Time_DelayMs(1u);
    rc = charger_read_word(BQ24725_REG_CHARGE_OPTION, &s_regs.charge_option);
    if (rc != 0) {
        return rc;
    }

    /* Charge current: 512 mA -> code 8 in bits [15:6]. */
    code = (uint16_t)(BQ24725_CHG_CURRENT_CODE_MA(DRV_CHARGER_CHG_CURRENT_MA)
                      << BQ24725_CHG_CURRENT_SHIFT);
    rc = charger_write_word(BQ24725_REG_CHARGE_CURRENT, code);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_CHARGE_CURRENT, &s_regs.charge_current);
    if (rc != 0) {
        return rc;
    }

    /* Charge voltage: 8704 mV -> code 0x220 in bits [15:4]. */
    code = (uint16_t)(BQ24725_CHG_VOLTAGE_CODE_MV(DRV_CHARGER_CHG_VOLTAGE_MV)
                      << BQ24725_CHG_VOLTAGE_SHIFT);
    rc = charger_write_word(BQ24725_REG_CHARGE_VOLTAGE, code);
    if (rc != 0) {
        return rc;
    }
    rc = charger_read_word(BQ24725_REG_CHARGE_VOLTAGE, &s_regs.charge_voltage);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

const DrvChargerRegs *Drv_Charger_GetRegs(void) {
    return &s_regs;
}

static int charger_set_charge_inhibit(bool inhibit) {
    uint16_t option;
    int rc = charger_read_word(BQ24725_REG_CHARGE_OPTION, &option);
    if (rc != 0) {
        return rc;
    }
    if (inhibit) {
        option |= BQ24725_OPTION_CHARGE_INHIBIT;
    } else {
        option &= (uint16_t)~BQ24725_OPTION_CHARGE_INHIBIT;
    }
    rc = charger_write_word(BQ24725_REG_CHARGE_OPTION, option);
    if (rc == 0) {
        s_regs.charge_option = option;
    }
    return rc;
}

int Drv_Charger_EnableCharge(void) {
    return charger_set_charge_inhibit(false);
}

int Drv_Charger_InhibitCharge(void) {
    return charger_set_charge_inhibit(true);
}

int Drv_Charger_ReadStatus(DrvChargerStatus *status) {
    uint16_t option;
    int rc;

    if (status == 0) {
        return -1;
    }
    rc = charger_read_word(BQ24725_REG_CHARGE_OPTION, &option);
    if (rc != 0) {
        return rc;
    }
    s_regs.charge_option = option;
    status->option = option;
    status->ac_ok = (option & BQ24725_OPTION_AC_PRESENT) != 0u;
    return 0;
}

int Drv_Charger_SetChargeCurrent(uint32_t ma) {
    uint16_t code;
    int rc;

    if (ma < DRV_CHARGER_CHG_CURRENT_MIN_MA ||
        ma > DRV_CHARGER_CHG_CURRENT_MAX_MA) {
        return -1;
    }
    code = (uint16_t)(BQ24725_CHG_CURRENT_CODE_MA(ma)
                      << BQ24725_CHG_CURRENT_SHIFT);
    rc = charger_write_word(BQ24725_REG_CHARGE_CURRENT, code);
    if (rc == 0) {
        s_regs.charge_current = code;
    } else {
        LOG_WARN("SetChargeCurrent error: %i", rc);
    }
    return rc;
}

int Drv_Charger_SetChargeVoltage(uint32_t mv) {
    uint16_t code;
    int rc;

    if (mv < DRV_CHARGER_CHG_VOLTAGE_MIN_MV ||
        mv > DRV_CHARGER_CHG_VOLTAGE_MAX_MV) {
        return -1;
    }
    code = (uint16_t)(BQ24725_CHG_VOLTAGE_CODE_MV(mv)
                      << BQ24725_CHG_VOLTAGE_SHIFT);
    rc = charger_write_word(BQ24725_REG_CHARGE_VOLTAGE, code);
    if (rc == 0) {
        s_regs.charge_voltage = code;
    } else {
        LOG_WARN("SetChargeVoltage error: %i", rc);
    }
    return rc;
}
