#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * bq24725 Li battery charger (I2C2).
 *
 * Drv_Charger_Init() brings the charger up: probe, SMBus watchdog enable
 * (DRV_CHARGER_WATCHDOG_S), charge current DRV_CHARGER_CHG_CURRENT_MA and charge
 * voltage DRV_CHARGER_CHG_VOLTAGE_MV, each write confirmed by a read-back.
 * Drv_Charger_GetRegs() exposes the values read during init (ManufacturerID/
 * DeviceID and the read-back ChargeCurrent/ChargeVoltage) so the caller can log
 * them; a future charger monitor reuses the same cache.
 *
 * Runtime control: EnableCharge/InhibitCharge toggle ChargeOption bit0 (0 =
 * charging allowed), ReadStatus reports the ChargeOption word with the
 * adapter-present bit the charger exposes as status, SetChargeCurrent/
 * SetChargeVoltage program the regulation envelope and refresh the SMBus
 * watchdog. Drv_Charger_InhibitCharge() belongs to the power-off path.
 *
 * Returns 0 on success or a negative error code (bus code or -1 for an
 * out-of-range setting).
 */

typedef struct {
    uint16_t charge_option;
    uint16_t charge_current;
    uint16_t charge_voltage;
    uint16_t input_current;
    uint16_t manufacturer_id;
    uint16_t device_id;
} DrvChargerRegs;

typedef struct {
    uint16_t option; /* raw ChargeOption (0x12) word */
    bool     ac_ok;  /* true = AC adapter present (bit4, ACDET > 2.4 V) */
} DrvChargerStatus;

/* Functional bring-up parameters from the charger init contract. */
#define DRV_CHARGER_CHG_CURRENT_MA 512u
#define DRV_CHARGER_CHG_VOLTAGE_MV 8704u

/*
 * SMBus watchdog timeout programmed in ChargeOption [14:13] (bits = 01).
 * Fail-safe: if the MCU stops refreshing ChargeCurrent/ChargeVoltage the
 * charger auto-suspends. The bootloader's charge loop feeds it at 1 Hz, so
 * off-state charging still works.
 */
#define DRV_CHARGER_WATCHDOG_S 44u

/* Datasheet limits: writing outside these ranges clears the register and
 * terminates charging. */
#define DRV_CHARGER_CHG_CURRENT_MIN_MA 128u
#define DRV_CHARGER_CHG_CURRENT_MAX_MA 8128u
#define DRV_CHARGER_CHG_VOLTAGE_MIN_MV 1024u
#define DRV_CHARGER_CHG_VOLTAGE_MAX_MV 19200u

int                   Drv_Charger_Init(void);
int                   Drv_Charger_EnableCharge(void);
int                   Drv_Charger_InhibitCharge(void);
int                   Drv_Charger_ReadStatus(DrvChargerStatus *status);
int                   Drv_Charger_SetChargeCurrent(uint32_t ma);
int                   Drv_Charger_SetChargeVoltage(uint32_t mv);
const DrvChargerRegs *Drv_Charger_GetRegs(void);
