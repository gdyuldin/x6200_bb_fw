#include "app_power.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_nvram.h"
#include "app_protect.h"
#include "app_radio.h"
#include "board_config.h"
#include "bsp_board.h"
#include "bsp_boot.h"
#include "bsp_log.h"
#include "drv_charger.h"
#include "drv_gauge.h"
#include "drv_pmic.h"

#define LOG_TAG "app.power"

#define APP_POWER_TICK_MS 10u

typedef enum {
    APP_POWER_PHASE_RUN = 0,
    APP_POWER_PHASE_SETTLE
} AppPowerPhase;

static AppPowerPhase    s_phase;
static volatile bool    s_off_request;
static uint32_t         s_settle_ticks;
static uint32_t         s_key_low_ms;
/* Written from the I2C host-link ISR (App_Power_NotifyHostFrame), read from the
 * TIM6 tick: volatile so the tick observes the reset promptly. */
static volatile uint32_t s_host_idle_ms;
static uint32_t         s_sec_div;
static uint16_t         s_grace_s = BOARD_AUTO_OFF_GRACE_S;
static uint16_t         s_countdown_s = BOARD_AUTO_OFF_COUNTDOWN_S;

/* Phase 0: one-shot safe-off steps, run from the TIM6 tick (the I2C2 owner,
 * no bus guard needed). Transfer timeouts are bounded so a stuck bus cannot
 * stall the shutdown; the main loop keeps logging and kicking the IWDG. */
static void poweroff_phase0(void) {
    int rc;

    rc = Drv_Charger_InhibitCharge();
    if (rc != 0) {
        LOG_ERROR("charger inhibit failed (%d)", rc);
    }
    rc = Drv_Gauge_Shutdown();
    if (rc != 0) {
        LOG_ERROR("gauge shutdown failed (%d)", rc);
    }
    /* Persist the charger on/off preference for the bootloader charge loop
     * (NVPARAM at EEPROM 0x2000). Deliberate exception to the "no EEPROM write
     * from an ISR" rule: this runs once during power-off, where the ~10 ms
     * blocking write is acceptable and the rail is still up. */
    rc = App_Nvram_ChargeSave();
    if (rc != 0) {
        LOG_ERROR("nvparam save failed (%d)", rc);
    }
    (void)App_Radio_SetRadioMode(APP_RADIO_MODE_OFF);
    Drv_Pmic_DropOutputs();
}

/* Phase 2: final cut, no return. */
static void poweroff_phase2(void) __attribute__((noreturn));

static void poweroff_phase2(void) {
    Bsp_Board_FinalPowerCut();
}

/* Auto power-off tick: while the arming condition holds, a grace period then a
 * countdown runs to power-off; clearing it re-arms both counters. The arm is
 * "no host frame for BOARD_AUTO_OFF_IDLE_MS" or the battery warning; host
 * frames are counted in App_Power_NotifyHostFrame. */
static void auto_off_tick(void) {
    if (s_host_idle_ms >= BOARD_AUTO_OFF_IDLE_MS ||
        App_Protect_IsBattWarning()) {
        if (s_grace_s > 0u) {
            s_grace_s--;
        } else if (s_countdown_s > 0u) {
            s_countdown_s--;
        } else {
            App_PowerOff_Request();
        }
    } else {
        s_grace_s = BOARD_AUTO_OFF_GRACE_S;
        s_countdown_s = BOARD_AUTO_OFF_COUNTDOWN_S;
    }
}

void App_Power_Tick(void) {
    if (s_off_request) {
        s_off_request = false;
        LOG_INFO("off requested");
        poweroff_phase0();
        s_settle_ticks = BOARD_POWER_OFF_SETTLE_MS / APP_POWER_TICK_MS;
        s_phase = APP_POWER_PHASE_SETTLE;
    }

    switch (s_phase) {
    case APP_POWER_PHASE_RUN:
        /* Power button held > 5 s = emergency off. */
        if (Bsp_Board_IsPowerButtonPressed()) {
            s_key_low_ms += APP_POWER_TICK_MS;
            if (s_key_low_ms >= BOARD_POWER_KEY_OFF_MS) {
                s_key_low_ms = 0u;
                App_PowerOff_Request();
            }
        } else {
            s_key_low_ms = 0u;
        }
        if (s_host_idle_ms < BOARD_AUTO_OFF_IDLE_MS) {
            s_host_idle_ms += APP_POWER_TICK_MS;
        }
        s_sec_div++;
        /* 1 Hz pace of the auto-off countdown. */
        if (s_sec_div >= 100u) {
            s_sec_div = 0u;
            auto_off_tick();
        }
        break;
    case APP_POWER_PHASE_SETTLE:
        if (s_settle_ticks > 0u) {
            s_settle_ticks--;
        } else {
            poweroff_phase2();
        }
        break;
    default:
        break;
    }
}

void App_PowerOff_Request(void) {
    if (s_phase != APP_POWER_PHASE_RUN) {
        return;
    }
    s_off_request = true;
}

void App_Power_NotifyHostFrame(void) {
    s_host_idle_ms = 0u;
}

uint16_t App_Power_GetCountdownS(void) {
    return s_countdown_s;
}

bool App_Power_IsAutoOffActive(void) {
    if (s_phase != APP_POWER_PHASE_RUN) {
        return false;
    }
    return s_host_idle_ms >= BOARD_AUTO_OFF_IDLE_MS ||
           App_Protect_IsBattWarning();
}

int App_Power_Init(void) {
    /*
     * Bring the PMIC rails up. A failure is logged and reported, but it must
     * not stop the boot; the rail outputs are then dropped before the
     * bootloader hand-off through the BSP hook, so that a rail glitch cannot
     * re-latch CPURSTF and the IAP gate stays valid.
     */
    int rc = Drv_Pmic_Init();
    if (rc != 0) {
        LOG_WARN("pmic init failed (%d), continuing", rc);
    } else {
        LOG_INFO("pmic rails up");
    }

    /*
     * Charger and gauge bring-up: both are best-effort. The charger may be
     * unpowered and the gauge is absent without a battery, so a failure is
     * logged and the boot continues.
     */
    rc = Drv_Charger_Init();
    if (rc != 0) {
        LOG_WARN("charger init failed (%d), continuing", rc);
    } else {
        const DrvChargerRegs *regs = Drv_Charger_GetRegs();
        LOG_INFO("charger ready: mfg 0x%04x dev 0x%04x, "
                 "chg %umA (0x%04x), v %umV (0x%04x)",
                 (unsigned)regs->manufacturer_id,
                 (unsigned)regs->device_id,
                 (unsigned)DRV_CHARGER_CHG_CURRENT_MA,
                 (unsigned)regs->charge_current,
                 (unsigned)DRV_CHARGER_CHG_VOLTAGE_MV,
                 (unsigned)regs->charge_voltage);
    }

    rc = Drv_Gauge_Init();
    if (rc != 0) {
        LOG_WARN("gauge not found (%d)", rc);
    } else {
        LOG_INFO("gauge found");
    }

    Bsp_Boot_SetPreHandoffHook(Drv_Pmic_DropOutputs);
    s_phase = APP_POWER_PHASE_RUN;
    return 0;
}
