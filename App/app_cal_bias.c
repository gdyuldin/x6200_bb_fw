#include "app_cal.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_console.h"
#include "app_nvram.h"
#include "app_protect.h"
#include "app_ptt.h"
#include "app_radio.h"
#include "bsp_log.h"
#include "drv_pa_bias.h"

#define LOG_TAG "app.cal.bias"

#define BIAS_STEP 10

static uint16_t s_drive;
static uint16_t s_final;
static bool     s_sel_final;
static bool     s_drive_off;
static bool     s_final_off;
static bool     s_tx_on;
static bool     s_active;

/* Key the PA without a signal: the calibration TX request drives both the
 * transition and the signal inhibit. */
static void bias_set_tx(bool on) {
    s_tx_on = on;
    App_Ptt_SetCalibrate(on);
}

static void bias_apply(void) {
    int rc;

    rc = Drv_PaBias_SetBias(s_drive, s_final);
    if (rc != 0) {
        LOG_WARN("bias apply failed (%d)", rc);
    }
    rc = Drv_PaBias_SetOff(s_drive_off, s_final_off);
    if (rc != 0) {
        LOG_WARN("bias off apply failed (%d)", rc);
    }
}

static void bias_report(void) {
    AppProtectSnapshot p;
    int32_t            t;
    uint32_t           t_abs;

    App_Protect_GetSnapshot(&p);
    if (!p.pa_valid) {
        LOG_INFO("ch=%s drive=%u%s final=%u%s tx=%u temp=--",
                 s_sel_final ? "final" : "drive", (unsigned)s_drive,
                 s_drive_off ? " off" : "", (unsigned)s_final,
                 s_final_off ? " off" : "",
                 (unsigned)(App_Radio_GetRadioMode() == APP_RADIO_MODE_TX));
        return;
    }

    t     = p.pa_temp_mc;
    t_abs = (t < 0) ? (uint32_t)(-t) : (uint32_t)t;
    LOG_INFO("ch=%s drive=%u%s final=%u%s tx=%u temp=%s%u.%01uC",
             s_sel_final ? "final" : "drive", (unsigned)s_drive,
             s_drive_off ? " off" : "", (unsigned)s_final,
             s_final_off ? " off" : "",
             (unsigned)(App_Radio_GetRadioMode() == APP_RADIO_MODE_TX),
             (t < 0) ? "-" : "", (unsigned)(t_abs / 1000u),
             (unsigned)((t_abs % 1000u) / 100u));
}

static void bias_enter(void) {
    const DrvNvramCalib *calib = App_Nvram_Calib();

    s_drive     = calib->bias_drive;
    s_final     = calib->bias_final;
    s_drive_off = false;
    s_final_off = false;
    s_active    = true;
    bias_set_tx(false);
    bias_apply();
    bias_report();
}

static void bias_exit(void) {
    s_active = false;
    bias_set_tx(false);
}

static void bias_on_key(char c) {
    uint16_t *value = s_sel_final ? &s_final : &s_drive;
    bool     *off   = s_sel_final ? &s_final_off : &s_drive_off;

    switch (c) {
    case 'd':
        s_sel_final = false;
        break;
    case 'f':
        s_sel_final = true;
        break;
    case '+':
        if (*value <= (uint16_t)(0x0FFFu - BIAS_STEP)) {
            *value = (uint16_t)(*value + BIAS_STEP);
        } else {
            *value = 0x0FFFu;
        }
        *off = false;
        bias_apply();
        break;
    case '-':
        if (*value >= BIAS_STEP) {
            *value = (uint16_t)(*value - BIAS_STEP);
        } else {
            *value = 0u;
        }
        *off = false;
        bias_apply();
        break;
    case '0':
        *off = !*off;
        bias_apply();
        break;
    case 't':
        bias_set_tx(!s_tx_on);
        break;
    case 's':
        App_Nvram_CalibSetBias(s_drive, s_final);
        LOG_INFO("Saved to NVRAM: drive: %u, final: %u", s_drive, s_final);
    case 'p':
        break;
    default:
        LOG_WARN("unknown key '%c'", c);
        break;
    }
}

static void bias_help(void) {
    LOG_INFO("bias: d drive, f final, +/- %d, 0 off, t tx, s save, q exit",
             BIAS_STEP);
}

static void bias_tick(void) {
    if (s_active) {
        bias_report();
    }
}

static const AppConsoleMode s_mode = {
    .key    = 'o',
    .name   = "bias",
    .enter  = bias_enter,
    .exit   = bias_exit,
    .on_key = bias_on_key,
    .report = bias_report,
    .help   = bias_help,
    .tick   = bias_tick,
};

int App_CalBias_Init(void) {
    return App_Console_RegisterMode(&s_mode);
}
