#include "app_cal.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_alc.h"
#include "app_console.h"
#include "app_protect.h"
#include "app_radio.h"
#include "app_radio_state.h"
#include "bsp_log.h"

#define LOG_TAG "app.cal.alc"

static bool s_active;

static void alc_report(void) {
    float level = App_Alc_AdjustLevelDb(0.0f);
    float corr  = App_Alc_AdjustPowerCorrDb(0.0f);

    LOG_INFO("level=%.1f dB corr=%.2f dB txpwr=%.1f W freq=%u",
             (double)level, (double)corr, (double)App_RadioState_GetTxPwr(),
             (unsigned)App_RadioState_GetTxFreq());
}

static void alc_enter(void) {
    s_active = true;
    alc_report();
}

static void alc_exit(void) {
    s_active = false;
}

static void alc_on_key(char c) {
    switch (c) {
    case '+':
        (void)App_Alc_AdjustLevelDb(0.5f);
        break;
    case '-':
        (void)App_Alc_AdjustLevelDb(-0.5f);
        break;
    case '0':
        (void)App_Alc_ResetLevelDb();
        break;
    case '>':
        (void)App_Alc_AdjustPowerCorrDb(0.5f);
        break;
    case '<':
        (void)App_Alc_AdjustPowerCorrDb(-0.5f);
        break;
    case '=':
        (void)App_Alc_ResetPowerCorrDb();
        break;
    case 'p':
        break;
    default:
        LOG_WARN("unknown key '%c'", c);
        break;
    }
}

static void alc_help(void) {
    LOG_INFO("alc: +/- 0.5 dB level, 0 level default, >/< 0.5 dB power corr, "
             "= corr default, p print, q exit");
}

static void alc_tick(void) {
    AppAlcSnapshot     alc;
    AppProtectSnapshot prot;
    int32_t            t;
    uint32_t           t_abs;
    float              corr;

    if (!s_active || App_Radio_GetRadioMode() != APP_RADIO_MODE_TX) {
        return;
    }

    App_Alc_GetSnapshot(&alc);
    App_Protect_GetSnapshot(&prot);
    corr = App_Alc_AdjustPowerCorrDb(0.0f);
    if (!prot.pa_valid) {
        LOG_INFO("tx level=%.1f dB corr=%.2f dB k=%.1f gslow=%.2f fwd=%.1f W "
                 "lim=%u mW temp=--",
                 (double)App_Alc_AdjustLevelDb(0.0f), (double)corr,
                 (double)alc.k_est, (double)alc.g_slow, (double)alc.fwd_avg_w,
                 (unsigned)alc.limit_mw);
        return;
    }

    t     = prot.pa_temp_mc;
    t_abs = (t < 0) ? (uint32_t)(-t) : (uint32_t)t;
    LOG_INFO("tx level=%.1f dB corr=%.2f dB k=%.1f gslow=%.2f fwd=%.1f W "
             "lim=%u mW temp=%s%u.%01uC",
             (double)App_Alc_AdjustLevelDb(0.0f), (double)corr,
             (double)alc.k_est, (double)alc.g_slow, (double)alc.fwd_avg_w,
             (unsigned)alc.limit_mw, (t < 0) ? "-" : "", (unsigned)(t_abs / 1000u),
             (unsigned)((t_abs % 1000u) / 100u));
}

static const AppConsoleMode s_mode = {
    .key    = 'a',
    .name   = "alc",
    .enter  = alc_enter,
    .exit   = alc_exit,
    .on_key = alc_on_key,
    .report = alc_report,
    .help   = alc_help,
    .tick   = alc_tick,
};

int App_CalAlc_Init(void) {
    return App_Console_RegisterMode(&s_mode);
}
