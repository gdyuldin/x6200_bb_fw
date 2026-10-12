#include "app_cal.h"

#include <stdbool.h>

#include "app_console.h"
#include "app_radio.h"
#include "bsp_log.h"
#include "sdr_compressor.h"

#define LOG_TAG "app.cal.comp"

static bool s_active;

static void comp_report(void) {
    SdrCompressorStats st;

    Sdr_Compressor_GetStats(&st);
    LOG_INFO("th=%.1f dB on=%u", (double)st.threshold, (unsigned)st.enabled);
}

static void comp_enter(void) {
    s_active = true;
    comp_report();
}

static void comp_exit(void) {
    s_active = false;
}

static void comp_on_key(char c) {
    SdrCompressorStats st;

    switch (c) {
    case '+':
        Sdr_Compressor_GetStats(&st);
        Sdr_Compressor_SetThreshold(st.threshold + 1.0f);
        break;
    case '-':
        Sdr_Compressor_GetStats(&st);
        Sdr_Compressor_SetThreshold(st.threshold - 1.0f);
        break;
    case 'e':
        Sdr_Compressor_GetStats(&st);
        Sdr_Compressor_Enable(!st.enabled);
        break;
    case '0':
        Sdr_Compressor_ResetThreshold();
        break;
    case 'p':
        break;
    default:
        LOG_WARN("unknown key '%c'", c);
        break;
    }
}

static void comp_help(void) {
    LOG_INFO("comp: +/- 1 dB, e on/off, 0 default, p print, q exit");
}

static void comp_tick(void) {
    SdrCompressorStats st;

    if (!s_active || App_Radio_GetRadioMode() != APP_RADIO_MODE_TX) {
        return;
    }

    Sdr_Compressor_GetStats(&st);
    LOG_INFO("tx rms in=%.1f dB out=%.1f dB gain=%.1f dB th=%.1f dB",
             (double)st.rms_in_db, (double)st.rms_out_db,
             (double)st.applied_db, (double)st.threshold);
}

static const AppConsoleMode s_mode = {
    .key    = 'c',
    .name   = "comp",
    .enter  = comp_enter,
    .exit   = comp_exit,
    .on_key = comp_on_key,
    .report = comp_report,
    .help   = comp_help,
    .tick   = comp_tick,
};

int App_CalComp_Init(void) {
    return App_Console_RegisterMode(&s_mode);
}
