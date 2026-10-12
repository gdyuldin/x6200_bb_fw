#include "app_housekeeping.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_radio.h"
#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_iwdg.h"
#include "bsp_log.h"
#include "drv_fpga.h"

#define LOG_TAG "app.housekeeping"

#define APP_FPGA_TICK_TOLERANCE_PCT 5u

static bool s_otr_seen;

/*
 * Validate the FPGA tick arrival rate (~48 kHz) and keep the NCO frame alive.
 * The counters are read-and-cleared here in the main loop; a large deviation
 * points at lost or doubled PE10 ticks (overrun in the handler).
 */
static void fpga_validation_1s(void) {
    uint32_t ticks   = Drv_Fpga_TickCount();
    uint32_t samples = Drv_Fpga_SampleCount();
    uint32_t aic     = App_Radio_GetAicSamples();
    uint32_t m       = Drv_Fpga_GetSamplesPerTick();
    uint32_t tol     = BOARD_AUDIO_SAMPLE_RATE_HZ * APP_FPGA_TICK_TOLERANCE_PCT / 100u;

    if (ticks + tol < BOARD_AUDIO_SAMPLE_RATE_HZ || ticks > BOARD_AUDIO_SAMPLE_RATE_HZ + tol) {
        LOG_WARN("fpga tick rate %u/s, expected %u", (unsigned)ticks,
                 (unsigned)BOARD_AUDIO_SAMPLE_RATE_HZ);
    }

    /* AIC frames and FPGA ticks share the audio clock, so samples/M is the
     * tick-equivalent of the RX stream. Only meaningful while it streams. */
    if (m != 0u && samples >= m) {
        uint32_t expected = samples / m;
        if (aic + tol < expected || aic > expected + tol) {
            LOG_WARN("aic/fpga clock desync: aic %u, fpga %u", (unsigned)aic,
                     (unsigned)expected);
        }
    }

    /* Warn on the rising edge only, so sustained clipping is not spammed. */
    DrvFpgaStatus st;
    Drv_Fpga_GetStatus(&st);
    bool otr = st.adc_otr || st.dac_otr;
    if (otr && !s_otr_seen) {
        LOG_WARN("fpga over-range: adc %u dac %u", (unsigned)st.adc_otr,
                 (unsigned)st.dac_otr);
    }
    s_otr_seen = otr;
}

static void housekeeping_1s(void) {
    Bsp_Iwdg_Kick();
    fpga_validation_1s();
    /* TX staging overflow is accumulated in the I2S DMA ISR; report and reset
     * it here at most once per second. */
    App_Radio_LogTxDrops();
}

int App_Housekeeping_Init(void) {
    return App_Scheduler_Add(housekeeping_1s, BOARD_HOUSEKEEPING_PERIOD_MS);
}
