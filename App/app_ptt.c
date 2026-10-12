#include "app_ptt.h"

#include "app_alc.h"
#include "app_atu.h"
#include "app_nvram.h"
#include "app_radio.h"
#include "app_radio_state.h"
#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_board.h"
#include "bsp_exti.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "main.h"
#include "sdr_tx.h"

#define LOG_TAG "app.ptt"

/* hptt is written from the EXTI ISR, the other flags from the main loop. */
static AppPttStatus s_status;
static AppPttSource s_source;
static volatile bool s_swr_scan_out;

static void set_blocked(bool on) {
    uint32_t key = Bsp_Irq_SaveLow();
    s_status.blocked = on;
    Bsp_Irq_RestoreLow(key);
}

static bool any_source_active_locked(void) {
    return s_status.mtx || s_status.iptt || s_status.civptt || s_status.hptt ||
           s_status.cwkey || s_status.calib || s_status.atu_tune ||
           s_status.swr_scan;
}

/* The host mtx flag keys TX only for non-CW modulations. In CW the host must
 * use civptt or the key, so mtx is ignored entirely and must not hold TX on. */
static bool modulation_is_cw(SdrModulation mode) {
    return mode == SDR_MOD_CW || mode == SDR_MOD_CWR;
}

static void set_host_flag(bool *field, bool on, AppPttSource source) {
    bool changed;
    bool was_active;
    AppPttSource cur;
    uint32_t key = Bsp_Irq_SaveLow();

    was_active = any_source_active_locked();
    changed = *field != on;
    *field = on;
    if (on && !was_active) {
        s_source = source;
    }
    if (!any_source_active_locked()) {
        s_source = APP_PTT_SOURCE_NONE;
    }
    cur = s_source;
    Bsp_Irq_RestoreLow(key);

    App_Radio_SetTxSource(cur);

    if (changed) {
        App_Scheduler_Notify(APP_SCHED_EV_PTT);
    }
}

/* The scan carrier and its forced ALC setpoint follow the actual TX winner, not
 * the host bit alone: a scan request cannot take over an already keyed TX. */
static void update_scan_output(void) {
    bool on;
    uint32_t key = Bsp_Irq_SaveLow();

    on = (s_source == APP_PTT_SOURCE_SWR_SCAN) && s_status.swr_scan;
    if (on == s_swr_scan_out) {
        Bsp_Irq_RestoreLow(key);
        return;
    }
    s_swr_scan_out = on;
    Bsp_Irq_RestoreLow(key);

    App_Radio_SetTuneCarrier(on);
    App_Alc_SetSwrScanActive(on);
}

/* No logs, no delays and no hardware access in ISR context. */
static void ptt_exti_cb(void *ctx) {
    bool on;
    bool was_active;
    AppPttSource cur;
    uint32_t key;

    (void)ctx;
    on = Bsp_Board_IsExtMicPttPressed();
    key = Bsp_Irq_SaveLow();
    was_active = any_source_active_locked();
    s_status.hptt = on;
    if (on && !was_active) {
        s_source = APP_PTT_SOURCE_HPTT;
    }
    if (!any_source_active_locked()) {
        s_source = APP_PTT_SOURCE_NONE;
    }
    cur = s_source;
    Bsp_Irq_RestoreLow(key);
    App_Radio_SetTxSource(cur);
    App_Scheduler_Notify(APP_SCHED_EV_PTT);
}

/* ATU tune, SWR scan and calibration key the PA without the modulator, so they
 * are allowed in any modulation. Every other source needs a TX modulator. */
static bool tx_mode_allowed(const AppPttStatus *st, bool scan_active) {
    if (st->atu_tune || st->calib || scan_active) {
        return true;
    }
    return Sdr_Tx_ModeSupported(App_RadioState_GetTxMode());
}

/* Split, RIT and XIT are already folded into GetTxFreq(). The SWR scan is an
 * antenna measurement over the whole 1.5..54 MHz range, so it bypasses the
 * NVRAM transmit ranges and keeps only the absolute upper limit. */
static bool tx_freq_allowed(bool scan) {
    uint32_t freq = App_RadioState_GetTxFreq();

    if (freq == 0u || freq > BOARD_TX_FREQ_MAX_HZ) {
        return false;
    }
    if (scan) {
        return true;
    }
    return App_Nvram_TxAllowed(freq);
}

static void app_ptt_evaluate(void) {
    AppPttStatus st;
    uint32_t key = Bsp_Irq_SaveLow();
    st = s_status;
    Bsp_Irq_RestoreLow(key);

    /* A tune owns the PA until it finishes; the scan must not preempt it before
     * the ATU has set its own TX source. */
    bool scan_active  = App_Ptt_IsSwrScanActive();
    bool scan_request = scan_active && !App_Atu_IsTuning();
    bool request = (st.mtx && !modulation_is_cw(App_RadioState_GetTxMode())) ||
                   st.iptt || st.civptt || st.hptt || st.cwkey ||
                   st.calib || st.atu_tune || scan_request;
    AppRadioMode mode = App_Radio_GetRadioMode();

    if (!request) {
        set_blocked(false);
        if (mode == APP_RADIO_MODE_TX) {
            (void)App_Radio_SetRadioMode(APP_RADIO_MODE_RX);
            LOG_DEBUG("tx off (no ptt sources)");
        }
        return;
    }

    /* Voice recording owns the RX audio path; no source may key TX while it is
     * set. The state layer re-notifies PTT when the flag clears, so a still-held
     * request is evaluated again then. An already running TX is not interrupted
     * here: the no-request path above still ends it normally. */
    if (App_RadioState_GetVoiceRec()) {
        return;
    }

    if (st.blocked) {
        return; /* wait until every source is released */
    }

    if (mode == APP_RADIO_MODE_RX) {
        if (!tx_mode_allowed(&st, scan_active)) {
            set_blocked(true);
            LOG_DEBUG("tx refused, mode %u has no modulator",
                      (unsigned)App_RadioState_GetTxMode());
        } else if (!tx_freq_allowed(scan_active)) {
            set_blocked(true);
            LOG_WARN("tx refused, frequency not allowed");
        } else if (App_Radio_SetRadioMode(APP_RADIO_MODE_TX) != 0) {
            set_blocked(true);
            LOG_WARN("tx refused, latched until sources release");
        } else {
            LOG_DEBUG("tx on (mtx %u iptt %u civptt %u hptt %u calh %u)",
                      (unsigned)st.mtx, (unsigned)st.iptt,
                      (unsigned)st.civptt, (unsigned)st.hptt,
                      (unsigned)st.calib);
        }
    }
}

int App_Ptt_Init(void) {
    AppPttSource cur;
    uint32_t key = Bsp_Irq_SaveLow();
    s_status.hptt = Bsp_Board_IsExtMicPttPressed();
    if (s_status.hptt) {
        s_source = APP_PTT_SOURCE_HPTT;
    }
    cur = s_source;
    Bsp_Irq_RestoreLow(key);
    App_Radio_SetTxSource(cur);

    if (Bsp_Exti_Register(EXT_MIC_PTT_Pin, ptt_exti_cb, 0) != 0) {
        LOG_WARN("ext mic ptt exti register failed");
        return -1;
    }
    if (App_Scheduler_AddEvent(app_ptt_evaluate, APP_SCHED_EV_PTT) != 0) {
        LOG_WARN("ptt event add failed");
        (void)Bsp_Exti_Unregister(EXT_MIC_PTT_Pin);
        return -1;
    }

    if (s_status.hptt) {
        App_Scheduler_Notify(APP_SCHED_EV_PTT);
    }
    LOG_INFO("ptt arbiter ready");
    return 0;
}

void App_Ptt_SetMtx(bool on) {
    set_host_flag(&s_status.mtx, on, APP_PTT_SOURCE_MTX);
}

void App_Ptt_SetIptt(bool on) {
    set_host_flag(&s_status.iptt, on, APP_PTT_SOURCE_IPTT);
}

void App_Ptt_SetCivPtt(bool on) {
    set_host_flag(&s_status.civptt, on, APP_PTT_SOURCE_CIVPTT);
}

void App_Ptt_SetCwKey(bool on) {
    set_host_flag(&s_status.cwkey, on, APP_PTT_SOURCE_CWKEY);
}

void App_Ptt_SetAtuTune(bool on) {
    set_host_flag(&s_status.atu_tune, on, APP_PTT_SOURCE_ATU_TUNE);
}

void App_Ptt_SetSwrScan(bool on) {
    set_host_flag(&s_status.swr_scan, on, APP_PTT_SOURCE_SWR_SCAN);
    update_scan_output();
}

bool App_Ptt_IsSwrScanActive(void) {
    return s_swr_scan_out;
}

void App_Ptt_SetCalibrate(bool on) {
    set_host_flag(&s_status.calib, on, APP_PTT_SOURCE_CALIB);
}

AppPttSource App_Ptt_GetSource(void) {
    uint32_t key = Bsp_Irq_SaveLow();
    AppPttSource source = s_source;
    Bsp_Irq_RestoreLow(key);
    return source;
}

/* Called from app_protect. Only a source that was active at that moment has to
 * stay blocked until it is released. The RX transition lives here so callers
 * do not need to know or check the radio mode. */
void App_Ptt_ForceRx(void) {
    uint32_t key = Bsp_Irq_SaveLow();
    if (s_status.mtx || s_status.iptt || s_status.civptt || s_status.hptt ||
        s_status.calib || s_status.atu_tune || s_status.swr_scan) {
        s_status.blocked = true;
    }
    Bsp_Irq_RestoreLow(key);

    if (App_Radio_GetRadioMode() == APP_RADIO_MODE_TX) {
        (void)App_Radio_SetRadioMode(APP_RADIO_MODE_RX);
    }
}

void App_Ptt_GetStatus(AppPttStatus *out) {
    if (out == 0) {
        return;
    }
    uint32_t key = Bsp_Irq_SaveLow();
    *out = s_status;
    Bsp_Irq_RestoreLow(key);
}
