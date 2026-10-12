#include "app_radio_state.h"

#include <string.h>

#include "app_alc.h"
#include "app_atu.h"
#include "app_event.h"
#include "app_fpga.h"
#include "app_radio.h"
#include "app_radio_rfpath.h"
#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_log.h"
#include "drv_codec.h"
#include "sdr_agc.h"
#include "sdr_config.h"
#include "sdr_rx.h"
#include "sdr_cw.h"
#include "sdr_squelch.h"
#include "sdr_spectrum.h"

#define LOG_TAG "app.radio_state"

typedef struct {
    uint32_t      freq_hz[APP_RADIO_VFO_COUNT];
    int32_t      rx_filter_low_hz;
    int32_t      rx_filter_high_hz;
    uint8_t       active_vfo;
    bool          split;
    bool          swr_scan;
    bool          voice_rec;
    bool          spmode;
    bool          att[APP_RADIO_VFO_COUNT];
    bool          pre[APP_RADIO_VFO_COUNT];
    uint8_t       rfg;
    float         txpwr;
    SdrModulation mode[APP_RADIO_VFO_COUNT];
    SdrModulation tx_mode;
    uint8_t       agc_mode[APP_RADIO_VFO_COUNT];
    uint8_t       fftdec;
    uint8_t       fftzoomcw;
    bool          sql_en;
    uint8_t       sql_thr;
    uint8_t       sql_thr_fm;
    bool          tuner_on;
    uint8_t       monitor_level;
    float         monitor_k;
    uint32_t      cw_tone_hz;
    uint32_t      cw_speed_wpm;
    SdrCwKeyMode  cw_key_mode;
    SdrCwIambicMode cw_iambic_mode;
    uint8_t       cw_vol;
    bool          cw_train;
    uint32_t      cw_qsk_time_ms;
    uint32_t      cw_dash_ratio_x10;
    uint8_t       rxvol;
    int8_t        hpr_gain_db;
    bool          hpr_mute;
    float         left_dac_scale;
    float         voice_rec_right_dac_scale;
    uint8_t       mic_sel;
    uint8_t       ling;
    uint8_t       loutg;
    uint8_t       imicg;
    uint8_t       hmicg;
    int32_t       rit_hz;
    int32_t       xit_hz;
    bool          configured;
} AppRadioState;

/* s_state is the committed state and mirrors what the hardware runs; s_edit is
 * the working copy that accumulates staged edits. It may survive several host
 * blocks until the staged set is valid and gets committed. */
static AppRadioState s_state;
static AppRadioState s_edit;
static bool          s_edit_pending;

#define APP_RADIO_STATE_RXVOL_MAX 100u
/* Below this point rxvol attenuates the HPR driver; above it the left DAC is
 * boosted digitally instead. */
#define APP_RADIO_STATE_RXVOL_ATT_MAX 70u
/* RIT/XIT offsets are limited to +-2000 Hz by the host protocol. */
#define APP_RADIO_STATE_OFFSET_MAX_HZ 2000
/* Left DAC digital level at maximum HPR gain, and the fixed right DAC (USB /
 * LOR) level at normal RX. */
#define APP_RADIO_STATE_LEFT_DB  (-41.0f)
#define APP_RADIO_STATE_RIGHT_DB (-28.0f)

/* Left-DAC multipliers for the 0 / 0.5 dB fine step (also the voice_rec
 * right-DAC scale) and the fixed normal-RX right-DAC multiplier. */
static float s_left_scale[2];
static float s_right_scale;

static float monitor_k_from_level(uint8_t level) {
    float db = ((float)level - 30.0f) * 0.5f;
    return Sdr_Db2Lin(db);
}

/*
 * Split the host RX volume (0..100, 0.5 dB per step) into the per-output HPR
 * driver gain and the DAC multipliers. Up to the split point A = (70 - rxvol)
 * * 0.5 dB is the wanted attenuation: the 1 dB driver step takes ceil(A) and
 * the remaining 0 / 0.5 dB goes into the left-DAC scale. Above the split point
 * the driver stays at maximum and the left DAC gets the boost for a quiet
 * band. rxvol 0 mutes HPR (the gain lands on DRV_CODEC_OUT_GAIN_MIN, which the
 * driver replaces when muting).
 */
static void rxvol_apply(AppRadioState *st, uint8_t rxvol) {
    unsigned atten_half = 0u;
    unsigned atten_db   = 0u;
    unsigned fine_half  = 0u;
    unsigned boost_half = 0u;

    if (rxvol < APP_RADIO_STATE_RXVOL_ATT_MAX) {
        atten_half = APP_RADIO_STATE_RXVOL_ATT_MAX - rxvol; /* 0.5 dB steps */
        atten_db   = (atten_half + 1u) / 2u;                /* ceil() */
        fine_half  = 2u * atten_db - atten_half;            /* 0 or 1 */
    } else {
        boost_half = rxvol - APP_RADIO_STATE_RXVOL_ATT_MAX;
    }

    st->rxvol       = rxvol;
    st->hpr_gain_db = (int8_t)(DRV_CODEC_OUT_GAIN_MAX - (int)atten_db);
    st->hpr_mute    = (rxvol == 0u);
    st->left_dac_scale = Sdr_Db2Lin(APP_RADIO_STATE_LEFT_DB +
                                    0.5f * (float)(fine_half + boost_half));
    /* In voice_rec HPR comes from the right DAC, which gets the same fine step
     * but no quiet-band boost. */
    st->voice_rec_right_dac_scale = s_left_scale[fine_half];
}

/* The event bus only carries real changes, so the initial committed snapshot
 * has to be published explicitly or the subscribers stay on their defaults.
 * Sdr_Rx_SetConfig also applies the RX configuration and emits SdrRxCfg. */
static void publish_initial_blocks(void) {
    SdrRxCfg rx = {
        .rx_mode        = s_state.mode[s_state.active_vfo],
        .filter_low_hz  = s_state.rx_filter_low_hz,
        .filter_high_hz = s_state.rx_filter_high_hz,
        .fftdec         = s_state.fftdec,
    };
    SdrTxCfg     tx   = { .tx_mode = s_state.tx_mode };
    SdrKeyToneCfg tone = { .key_tone_hz = s_state.cw_tone_hz };

    Sdr_Rx_SetConfig(&rx);
    App_Event_NotifyTxCfg(&tx);
    App_Event_NotifyKeyTone(&tone);
}

int App_RadioState_Init(void) {
    memset(&s_state, 0, sizeof(s_state));
    memset(&s_edit, 0, sizeof(s_edit));
    s_left_scale[0] = Sdr_Db2Lin(APP_RADIO_STATE_LEFT_DB);
    s_left_scale[1] = Sdr_Db2Lin(APP_RADIO_STATE_LEFT_DB + 0.5f);
    s_right_scale   = Sdr_Db2Lin(APP_RADIO_STATE_RIGHT_DB);
    /* Boot volume is 0: HPR muted, host and USB still driven. */
    rxvol_apply(&s_state, 0u);
    s_state.freq_hz[APP_RADIO_VFO_A] = BOARD_DEFAULT_FREQ_HZ;
    s_state.freq_hz[APP_RADIO_VFO_B] = BOARD_DEFAULT_FREQ_HZ;
    s_state.rx_filter_low_hz  = 300u;
    s_state.rx_filter_high_hz = 2700u;
    /* The requested value is what the driver will commit on its next PE10 tick;
     * it may differ from the still-active one during early boot. */
    s_state.fftdec = Drv_Fpga_GetRequestedFftdec();
    s_edit.fftdec  = s_state.fftdec;
    s_state.monitor_level = 0u;
    s_state.monitor_k     = monitor_k_from_level(s_state.monitor_level);
    s_edit.monitor_level  = s_state.monitor_level;
    s_edit.monitor_k      = s_state.monitor_k;
    s_state.cw_tone_hz = 600u;
    s_edit.cw_tone_hz  = 600u;
    s_state.cw_speed_wpm    = 20u;
    s_state.cw_key_mode     = SDR_CW_KEY_DUAL_RIGHT;
    s_state.cw_iambic_mode  = SDR_CW_IAMBIC_B;
    s_state.cw_vol          = 30u;
    s_state.cw_train        = false;
    s_state.cw_qsk_time_ms    = 100u;
    s_state.cw_dash_ratio_x10 = 30u;
    /* Split is off at boot, so TX follows the active VFO. */
    s_state.tx_mode = s_state.mode[s_state.active_vfo];
    s_edit.tx_mode  = s_edit.mode[s_edit.active_vfo];
    s_edit_pending = false;

    /* Both VFOs start on the same frequency; push both final branch values so
     * the driver's tuning sets are valid before the first split/TX frame. */
    App_Fpga_SetRxFrequency(BOARD_DEFAULT_FREQ_HZ);
    App_Fpga_SetTxFrequency(BOARD_DEFAULT_FREQ_HZ);

    /* Seed the spectrum path with the same initial state. Sdr_Spectrum_Init()
     * runs before this function in app_main, so the setters are already valid. */
    Sdr_Spectrum_SetModulation(s_state.mode[s_state.active_vfo]);
    Sdr_Spectrum_SetFftdec(s_state.fftdec);
    Sdr_Spectrum_SetFftzoomcw(s_state.fftzoomcw);
    Sdr_Spectrum_SetFilter(s_state.rx_filter_low_hz, s_state.rx_filter_high_hz);
    App_Alc_SetPowerLimit(s_state.txpwr);
    App_Alc_NotifyFrequency(BOARD_DEFAULT_FREQ_HZ);
    Sdr_Squelch_Configure(s_state.sql_en, s_state.sql_thr, s_state.sql_thr_fm);
    publish_initial_blocks();
    return 0;
}

/* Stage this host block's changes. While a staged set is still pending (a
 * previous block left edits uncommitted, e.g. an ATU tune in progress) the
 * working copy is kept so the new changes merge into it. */
void App_RadioState_Begin(void) {
    if (!s_edit_pending) {
        s_edit = s_state;
    }
    s_edit_pending = true;
}

/* Final frequency of a branch: RX is the active VFO plus RIT, TX follows the
 * split VFO (except during SWR scan, which always uses the active VFO) and adds
 * XIT. A non-positive result means "unset". */
static uint32_t branch_freq(const AppRadioState *st, bool tx_active) {
    int64_t freq = st->freq_hz[st->active_vfo];

    if (tx_active) {
        if (st->split && !st->swr_scan) {
            freq = st->freq_hz[1u - st->active_vfo];
        }
        freq += st->xit_hz;
    } else {
        freq += st->rit_hz;
    }
    return (freq > 0) ? (uint32_t)freq : 0u;
}

/* Retune the analog front end when the branch frequency moves. If the band
 * changes during TX, mute first: the relay sequence runs asynchronously over
 * ~120 ms. K depends on the band, so the ALC rescales it, and the spectrum
 * holds its noise estimate while the front end and passband settle. */
static void apply_branch_freq(uint32_t old_freq, uint32_t new_freq,
                              bool tx_active) {
    if (new_freq == old_freq) {
        return;
    }
    if (tx_active && App_RadioRfPath_LpfWillChange(new_freq)) {
        App_Fpga_TxMute(120u);
    }
    App_RadioRfPath_SetFreq(new_freq);
    App_Alc_NotifyFrequency(new_freq);
    Sdr_Spectrum_NotifyRetune();
}

void App_RadioState_Apply(void) {
    AppRadioState old;
    uint32_t      old_rx_freq;
    uint32_t      old_tx_freq;
    uint32_t      new_rx_freq;
    uint32_t      new_tx_freq;
    bool          tx_active;
    SdrModulation old_rx_mode;
    SdrModulation new_rx_mode;
    SdrModulation old_tx_mode;
    SdrModulation new_tx_mode;

    if (!s_edit_pending) {
        return;
    }

    /* Hold the commit while the ATU is tuning: the frequency/filter must not
     * move (the LPF relays and the tune network would fight). The staged edits
     * stay pending and are applied on the next host block after tuning ends. */
    if (App_Atu_IsTuning()) {
        return;
    }

    /* The active-VFO mode and fftdec arrive in separate host blocks and must
     * form a resample-usable pair. Coerce fftdec down to what the mode supports
     * so unrelated staged edits still commit; only a mode the DSP layer has no
     * plan for holds the edit. */
    if (!Sdr_Rx_ClampFftdec(s_edit.mode[s_edit.active_vfo], &s_edit.fftdec)) {
        LOG_ERROR("no resample plan for mode %u, edit held",
                  (unsigned)s_edit.mode[s_edit.active_vfo]);
        return;
    }

    old = s_state;
    old_rx_freq = branch_freq(&old, false);
    old_tx_freq = branch_freq(&old, true);

    tx_active   = App_Radio_GetRadioMode() == APP_RADIO_MODE_TX;
    old_rx_mode = old.mode[old.active_vfo];
    new_rx_mode = s_edit.mode[s_edit.active_vfo];
    old_tx_mode = old.split ? old.mode[1u - old.active_vfo] : old_rx_mode;
    new_tx_mode = s_edit.split ? s_edit.mode[1u - s_edit.active_vfo]
                               : new_rx_mode;
    /* Fold the split rule into the committed TX mode so App_Event carries the
     * same value GetTxMode will return. */
    s_edit.tx_mode = new_tx_mode;

    /* Touch the RF filters only when the active branch frequency changes.
     * While split-TX an active-VFO frequency change is RX-only. */
    new_rx_freq = branch_freq(&s_edit, false);
    new_tx_freq = branch_freq(&s_edit, true);
    apply_branch_freq(tx_active ? old_tx_freq : old_rx_freq,
                      tx_active ? new_tx_freq : new_rx_freq, tx_active);

    /* att/pre live per VFO but only the active VFO drives the shared hardware
     * line, so switching VFO re-applies the value of the new active VFO. rfg is
     * global. */
    if (s_edit.rfg != old.rfg) {
        LOG_DEBUG("rfg %u", (unsigned)s_edit.rfg);
        App_RadioRfPath_SetRfg(s_edit.rfg);
    }
    if (s_edit.txpwr != old.txpwr) {
        LOG_INFO("txpwr %.1f W", s_edit.txpwr);
        App_Alc_SetPowerLimit(s_edit.txpwr);
    }
    if (s_edit.att[s_edit.active_vfo] != old.att[old.active_vfo]) {
        LOG_DEBUG("att %u", (unsigned)s_edit.att[s_edit.active_vfo]);
        App_RadioRfPath_SetAtt(s_edit.att[s_edit.active_vfo]);
    }
    if (s_edit.pre[s_edit.active_vfo] != old.pre[old.active_vfo]) {
        LOG_DEBUG("pre %u", (unsigned)s_edit.pre[s_edit.active_vfo]);
        App_RadioRfPath_SetPre(s_edit.pre[s_edit.active_vfo]);
    }

    if (s_edit.fftdec != old.fftdec) {
        LOG_DEBUG("fftdec %u", (unsigned)s_edit.fftdec);
        /* Request the hardware change; the matching DSP config is pushed after
         * the commit and staged by sdr_rx until the raw generation changes. */
        App_Fpga_SetSampleRate(s_edit.fftdec);
    }

    if (s_edit.fftzoomcw != old.fftzoomcw) {
        LOG_DEBUG("fftzoomcw %u", (unsigned)s_edit.fftzoomcw);
    }

    if (s_edit.agc_mode[s_edit.active_vfo] !=
        old.agc_mode[s_edit.active_vfo]) {
        /* agc_mode is a direct setter; the AGC takes rx_mode from SdrRxCfg, so
         * a VFO switch alone never reconfigures it. */
        Sdr_AGC_SetMode((SdrAgcMode)s_edit.agc_mode[s_edit.active_vfo]);
    }

    if (s_edit.spmode != old.spmode) {
        LOG_DEBUG("spmode %u", (unsigned)s_edit.spmode);
        App_Radio_UpdateSpeaker();
    }

    s_state        = s_edit;
    s_edit_pending = false;

    if (new_rx_freq != old_rx_freq) {
        LOG_DEBUG("rx freq %u", (unsigned)new_rx_freq);
        App_Fpga_SetRxFrequency(new_rx_freq);
    }
    if (new_tx_freq != old_tx_freq) {
        LOG_DEBUG("tx freq %u", (unsigned)new_tx_freq);
        App_Fpga_SetTxFrequency(new_tx_freq);
    }

    /* Publish only real committed-state transitions; a held edit returned
     * earlier and never reaches this point. The RX block is published
     * by sdr_rx, which owns the resample generation; it is skipped when a VFO
     * switch lands on the same effective mode and filter/rate are unchanged. */
    if (new_rx_mode != old_rx_mode ||
        s_state.rx_filter_low_hz != old.rx_filter_low_hz ||
        s_state.rx_filter_high_hz != old.rx_filter_high_hz ||
        s_state.fftdec != old.fftdec) {
        SdrRxCfg rx = {
            .rx_mode        = new_rx_mode,
            .filter_low_hz  = s_state.rx_filter_low_hz,
            .filter_high_hz = s_state.rx_filter_high_hz,
            .fftdec         = s_state.fftdec,
        };

        Sdr_Rx_SetConfig(&rx);
    }
    if (new_tx_mode != old_tx_mode) {
        SdrTxCfg tx = { .tx_mode = new_tx_mode };

        App_Event_NotifyTxCfg(&tx);
        /* The host mtx flag is not a TX request in CW, so a TX mode change can
         * turn a held host request into a no-request (or back). */
        App_Scheduler_Notify(APP_SCHED_EV_PTT);
    }

    if (s_state.cw_tone_hz != old.cw_tone_hz) {
        SdrKeyToneCfg tone = { .key_tone_hz = s_state.cw_tone_hz };

        App_Event_NotifyKeyTone(&tone);
    }
    if (s_state.cw_speed_wpm != old.cw_speed_wpm) {
        Sdr_CW_SetKeySpeed(s_state.cw_speed_wpm);
    }
    if (s_state.cw_key_mode != old.cw_key_mode) {
        Sdr_CW_SetKeyMode(s_state.cw_key_mode);
    }
    if (s_state.cw_iambic_mode != old.cw_iambic_mode) {
        Sdr_CW_SetIambicMode(s_state.cw_iambic_mode);
    }
    if (s_state.cw_vol != old.cw_vol) {
        Sdr_CW_SetSidetoneVolume(s_state.cw_vol);
    }
    if (s_state.cw_train != old.cw_train) {
        Sdr_CW_SetTraining(s_state.cw_train);
    }
    if (s_state.cw_qsk_time_ms != old.cw_qsk_time_ms ||
        s_state.cw_dash_ratio_x10 != old.cw_dash_ratio_x10) {
        Sdr_CW_SetQskTimeAndRatio(s_state.cw_qsk_time_ms,
                                  s_state.cw_dash_ratio_x10);
    }

    /* The spectrum path mirrors the committed active-VFO configuration. Do not
     * change its modulation while transmitting: the TX modulator takes its
     * mode from the inactive VFO and the spectrum must stay on the RX mode. */
    if (App_Radio_GetRadioMode() != APP_RADIO_MODE_TX) {
        Sdr_Spectrum_SetModulation(s_state.mode[s_state.active_vfo]);
    }
    Sdr_Spectrum_SetFftdec(s_state.fftdec);
    Sdr_Spectrum_SetFftzoomcw(s_state.fftzoomcw);
    Sdr_Spectrum_SetFilter(s_state.rx_filter_low_hz, s_state.rx_filter_high_hz);

    if (s_state.rxvol != old.rxvol || s_state.voice_rec != old.voice_rec ||
        s_state.mic_sel != old.mic_sel ||
        s_state.ling != old.ling || s_state.loutg != old.loutg ||
        s_state.imicg != old.imicg || s_state.hmicg != old.hmicg) {
        App_Radio_ApplyAudioConfig();
    }

    /* The PTT decision is made in the main loop from the committed flag: a
     * pending PTT request must start TX when voice_rec is cleared and must be
     * ignored while it is set, even if the request edge happened earlier. */
    if (s_state.voice_rec != old.voice_rec) {
        App_Scheduler_Notify(APP_SCHED_EV_PTT);
    }

    if (s_state.sql_en != old.sql_en || s_state.sql_thr != old.sql_thr ||
        s_state.sql_thr_fm != old.sql_thr_fm) {
        Sdr_Squelch_Configure(s_state.sql_en, s_state.sql_thr,
                              s_state.sql_thr_fm);
    }

    /* Deferred bring-up: the radio stayed off until the host configuration
     * committed. On the !configured -> configured edge only, and only on the
     * successful commit path above, bring the analog path up on the accepted
     * frequency. publish_initial_blocks re-pushes the accepted config so DSP
     * subscribers see it even when it equals the boot defaults (the diff
     * publishes nothing then). */
    if (!old.configured && s_state.configured) {
        LOG_INFO("host configuration accepted, radio on");
        App_Radio_SetRadioMode(APP_RADIO_MODE_RX);
        publish_initial_blocks();
    }
}

void App_RadioState_SetConfigured(void) {
    if (!s_edit_pending) {
        LOG_ERROR("set configured outside staged edit");
        return;
    }
    s_edit.configured = true;
}

bool App_RadioState_IsConfigured(void) {
    return s_state.configured;
}

void App_RadioState_SetFreq(AppRadioVfo vfo, uint32_t freq_hz) {
    if (!s_edit_pending) {
        LOG_ERROR("set freq outside staged edit");
        return;
    }
    if (vfo >= APP_RADIO_VFO_COUNT || freq_hz == 0u) {
        return;
    }
    s_edit.freq_hz[vfo] = freq_hz;
}

void App_RadioState_SetActiveVfo(AppRadioVfo vfo) {
    if (!s_edit_pending) {
        LOG_ERROR("set vfo outside staged edit");
        return;
    }
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return;
    }
    s_edit.active_vfo = (uint8_t)vfo;
}

void App_RadioState_SetSplit(bool split) {
    if (!s_edit_pending) {
        LOG_ERROR("set split outside staged edit");
        return;
    }
    s_edit.split = split;
}

void App_RadioState_SetSwrScan(bool on) {
    if (!s_edit_pending) {
        LOG_ERROR("set swr scan outside staged edit");
        return;
    }
    s_edit.swr_scan = on;
}

bool App_RadioState_GetSwrScan(void) {
    return s_state.swr_scan;
}

void App_RadioState_SetVoiceRec(bool on) {
    if (!s_edit_pending) {
        LOG_ERROR("set voice_rec outside staged edit");
        return;
    }
    s_edit.voice_rec = on;
}

bool App_RadioState_GetVoiceRec(void) {
    return s_state.voice_rec;
}

void App_RadioState_SetSpmode(bool spmode) {
    if (!s_edit_pending) {
        LOG_ERROR("set spmode outside staged edit");
        return;
    }
    s_edit.spmode = spmode;
}

void App_RadioState_SetMode(AppRadioVfo vfo, SdrModulation mode) {
    if (!s_edit_pending) {
        LOG_ERROR("set mode outside staged edit");
        return;
    }
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return;
    }
    if ((unsigned)mode > (unsigned)SDR_MOD_WFM) {
        LOG_DEBUG("mode %u out of range, ignored", (unsigned)mode);
        return;
    }
    s_edit.mode[vfo] = mode;
}

void App_RadioState_SetAgcMode(AppRadioVfo vfo, uint32_t value) {
    SdrAgcMode mode;

    if (!s_edit_pending) {
        LOG_ERROR("set agc outside staged edit");
        return;
    }
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return;
    }

    if (value == 0u) {
        mode = SDR_AGC_OFF;
    } else if (value == 1u) {
        mode = SDR_AGC_SLOW;
    } else if (value == 2u) {
        mode = SDR_AGC_FAST;
    } else {
        mode = SDR_AGC_AUTO;
    }
    s_edit.agc_mode[vfo] = (uint8_t)mode;
}

void App_RadioState_SetFftdec(uint8_t fftdec) {
    if (!s_edit_pending) {
        LOG_ERROR("set fftdec outside staged edit");
        return;
    }
    if (fftdec > DRV_FPGA_FFTDEC_MAX) {
        fftdec = DRV_FPGA_FFTDEC_MAX;
    }
    s_edit.fftdec = fftdec;
}

void App_RadioState_SetFftzoomcw(uint8_t n) {
    if (!s_edit_pending) {
        LOG_ERROR("set fftzoomcw outside staged edit");
        return;
    }
    if (n > SDR_SPECTRUM_ZOOM_MAX_STAGES) {
        n = SDR_SPECTRUM_ZOOM_MAX_STAGES;
    }
    s_edit.fftzoomcw = n;
}

void App_RadioState_SetSquelch(bool enable, uint8_t threshold,
                               uint8_t threshold_fm) {
    if (!s_edit_pending) {
        LOG_ERROR("set squelch outside staged edit");
        return;
    }
    if (threshold > 100u) {
        threshold = 100u;
    }
    if (threshold_fm > 100u) {
        threshold_fm = 100u;
    }
    s_edit.sql_en     = enable;
    s_edit.sql_thr    = threshold;
    s_edit.sql_thr_fm = threshold_fm;
}

void App_RadioState_SetTunerOn(bool on) {
    if (!s_edit_pending) {
        LOG_ERROR("set tuner_on outside staged edit");
        return;
    }
    s_edit.tuner_on = on;
}

bool App_RadioState_GetTunerOn(void) {
    return s_state.tuner_on;
}

void App_RadioState_SetMonitorLevel(uint8_t level) {
    if (!s_edit_pending) {
        LOG_ERROR("set monitor level outside staged edit");
        return;
    }
    if (level > 100u) {
        level = 100u;
    }
    s_edit.monitor_level = level;
    s_edit.monitor_k     = monitor_k_from_level(level);
}

float App_RadioState_GetMonitorK(void) {
    return s_state.monitor_k;
}

uint8_t App_RadioState_GetMonitorLevel(void) {
    return s_state.monitor_level;
}

void App_RadioState_SetCwTone(uint32_t cw_tone_hz) {
    if (!s_edit_pending) {
        LOG_ERROR("set cwtone outside staged edit");
        return;
    }
    s_edit.cw_tone_hz = cw_tone_hz;
}

void App_RadioState_SetCwKeySpeed(uint32_t wpm) {
    if (!s_edit_pending) {
        LOG_ERROR("set cw speed outside staged edit");
        return;
    }
    if (wpm < SDR_CW_SPEED_WPM_MIN) {
        wpm = SDR_CW_SPEED_WPM_MIN;
    } else if (wpm > SDR_CW_SPEED_WPM_MAX) {
        wpm = SDR_CW_SPEED_WPM_MAX;
    }
    s_edit.cw_speed_wpm = wpm;
}

void App_RadioState_SetCwKeyMode(SdrCwKeyMode mode) {
    if (!s_edit_pending) {
        LOG_ERROR("set cw key mode outside staged edit");
        return;
    }
    if (mode != SDR_CW_KEY_STRAIGHT && mode != SDR_CW_KEY_DUAL_LEFT
        && mode != SDR_CW_KEY_DUAL_RIGHT) {
        return;
    }
    s_edit.cw_key_mode = mode;
}

void App_RadioState_SetCwIambicMode(SdrCwIambicMode mode) {
    if (!s_edit_pending) {
        LOG_ERROR("set cw iambic mode outside staged edit");
        return;
    }
    if (mode != SDR_CW_IAMBIC_A && mode != SDR_CW_IAMBIC_B) {
        return;
    }
    s_edit.cw_iambic_mode = mode;
}

void App_RadioState_SetCwVol(uint8_t cwvol) {
    if (!s_edit_pending) {
        LOG_ERROR("set cw volume outside staged edit");
        return;
    }
    if (cwvol > 63u) {
        cwvol = 63u;
    }
    s_edit.cw_vol = cwvol;
}

void App_RadioState_SetCwTrain(bool training) {
    if (!s_edit_pending) {
        LOG_ERROR("set cw training outside staged edit");
        return;
    }
    s_edit.cw_train = training;
}

void App_RadioState_SetCwQskTimeAndRatio(uint32_t qsk_ms,
                                         uint32_t dash_ratio_x10) {
    if (!s_edit_pending) {
        LOG_ERROR("set cw qsk/ratio outside staged edit");
        return;
    }
    if (qsk_ms > 1000u) {
        qsk_ms = 1000u;
    }
    if (dash_ratio_x10 < 25u) {
        dash_ratio_x10 = 25u;
    } else if (dash_ratio_x10 > 45u) {
        dash_ratio_x10 = 45u;
    }
    s_edit.cw_qsk_time_ms    = qsk_ms;
    s_edit.cw_dash_ratio_x10 = dash_ratio_x10;
}

uint32_t App_RadioState_GetRxFreq(void) {
    return branch_freq(&s_state, false);
}

uint32_t App_RadioState_GetTxFreq(void) {
    return branch_freq(&s_state, true);
}


AppRadioVfo App_RadioState_GetActiveVfo(void) {
    return (AppRadioVfo)s_state.active_vfo;
}

SdrModulation App_RadioState_GetMode(AppRadioVfo vfo) {
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return SDR_MOD_LSB;
    }
    return s_state.mode[vfo];
}

SdrModulation App_RadioState_GetRxMode(void) {
    return s_state.mode[s_state.active_vfo];
}

SdrModulation App_RadioState_GetTxMode(void) {
    return s_state.tx_mode;
}

uint8_t App_RadioState_GetFftdec(void) {
    return s_state.fftdec;
}

bool App_RadioState_IsSplit(void) {
    return s_state.split;
}

bool App_RadioState_GetSpmode(void) {
    return s_state.spmode;
}

void App_RadioState_SetAtt(AppRadioVfo vfo, bool on) {
    if (!s_edit_pending) {
        LOG_ERROR("set att outside staged edit");
        return;
    }
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return;
    }
    s_edit.att[vfo] = on;
}

void App_RadioState_SetPre(AppRadioVfo vfo, bool on) {
    if (!s_edit_pending) {
        LOG_ERROR("set pre outside staged edit");
        return;
    }
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return;
    }
    s_edit.pre[vfo] = on;
}

void App_RadioState_SetRfg(uint8_t rfg) {
    if (!s_edit_pending) {
        LOG_ERROR("set rfg outside staged edit");
        return;
    }
    if (rfg > APP_RADIO_RFPATH_RFG_MAX) {
        rfg = APP_RADIO_RFPATH_RFG_MAX;
    }
    s_edit.rfg = rfg;
}

void App_RadioState_SetTxPwr(float txpwr_w) {
    if (!s_edit_pending) {
        LOG_ERROR("set txpwr outside staged edit");
        return;
    }
    s_edit.txpwr = txpwr_w;
}

float App_RadioState_GetTxPwr(void) {
    return s_state.txpwr;
}

void App_RadioState_SetRxVol(uint8_t rxvol) {
    if (!s_edit_pending) {
        LOG_ERROR("set rxvol outside staged edit");
        return;
    }
    if (rxvol > APP_RADIO_STATE_RXVOL_MAX) {
        rxvol = APP_RADIO_STATE_RXVOL_MAX;
    }
    rxvol_apply(&s_edit, rxvol);
}

void App_RadioState_SetRxFilter(int32_t low_hz, int32_t high_hz) {
    if (!s_edit_pending) {
        LOG_ERROR("set rxfilter outside staged edit");
        return;
    }
    s_edit.rx_filter_low_hz  = low_hz;
    s_edit.rx_filter_high_hz = high_hz;
}

void App_RadioState_GetRxFilter(int32_t *low_hz, int32_t *high_hz) {
    if (low_hz != 0) {
        *low_hz = s_state.rx_filter_low_hz;
    }
    if (high_hz != 0) {
        *high_hz = s_state.rx_filter_high_hz;
    }
}

int32_t App_RadioState_GetRxFilterLowHz(void) {
    return s_state.rx_filter_low_hz;
}

int32_t App_RadioState_GetRxFilterHighHz(void) {
    return s_state.rx_filter_high_hz;
}

static int32_t clamp_offset(int32_t hz) {
    if (hz > APP_RADIO_STATE_OFFSET_MAX_HZ) {
        return APP_RADIO_STATE_OFFSET_MAX_HZ;
    }
    if (hz < -APP_RADIO_STATE_OFFSET_MAX_HZ) {
        return -APP_RADIO_STATE_OFFSET_MAX_HZ;
    }
    return hz;
}

void App_RadioState_SetRit(int32_t hz) {
    if (!s_edit_pending) {
        LOG_ERROR("set rit outside staged edit");
        return;
    }
    s_edit.rit_hz = clamp_offset(hz);
}

void App_RadioState_SetXit(int32_t hz) {
    if (!s_edit_pending) {
        LOG_ERROR("set xit outside staged edit");
        return;
    }
    s_edit.xit_hz = clamp_offset(hz);
}

bool App_RadioState_GetAtt(AppRadioVfo vfo) {
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return false;
    }
    return s_state.att[vfo];
}

bool App_RadioState_GetPre(AppRadioVfo vfo) {
    if (vfo >= APP_RADIO_VFO_COUNT) {
        return false;
    }
    return s_state.pre[vfo];
}

uint8_t App_RadioState_GetRfg(void) {
    return s_state.rfg;
}

float App_RadioState_GetLeftDacScale(void) {
    /* In voice_rec the left DAC carries the microphone to HPL; its level stays
     * fixed so the host signal does not move with rxvol. */
    return s_state.voice_rec ? s_left_scale[0] : s_state.left_dac_scale;
}

float App_RadioState_GetRightDacScale(void) {
    return s_state.voice_rec ? s_state.voice_rec_right_dac_scale
                             : s_right_scale;
}

int8_t App_RadioState_GetHprGainDb(void) {
    return s_state.hpr_gain_db;
}

bool App_RadioState_IsMuted(void) {
    return s_state.hpr_mute;
}


void App_RadioState_SetMicSel(uint8_t sel) {
    if (!s_edit_pending) {
        LOG_ERROR("set mic sel outside staged edit");
        return;
    }
    if (sel > (uint8_t)APP_RADIO_MIC_AUTO) {
        LOG_DEBUG("micsel %u out of range, ignored", (unsigned)sel);
        return;
    }
    s_edit.mic_sel = sel;
}

void App_RadioState_SetMicGains(uint8_t ling, uint8_t loutg, uint8_t imicg,
                                uint8_t hmicg) {
    if (!s_edit_pending) {
        LOG_ERROR("set mic gains outside staged edit");
        return;
    }
    s_edit.ling  = ling;
    s_edit.loutg = loutg;
    s_edit.imicg = imicg;
    s_edit.hmicg = hmicg;
}

uint8_t App_RadioState_GetMicSel(void) {
    return s_state.mic_sel;
}

uint8_t App_RadioState_GetLing(void) {
    return s_state.ling;
}

uint8_t App_RadioState_GetLoutg(void) {
    return s_state.loutg;
}

uint8_t App_RadioState_GetImicg(void) {
    return s_state.imicg;
}

uint8_t App_RadioState_GetHmicg(void) {
    return s_state.hmicg;
}
