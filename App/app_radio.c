#include "app_radio.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_alc.h"
#include "app_event.h"
#include "app_ptt.h"
#include "app_fpga.h"
#include "app_mic_pre.h"
#include "app_protect.h"
#include "app_radio_rfpath.h"
#include "app_radio_state.h"
#include "app_scheduler.h"
#include "app_telemetry.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_board.h"
#include "drv_codec.h"

#include "arm_math.h"
#include "bsp_gpio.h"
#include "bsp_time.h"
#include "main.h"
#include "drv_fpga.h"
#include "sdr_agc.h"
#include "sdr_am.h"
#include "sdr_audio_resample.h"
#include "sdr_cw.h"
#include "sdr_nb.h"
#include "sdr_nbfm.h"
#include "sdr_nr.h"
#include "sdr_resample.h"
#include "sdr_rx.h"
#include "sdr_squelch.h"
#include "sdr_spectrum.h"
#include "sdr_ssb.h"
#include "sdr_tx.h"
#include "sdr_wfm.h"

#include "app_tx_monitor.h"

#define LOG_TAG "app.radio"

#define Q31_SCALE (2147483648.0f)

/* Fixed codec PGA for the host line-in (IN3_R) when TX is keyed by the host
 * `mtx` flag. It is independent of the host gain setting used by the voice_rec
 * loop; tune on hardware. */
#define APP_RADIO_TX_HOST_IN_GAIN 0x00u

/* Time to let the LPF relay sequence and the analog TX path settle after a band
 * change. Used for the FPGA TX mute and for delaying CW generation start. */
#define APP_RADIO_TX_RELAY_SETTLE_MS 120u

static AppRadioMode      s_radio_mode = APP_RADIO_MODE_OFF;

/* TX path settle window, opened at TX entry only when the LPF band changes. */
static uint32_t s_tx_settle_until_ms;

static volatile uint32_t s_aic_samples;

/* TX modulation mode: inactive VFO when split, active VFO otherwise. Only the
 * TX modulator consumes it; RX demodulation and the spectrum keep the
 * active-VFO mode. */
static SdrModulation s_tx_mode = SDR_MOD_LSB;

/* TX request state pushed by the PTT arbiter. The winning source selects the
 * TX audio input; a calibration request also inhibits the TX signal path. Read
 * from the main loop (audio) and the I2S DMA ISR (inhibit). */
static AppPttSource  s_tx_source = APP_PTT_SOURCE_NONE;
static volatile bool s_tx_signal_inhibit;

/* Tune carrier used by the ATU and the SWR scan. The envelope ramps between 0
 * and 1 over APP_RADIO_TUNE_RAMP_MS so a relay blank is a smooth fade rather
 * than a hard gate; carrier_on is the ISR-visible target written by
 * App_Radio_SetTuneCarrier(). */
#define APP_RADIO_TUNE_RAMP_MS 1u

typedef struct {
    volatile bool carrier_on;
    float         env;
    float         env_step;
} AppRadioCarrier;

static AppRadioCarrier s_carrier;

/* TX publish staging: convert one I2S block of float I/Q to Q1.31 before
 * handing it to the FPGA driver. Small fixed buffers keep the conversion off
 * the stack in the I2S DMA ISR. */
static int32_t  s_tx_pub_i[DRV_CODEC_BLOCK_FRAMES];
static int32_t  s_tx_pub_q[DRV_CODEC_BLOCK_FRAMES];
static float    s_tx_pub_f_i[DRV_CODEC_BLOCK_FRAMES];
static float    s_tx_pub_f_q[DRV_CODEC_BLOCK_FRAMES];
/* Samples rejected by the TX staging ring since the last housekeeping log.
 * Accumulated atomically; never logged from the ISR. */
static volatile uint32_t s_tx_rejected;

static bool     s_hp_initialized;
static bool     s_hp_candidate;
static bool     s_hp_stable;
static uint32_t s_hp_change_ms;
static bool     s_speaker_written;
static bool     s_speaker_last;

/* Codec audio configuration last written by App_Radio_ApplyAudioConfig/ForTx.
 * s_applied_mic is the resolved type (BUILTIN/HANDLE), not the raw micsel. */
static DrvCodecInputL s_applied_input;
static uint8_t        s_applied_gain = 0;
static uint8_t        s_applied_mic;
static uint8_t        s_applied_loutg;
static bool           s_loutg_valid;
static int8_t         s_applied_hpr_gain;
static bool           s_applied_hpr_mute;
static bool           s_hpr_valid;
static DrvCodecInputR s_applied_input_r;
static uint8_t        s_applied_gain_r;
static bool           s_input_r_valid;
static DrvCodecHprSource s_applied_hpr_source;
static bool           s_hpr_source_valid;

static float s_mic_audio[DRV_CODEC_BLOCK_FRAMES] __attribute__((aligned(32)));
static float s_in_audio[DRV_CODEC_BLOCK_FRAMES];
static float s_mon_audio[DRV_CODEC_BLOCK_FRAMES];
static float s_rx_audio[DRV_CODEC_BLOCK_FRAMES];

/*
 * Publish one modulator output block. Called from the I2S DMA ISR by the future
 * TX modulator. Every float sample goes to the spectrum first (the spectrum
 * shows the generated TX signal, not only the part the driver accepted), then
 * the block is converted to Q1.31 and copied into the FPGA staging ring in
 * chunks. A short acceptance is counted and logged later from housekeeping; no
 * log, block or divide happens here.
 */
void App_Radio_TxPublish(const float *i, const float *q, uint32_t count) {
    uint32_t offset;
    uint32_t accepted;
    float    out_scale;

    if (i == NULL || q == NULL || count == 0u) {
        return;
    }

    Sdr_Spectrum_AppendTx(i, q, count);

    /* The spectrum stays pre-regulation; only the output stage is scaled. The
     * ALC also consumes this block's IQ statistics for its feed-forward path.
     * App_Alc_GetOutputScale() applies k_mod and the per-band level offset once,
     * at the float -> Q1.31 FPGA boundary. */
    out_scale = App_Alc_GetOutputScale(i, q, count) * Q31_SCALE;

    accepted = 0u;
    for (offset = 0u; offset < count; offset += DRV_CODEC_BLOCK_FRAMES) {
        uint32_t n = count - offset;

        if (n > DRV_CODEC_BLOCK_FRAMES) {
            n = DRV_CODEC_BLOCK_FRAMES;
        }
        for (uint32_t k = 0; k < n; ++k) {
            s_tx_pub_i[k] =
                (int32_t)(i[offset + k] * out_scale);
            s_tx_pub_q[k] =
                (int32_t)(q[offset + k] * out_scale);
        }
        /* A full ring rejects the rest of the block; the loop still converts
         * every chunk so the spectrum always sees the complete TX signal. */
        accepted += Drv_Fpga_TxWrite(s_tx_pub_i, s_tx_pub_q, n);
    }

    if (accepted < count) {
        uint32_t key = Bsp_Irq_SaveLow();

        s_tx_rejected += count - accepted;
        Bsp_Irq_RestoreLow(key);
    }
}

void App_Radio_LogTxDrops(void) {
    uint32_t rejected;
    uint32_t key = Bsp_Irq_SaveLow();

    rejected     = s_tx_rejected;
    s_tx_rejected = 0u;
    Bsp_Irq_RestoreLow(key);

    if (rejected != 0u) {
        LOG_WARN("tx staging overflow: %u samples", (unsigned)rejected);
    }
}

/* Fill one block of the tune carrier: a ramped DC envelope on
 * I, zero on Q. Runs in the I2S DMA ISR. */
static void radio_tune_carrier_block(uint32_t n, float *i, float *q) {
    float target = s_carrier.carrier_on ? 1.0f : 0.0f;

    for (uint32_t k = 0u; k < n; ++k) {
        if (s_carrier.env < target) {
            s_carrier.env += s_carrier.env_step;
            if (s_carrier.env > target) {
                s_carrier.env = target;
            }
        } else if (s_carrier.env > target) {
            s_carrier.env -= s_carrier.env_step;
            if (s_carrier.env < target) {
                s_carrier.env = target;
            }
        }
        i[k] = s_carrier.env;
        q[k] = 0.0f;
    }
}

static void radio_block_cb(const int32_t *in_audio_buf, int32_t *out_audio_buf, unsigned n_samples,
                           void *ctx) {
    (void)ctx;

    float *sidetone = Sdr_CW_KeyerUpdate();

    s_aic_samples += n_samples;

    bool out_filled = false;

    /* TX modulation mode: inactive VFO when split, active VFO otherwise. Only
     * the modulator consumes it; RX demodulation and the spectrum keep the
     * active-VFO mode. */
    if (s_radio_mode == APP_RADIO_MODE_TX) {
        SdrModulation tx_mode = App_RadioState_GetTxMode();
        if (tx_mode != s_tx_mode) {
            s_tx_mode = tx_mode;
        }
    }

    /* Build the modulation input from the stereo ADC pair. Host line-in (right
     * ADC) is used only for a host-keyed (mtx) TX: digital modes take it alone,
     * voice modes mix it with the microphone, CW ignores it. */
    bool  host_keyed = s_radio_mode == APP_RADIO_MODE_TX &&
                       s_tx_source == APP_PTT_SOURCE_MTX;
    bool  host_only  = host_keyed &&
                       (s_tx_mode == SDR_MOD_LSB_D ||
                        s_tx_mode == SDR_MOD_USB_D);
    bool  host_used  = host_keyed && s_tx_mode != SDR_MOD_CW &&
                       s_tx_mode != SDR_MOD_CWR;
    float tx_mic_k   = host_only ? 0.0f : 1.0f;
    float tx_host_k  = host_used ? 1.0f : 0.0f;
    float audio_in_q31_to_f = 20.0f / Q31_SCALE;

    for (size_t i = 0; i < n_samples; i++) {
        s_mic_audio[i] = (float)in_audio_buf[2 * i] * audio_in_q31_to_f;
    }

    /* Some preprocess for handle mic */
    if (s_radio_mode == APP_RADIO_MODE_TX && tx_mic_k > 0.0f &&
        s_tx_source != APP_PTT_SOURCE_ATU_TUNE &&
        s_tx_source != APP_PTT_SOURCE_SWR_SCAN) {
        App_MicPre_Process(s_mic_audio, n_samples);
    }

    for (size_t i = 0; i < n_samples; i++) {
        s_in_audio[i] = s_mic_audio[i] * tx_mic_k +
                        (float)in_audio_buf[2 * i + 1] *
                            audio_in_q31_to_f * tx_host_k;
    }

    /* One multiplier per DAC per block: left DAC (HPR + host) and
        * right DAC (USB/LOR). No I2C or state reads in the loop. */
    float left_audio_out_k  = App_RadioState_GetLeftDacScale() * Q31_SCALE;
    float right_audio_out_k = App_RadioState_GetRightDacScale() * Q31_SCALE;


    if (s_radio_mode == APP_RADIO_MODE_RX) {
        if (App_RadioState_GetVoiceRec()) {
            /* Voice recording: neither demodulation nor modulation runs. The
             * left ADC (microphone) goes to the left DAC and the right ADC
             * (host line-in) to the right DAC, both with the same block scale
             * as the modulation input. s_in_audio stays a TX-only buffer.
             * The Q31 factors cancel, so the ADC-to-DAC transfer is folded into
             * one coefficient per channel. */
            const float mic_k  = audio_in_q31_to_f * left_audio_out_k;
            const float host_k = audio_in_q31_to_f * right_audio_out_k;

            for (size_t i = 0; i < n_samples; i++) {
                out_audio_buf[2u * i] =
                    clip_q63_to_q31((q63_t)((float)in_audio_buf[2 * i] * mic_k));
                out_audio_buf[2u * i + 1] =
                    clip_q63_to_q31((q63_t)((float)in_audio_buf[2 * i + 1] * host_k));
            }
            out_filled = true;
        } else if (Sdr_Rx_Process(s_rx_audio, DRV_CODEC_BLOCK_FRAMES)) {

            for (size_t i = 0; i < DRV_CODEC_BLOCK_FRAMES; i++) {
                float sample = s_rx_audio[i];
                if (sidetone) {
                    sample += sidetone[i];
                }
                out_audio_buf[2u * i]     = clip_q63_to_q31(sample * left_audio_out_k);
                out_audio_buf[2u * i + 1] = clip_q63_to_q31(sample * right_audio_out_k);
            }
            out_filled = true;
        }
    } else if (s_radio_mode == APP_RADIO_MODE_TX) {
        /*
         * Modulate one block and publish the I/Q to the TX staging ring and the
         * TX spectrum. The tune-carrier branch bypasses the modulator; the TX
         * audio monitor below stays silent.
         */

        if (s_tx_source == APP_PTT_SOURCE_ATU_TUNE ||
            s_tx_source == APP_PTT_SOURCE_SWR_SCAN) {
            /* Tune carrier: no modulator, no monitor audio. The envelope
             * blank (App_Radio_SetTuneCarrier(false)) owns the relay-switch mute
             * instead of s_tx_signal_inhibit. */
            radio_tune_carrier_block(DRV_CODEC_BLOCK_FRAMES, s_tx_pub_f_i,
                                    s_tx_pub_f_q);
            App_Radio_TxPublish(s_tx_pub_f_i, s_tx_pub_f_q,
                                DRV_CODEC_BLOCK_FRAMES);
        } else {
            Sdr_Tx_Process(s_in_audio, DRV_CODEC_BLOCK_FRAMES, s_tx_mode,
                           s_tx_pub_f_i, s_tx_pub_f_q, s_mon_audio);

            if (!s_tx_signal_inhibit) {
                App_Radio_TxPublish(s_tx_pub_f_i, s_tx_pub_f_q,
                                    DRV_CODEC_BLOCK_FRAMES);
            }

            /* Monitor level scales only the modulated-signal monitor; the
             * sidetone keeps the same gain as in the RX/training path. */
            float mon_k = App_RadioState_GetMonitorK();
            for (unsigned i = 0; i < n_samples; ++i) {
                float sample = s_mon_audio[i] * mon_k;
                if (sidetone) {
                    sample += sidetone[i];
                }
                out_audio_buf[2u * i] = clip_q63_to_q31(sample * left_audio_out_k);
                out_audio_buf[2u * i + 1] = clip_q63_to_q31(sample * right_audio_out_k);
            }
            out_filled = true;
        }
    }

    if (!out_filled) {
        for (unsigned i = 0; i < n_samples; ++i) {
            out_audio_buf[2u * i]     = 0;
            out_audio_buf[2u * i + 1] = 0;
        }
    }

    Sdr_Spectrum_Process();
}

void App_Radio_UpdateSpeaker(void) {
    bool     connected;
    bool     att;
    uint32_t now;

    connected = Bsp_Gpio_Read((BspPin){HEADPHONE_DETECT_GPIO_Port,
                                       HEADPHONE_DETECT_Pin}) == GPIO_PIN_RESET;
    now       = Bsp_Time_GetMs();

    if (!s_hp_initialized) {
        s_hp_candidate   = connected;
        s_hp_stable      = connected;
        s_hp_change_ms   = now;
        s_hp_initialized = true;
    } else if (connected != s_hp_candidate) {
        s_hp_candidate = connected;
        s_hp_change_ms = now;
    } else if (s_hp_candidate != s_hp_stable &&
               (now - s_hp_change_ms) >= 200u) {
        s_hp_stable = s_hp_candidate;
        LOG_DEBUG("headphone %s", s_hp_stable ? "connected" : "disconnected");
    }

    att = !s_hp_stable ? false : App_RadioState_GetSpmode();

    if (!s_speaker_written || att != s_speaker_last) {
        Bsp_Gpio_Write((BspPin){AUDIO_OUT_ATT_GPIO_Port, AUDIO_OUT_ATT_Pin},
                       att ? GPIO_PIN_RESET : GPIO_PIN_SET);
        s_speaker_written = true;
        s_speaker_last    = att;
        LOG_DEBUG("speaker %s (hp %u, spmode %u)", att ? "off" : "on",
                  (unsigned)s_hp_stable,
                  (unsigned)App_RadioState_GetSpmode());
    }
}

static void audio_apply_input_gain(DrvCodecInputL input, uint8_t gain) {
    int rc;

    if (input != s_applied_input) {
        rc = Drv_Codec_SetInputL(input);
        if (rc == 0) {
            s_applied_input = input;
            LOG_DEBUG("audio input %u", (unsigned)input);
        } else {
            LOG_WARN("audio input %u failed (%d)", (unsigned)input, rc);
        }
    }
    if (gain != s_applied_gain) {
        rc = Drv_Codec_SetInGainL(gain);
        if (rc == 0) {
            s_applied_gain = gain;
            LOG_DEBUG("audio gain %u", (unsigned)gain);
        } else {
            LOG_WARN("audio gain %u failed (%d)", (unsigned)gain, rc);
        }
    }
}

static AppRadioMicSel audio_resolve_mic(void) {
    uint8_t sel = App_RadioState_GetMicSel();

    if (sel == (uint8_t)APP_RADIO_MIC_AUTO) {
        return Bsp_Board_IsExtMicConnected() ? APP_RADIO_MIC_HANDLE
                                             : APP_RADIO_MIC_BUILTIN;
    }
    return (AppRadioMicSel)sel;
}

static void audio_apply_mic(void) {
    AppRadioMicSel mic = audio_resolve_mic();

    if (mic == APP_RADIO_MIC_HANDLE) {
        audio_apply_input_gain(DRV_CODEC_IN_L_HMIC, App_RadioState_GetHmicg());
    } else {
        audio_apply_input_gain(DRV_CODEC_IN_L_IMIC, App_RadioState_GetImicg());
    }
    App_MicPre_SetEnabled(mic == APP_RADIO_MIC_HANDLE);
    if (s_applied_mic != (uint8_t)mic) {
        s_applied_mic = (uint8_t)mic;
        LOG_DEBUG("audio mic %u", (unsigned)mic);
    }
}

static void audio_apply_usb(void) {
    App_MicPre_SetEnabled(false);
    audio_apply_input_gain(DRV_CODEC_IN_L_USB, App_RadioState_GetLing());
}

static void audio_apply_loutg(void) {
    uint8_t loutg = App_RadioState_GetLoutg();
    int     rc;

    if (s_loutg_valid && loutg == s_applied_loutg) {
        return;
    }
    rc = Drv_Codec_SetLORGain(true, (int8_t)(loutg - 5));
    if (rc == 0) {
        s_applied_loutg = loutg;
        s_loutg_valid   = true;
        LOG_DEBUG("audio loutg %u", (unsigned)loutg);
    } else {
        LOG_WARN("audio loutg %u failed (%d)", (unsigned)loutg, rc);
    }
}

static void audio_apply_hpr(void) {
    int8_t gain = App_RadioState_GetHprGainDb();
    bool   mute = App_RadioState_IsMuted();
    int    rc;

    if (s_hpr_valid && gain == s_applied_hpr_gain && mute == s_applied_hpr_mute) {
        return;
    }
    rc = Drv_Codec_SetHPRGain(!mute, gain);
    if (rc == 0) {
        s_applied_hpr_gain = gain;
        s_applied_hpr_mute = mute;
        s_hpr_valid        = true;
        LOG_DEBUG("audio hpr gain %d mute %u", (int)gain, (unsigned)mute);
    } else {
        LOG_WARN("audio hpr gain %d failed (%d)", (int)gain, rc);
    }
}

static void audio_apply_input_r(DrvCodecInputR input, uint8_t gain) {
    int rc;

    if (s_input_r_valid && input == s_applied_input_r &&
        gain == s_applied_gain_r) {
        return;
    }
    rc = Drv_Codec_SetInputR(input);
    if (rc != 0) {
        LOG_WARN("audio input r %u failed (%d)", (unsigned)input, rc);
        return;
    }
    rc = Drv_Codec_SetInGainR(gain);
    if (rc != 0) {
        LOG_WARN("audio gain r %u failed (%d)", (unsigned)gain, rc);
        return;
    }
    s_applied_input_r = input;
    s_applied_gain_r  = gain;
    s_input_r_valid   = true;
    LOG_DEBUG("audio input r %u gain %u", (unsigned)input, (unsigned)gain);
}

static void audio_apply_hpr_source(DrvCodecHprSource source) {
    int rc;

    if (s_hpr_source_valid && source == s_applied_hpr_source) {
        return;
    }
    rc = Drv_Codec_SetHprSource(source);
    if (rc == 0) {
        s_applied_hpr_source = source;
        s_hpr_source_valid   = true;
        LOG_DEBUG("audio hpr source %u", (unsigned)source);
    } else {
        LOG_WARN("audio hpr source %u failed (%d)", (unsigned)source, rc);
    }
}

/* Voice recording: microphone to HPL (host), host line-in to HPR and LOR.
 * HPR and LOR share the right DAC, so the host signal also reaches USB. */
static void audio_apply_voice_rec(void) {
    audio_apply_mic();
    audio_apply_input_r(DRV_CODEC_IN_R_HOST, App_RadioState_GetLing());
    audio_apply_hpr_source(DRV_CODEC_HPR_SRC_DACR_P);
}

/* Normal RX: HPR and host share the left DAC, USB/LOR the right DAC. */
static void audio_apply_rx_normal(void) {
    audio_apply_mic();
    audio_apply_input_r(DRV_CODEC_IN_R_NONE, 0u);
    audio_apply_hpr_source(DRV_CODEC_HPR_SRC_DACL_N);
}

static void audio_apply_tx_input(void) {
    SdrModulation mode;

    if (s_tx_source == APP_PTT_SOURCE_ATU_TUNE ||
        s_tx_source == APP_PTT_SOURCE_SWR_SCAN) {
        return; /* the tune carrier does not use the audio input path */
    }
    /* Host-keyed TX: the microphone stays on the left ADC and the host
     * line-in is added on the right one; the ISR picks the source per mode. */
    if (s_tx_source == APP_PTT_SOURCE_MTX) {
        audio_apply_mic();
        audio_apply_input_r(DRV_CODEC_IN_R_HOST, APP_RADIO_TX_HOST_IN_GAIN);
        return;
    }
    /* Digital modes expect an external source regardless of how TX was keyed. */
    mode = App_RadioState_GetTxMode();
    if (s_tx_source == APP_PTT_SOURCE_CIVPTT || mode == SDR_MOD_LSB_D ||
        mode == SDR_MOD_USB_D) {
        audio_apply_usb();
    } else {
        audio_apply_mic();
    }
}

void App_Radio_ApplyAudioConfig(void) {
    audio_apply_loutg();
    audio_apply_hpr();

    if (App_Radio_GetRadioMode() == APP_RADIO_MODE_TX) {
        audio_apply_tx_input();
    } else if (App_Radio_GetRadioMode() == APP_RADIO_MODE_RX) {
        if (App_RadioState_GetVoiceRec()) {
            audio_apply_voice_rec();
        } else {
            audio_apply_rx_normal();
        }
    }
}

void App_Radio_ApplyAudioForTx(void) {
    audio_apply_tx_input();
}

static void speaker_task(void) {
    App_Radio_UpdateSpeaker();
}

void App_Radio_SetTxSource(AppPttSource source) {
    s_tx_source         = source;
    s_tx_signal_inhibit = (source == APP_PTT_SOURCE_CALIB);
}

void App_Radio_SetTuneCarrier(bool on) {
    uint32_t key = Bsp_Irq_SaveLow();
    s_carrier.carrier_on = on;
    Bsp_Irq_RestoreLow(key);
}

int App_Radio_Init(void) {
    int rc;

    s_radio_mode  = APP_RADIO_MODE_OFF;

    s_carrier.carrier_on = false;
    s_carrier.env        = 0.0f;
    s_carrier.env_step   = 1.0f / ((float)APP_RADIO_TUNE_RAMP_MS *
                               (float)BOARD_AUDIO_SAMPLE_RATE_HZ / 1000.0f);

    Sdr_Resample_Init();
    Sdr_SSB_Init();
    Sdr_CW_Init();
    Sdr_AM_Init();
    Sdr_NBFM_Init();
    Sdr_WFM_Init();
    Sdr_AGC_Init();
    Sdr_NR_Init();
    Sdr_NB_Init();
    Sdr_AudioResample_Init();
    Sdr_Rx_Init();
    Sdr_Squelch_Init();
    Sdr_Tx_Init();
    App_MicPre_Init();

    rc = Drv_Codec_Init();
    if (rc != 0) {
        LOG_ERROR("codec init failed (%d)", rc);
        return rc;
    }
    rc = Drv_Codec_SetInputL(DRV_CODEC_IN_L_HMIC);
    // rc = Drv_Codec_SetInputL(DRV_CODEC_IN_L_USB);
    if (rc != 0) {
        LOG_ERROR("set inputs failed (%d)", rc);
        return rc;
    }
    rc = Drv_Codec_SetInGainL(s_applied_gain);
    if (rc != 0) {
        LOG_ERROR("set gain failed (%d)", rc);
        return rc;
    }
    s_applied_input = DRV_CODEC_IN_L_HMIC;
    s_applied_mic   = APP_RADIO_MIC_HANDLE;
    App_MicPre_SetEnabled(true);
    s_applied_loutg = 0u;
    s_loutg_valid   = true;
    s_hpr_valid     = false;
    s_input_r_valid = false;
    s_hpr_source_valid = false;
    rc = Drv_Codec_SetBlockCallback(radio_block_cb, 0);
    if (rc != 0) {
        LOG_ERROR("set block callback failed (%d)", rc);
        return rc;
    }
    rc = Drv_Codec_Start();
    if (rc != 0) {
        LOG_ERROR("codec start failed (%d)", rc);
        return rc;
    }

    // Enable audio out
    Bsp_Gpio_Write((BspPin){AUDIO_OUT_EN_GPIO_Port, AUDIO_OUT_EN_Pin}, GPIO_PIN_SET);

    // Set external mic power (inverted)
    Bsp_Gpio_Write((BspPin){GPIOG, GPIO_PIN_13}, GPIO_PIN_RESET);

    Drv_Codec_SetHPLGain(true, 13); // HOST out
    Drv_Codec_SetLOLGain(false, 15);
    Drv_Codec_SetLORGain(true, -5); // USB out
    audio_apply_hpr();              // SPEAKER/phones, from the initial rxvol

    /* The radio stays APP_RADIO_MODE_OFF until the host configuration commits
     * (App_RadioState_Apply bring-up); no RF relays are driven here. */
    if (App_Scheduler_Add(speaker_task, 10u) != 0) {
        LOG_ERROR("speaker task add failed");
        return -1;
    }
    return 0;
}

int App_Radio_SetRadioMode(AppRadioMode radio_mode) {
    AppRadioMode from = s_radio_mode;

    switch (radio_mode) {
    case APP_RADIO_MODE_RX:
        /* Request the FPGA RX branch first so the analog path switches only
         * after the bus has been handed back to RX. The actual branch changes
         * on the next PE10 tick; the spectrum resyncs on its next Process. */
        Drv_Fpga_SetTxActive(0);
        Sdr_Spectrum_SetTxMode(false);
        /* Return the RF filters to the RX (active VFO) frequency. */
        App_RadioRfPath_SetFreq(App_RadioState_GetRxFreq());
        Drv_Codec_Start();

        // PF2
        Bsp_Gpio_Write((BspPin){RF_PA_RELAY_2_GPIO_Port, RF_PA_RELAY_2_Pin}, GPIO_PIN_SET);
        // PB4
        Bsp_Board_SetPABias(false);
        // PB5
        Bsp_Gpio_Write((BspPin){T_R_RELAY_2_GPIO_Port, T_R_RELAY_2_Pin}, GPIO_PIN_RESET);
        // PF14
        Bsp_Board_SetAccTrx(false);
        // PF4
        Bsp_Board_SetTxRelay(false);
        // PF5
        Bsp_Board_ConnectRxADC(true);

        Bsp_Board_SetStatusLedRed(false);
        Bsp_Board_SetStatusLedGreen(true);

        App_Telemetry_SetTx(false);
        break;
    case APP_RADIO_MODE_TX:
        if (!App_RadioState_IsConfigured()) {
            LOG_WARN("tx refused: host configuration not loaded");
            return -1;
        }
        if (App_Protect_IsTxInhibited()) {
            return -1;
        }
        App_MicPre_Reset();
        App_Radio_ApplyAudioForTx();
        /* Apply the TX frequency (inactive VFO when split). If the LPF band
         * changes, mute the FPGA before the relay sequence starts and open the
         * settle window CW uses to delay its first element. */
        {
            uint32_t tx_freq = App_RadioState_GetTxFreq();
            bool     relay   = App_RadioRfPath_LpfWillChange(tx_freq);

            if (relay) {
                App_Fpga_TxMute(APP_RADIO_TX_RELAY_SETTLE_MS);
            }
            s_tx_settle_until_ms = Bsp_Time_GetMs() +
                                   (relay ? APP_RADIO_TX_RELAY_SETTLE_MS : 0u);
            App_RadioRfPath_SetFreq(tx_freq);
        }
        App_Telemetry_SetTx(true);
        Drv_Codec_Start();
        Bsp_Board_SetStatusLedRed(true);
        Bsp_Board_SetStatusLedGreen(false);

        // PF14
        Bsp_Board_SetAccTrx(true);
        // PB5
        // No TX on reset
        Bsp_Gpio_Write((BspPin){T_R_RELAY_2_GPIO_Port, T_R_RELAY_2_Pin}, GPIO_PIN_SET);
        // PF4
        Bsp_Board_SetTxRelay(true);
        // PF5
        Bsp_Board_ConnectRxADC(false);
        // PB4
        Bsp_Board_SetPABias(true);
        // PF2
        // No TX on set
        Bsp_Gpio_Write((BspPin){RF_PA_RELAY_2_GPIO_Port, RF_PA_RELAY_2_Pin}, GPIO_PIN_RESET);

        /* FPGA last: the analog/relay path is already in TX when the bus asks
         * the FPGA to accept TX I/Q on its next tick. SetTxActive also drops
         * stale staging data so nothing from the previous transmission leaves. */
        App_Alc_NotifyTxStart();
        Drv_Fpga_SetTxActive(1);
        Sdr_Spectrum_SetTxMode(true);
        break;
    case APP_RADIO_MODE_OFF:
    default:
        Drv_Fpga_SetTxActive(0);
        Sdr_Spectrum_SetTxMode(false);
        Drv_Codec_Stop();
        /* Return the RF filters to the RX frequency when turning off. */
        App_RadioRfPath_SetFreq(App_RadioState_GetRxFreq());
        break;
    }
    s_radio_mode = radio_mode;
    /* Entering RX re-applies the RX audio route. The mode must already read RX
     * so ApplyAudioConfig picks the RX inputs/outputs, and a held voice_rec
     * flag takes effect here (including the initial OFF -> RX bring-up). */
    if (from != APP_RADIO_MODE_RX && radio_mode == APP_RADIO_MODE_RX) {
        App_Radio_ApplyAudioConfig();
    }
    if (from != radio_mode) {
        App_Event_NotifyRxTx(radio_mode);
        if (radio_mode == APP_RADIO_MODE_RX) {
            Sdr_AGC_ResetStats();
        }
    }
    return 0;
}

AppRadioMode App_Radio_GetRadioMode(void) {
    return s_radio_mode;
}

uint32_t App_Radio_TxSettleRemainingFrames(void) {
    uint32_t ms;

    if (s_radio_mode == APP_RADIO_MODE_TX) {
        uint32_t now = Bsp_Time_GetMs();

        ms = (s_tx_settle_until_ms > now) ? (s_tx_settle_until_ms - now) : 0u;
    } else {
        /* Not yet in TX: predict the window that SetRadioMode(TX) will open. */
        ms = App_RadioRfPath_LpfWillChange(App_RadioState_GetTxFreq())
                 ? APP_RADIO_TX_RELAY_SETTLE_MS
                 : 0u;
    }
    return (uint32_t)((uint64_t)ms * BOARD_AUDIO_SAMPLE_RATE_HZ / 1000u);
}

uint32_t App_Radio_GetAicSamples(void) {
    uint32_t basepri = Bsp_Irq_SaveLow();
    uint32_t value   = s_aic_samples;

    s_aic_samples = 0u;
    Bsp_Irq_RestoreLow(basepri);
    return value;
}
