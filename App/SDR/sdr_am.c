#include "sdr_am.h"

#include <arm_math.h>

#include "board_config.h"
#include "bsp_log.h"
#include "bsp_irq.h"
#include "drv_codec.h"
#include "sdr_iir.h"
#include "sdr_spectrum.h"
#include "app_event.h"

#define LOG_TAG "app.sdr.am"

#define SDR_AM_FILTER_NUM_STAGES SDR_IIR_STAGES_5
#define SDR_AM_RX_SAMPLE_RATE_DIVISOR 2u
#define SDR_AM_TX_SAMPLE_RATE_DIVISOR 1u
#define SDR_AM_RX_RATE (BOARD_AUDIO_SAMPLE_RATE_HZ / SDR_AM_RX_SAMPLE_RATE_DIVISOR)
#define SDR_AM_RX_BLOCK_SIZE (DRV_CODEC_BLOCK_FRAMES / SDR_AM_RX_SAMPLE_RATE_DIVISOR)
#define SDR_AM_TX_BLOCK_SIZE (DRV_CODEC_BLOCK_FRAMES / SDR_AM_TX_SAMPLE_RATE_DIVISOR)

#define SDR_AM_DC_BLOCK_FREQ (40.0f)
#define SDR_SAM_PLL_BW_HZ (40.0f)
#define SDR_SAM_PLL_INT_MAX_FREQ (350.0f)

static uint32_t s_filter_inst_freq[2];
static arm_biquad_casd_df1_inst_f32 s_iq_filter_inst[2];
static float s_filter_coeffs[2][SDR_AM_FILTER_NUM_STAGES * 5];
static volatile uint32_t s_filter_coeffs_id = 0;
static float s_iq_filter_state[2][SDR_AM_FILTER_NUM_STAGES * 4];

static arm_biquad_casd_df1_inst_f32 s_audio_filter_inst;
static float s_audio_filter_state[SDR_AM_FILTER_NUM_STAGES * 4];

static Sdr_DC_Filter s_dc;

/* Squelch metric: carrier level (the DC the demod removes) vs the audio-domain
 * noise. Updated once per demod block, read in the same ISR pass. */
static float s_signal_metric;

static struct {
    // NCO
    float nco_re;
    float nco_im;
    // pll
    float omega;
    float ki;
    float kp;
    // filter
    float err_avg;
} s_sam;


static struct {
    SdrModulation rx_mode;
    AppRadioMode radio_mode;
    uint32_t filter_freq;
} s_state;

static struct {
    bool reset_dc;
    bool reset_iq_iir;
    bool reset_audio_iir;
    bool reset_pll;
    bool filter_switch;
} s_pending;

static void reset_pll(void) {
    s_sam.omega = 0.0f;
    s_sam.nco_re = 1.0f;
    s_sam.nco_im = 0.0f;
}

static void pll_set_bandwidth(float f_bw_hz) {
    const float zeta = 0.707f;
    float wn = 2.0f * (float)M_PI * f_bw_hz;
    float Ts = 1.0f / SDR_AM_RX_RATE;

    s_sam.kp = 2.0f * zeta * wn * Ts;
    s_sam.ki = (wn * Ts) * (wn * Ts);
}

static void update_filter(void) {
    // Prevent switching from ISR
    uint32_t key = Bsp_Irq_SaveLow();
    s_pending.filter_switch = false;
    if (s_filter_inst_freq[s_filter_coeffs_id] == s_state.filter_freq) {
        Bsp_Irq_RestoreLow(key);
        return;
    }
    Bsp_Irq_RestoreLow(key);

    uint32_t shadow_id = s_filter_coeffs_id ^ 1u;

    if (s_filter_inst_freq[shadow_id] != s_state.filter_freq) {
        SdrIirSpec spec = {
            .edge = SDR_IIR_EDGE_STOPBAND,
            .f_edge_hz = s_state.filter_freq,
            .fs_hz = (float)SDR_AM_RX_RATE,
            .stages = SDR_AM_FILTER_NUM_STAGES,
            .type = SDR_IIR_TYPE_LPF
        };
        uint8_t stages;
        SdrIirResponse resp;
        int res = Sdr_Iir_DesignEllipEx(&spec, s_filter_coeffs[shadow_id],
                                         &stages, &resp);
        if (res != 0) {
            LOG_WARN("Sdr_Iir_DesignEllipEx error: %i", res);
            return;
        }
        LOG_DEBUG("iir lpf ref %.1f stop %.1f nbw %.1f gain %.2f level %.1f",
                  resp.f_ref_hz, resp.f_stop_hz, resp.noise_bw_hz,
                  resp.enbw_gain_db, resp.level_hz);
        s_filter_inst_freq[shadow_id] = s_state.filter_freq;
    }
    key = Bsp_Irq_SaveLow();
    s_pending.filter_switch = true;
    Bsp_Irq_RestoreLow(key);
}

/* Events subscriptions */

static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;
    int32_t abs_low = cfg->filter_low_hz > 0 ? cfg->filter_low_hz
                                             : -cfg->filter_low_hz;
    int32_t abs_high = cfg->filter_high_hz > 0 ? cfg->filter_high_hz
                                               : -cfg->filter_high_hz;
    int32_t max = MAX(abs_high, abs_low);
    bool filter_changed = s_state.filter_freq != (uint32_t)max;
    s_state.filter_freq = (uint32_t)max;

    if (cfg->rx_mode != s_state.rx_mode) {
        if (s_state.rx_mode == SDR_MOD_AM) {
            // Leaving AM
            uint32_t key = Bsp_Irq_SaveLow();
            s_pending.reset_iq_iir = true;
            s_pending.reset_audio_iir = true;
            s_pending.reset_dc = true;
            Bsp_Irq_RestoreLow(key);
        } else if (s_state.rx_mode == SDR_MOD_SAM) {
            // Leaving SAM
            uint32_t key = Bsp_Irq_SaveLow();
            s_pending.reset_audio_iir = true;
            s_pending.reset_dc = true;
            s_pending.reset_pll = true;
            Bsp_Irq_RestoreLow(key);
        }
        s_state.rx_mode = cfg->rx_mode;
        if (cfg->rx_mode == SDR_MOD_AM || cfg->rx_mode == SDR_MOD_SAM) {
            update_filter();
        }
    } else if (filter_changed &&
               (s_state.rx_mode == SDR_MOD_AM || s_state.rx_mode == SDR_MOD_SAM)) {
        update_filter();
    }
}

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (evt->mode == APP_RADIO_MODE_TX && s_state.radio_mode != evt->mode) {
        // Clear both iir on TX on AM and only first on TX on SAM
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.reset_iq_iir = true;
        s_pending.reset_audio_iir |= s_state.rx_mode == SDR_MOD_AM;
        s_pending.reset_dc = true;
        Bsp_Irq_RestoreLow(key);
    }
    s_state.radio_mode = evt->mode;
}

void Sdr_AM_Init(void) {
    // Setup IIR
    s_filter_coeffs_id = 0u;
    s_state.filter_freq = 5000u;
    arm_biquad_cascade_df1_init_f32(&s_iq_filter_inst[0], SDR_AM_FILTER_NUM_STAGES,
                                    s_filter_coeffs[s_filter_coeffs_id], s_iq_filter_state[0]);
    arm_biquad_cascade_df1_init_f32(&s_iq_filter_inst[1], SDR_AM_FILTER_NUM_STAGES,
                                    s_filter_coeffs[s_filter_coeffs_id], s_iq_filter_state[1]);
    arm_biquad_cascade_df1_init_f32(&s_audio_filter_inst, SDR_AM_FILTER_NUM_STAGES,
                                    s_filter_coeffs[s_filter_coeffs_id], s_audio_filter_state);
    update_filter();

    // Setup SAM PLL
    reset_pll();
    pll_set_bandwidth(SDR_SAM_PLL_BW_HZ);

    // Set pending flags
    s_pending.reset_dc = true;
    // state filled with init function
    s_pending.reset_iq_iir = false;
    s_pending.reset_audio_iir = false;
    s_pending.reset_pll = false;

    s_dc.alpha = (float)M_TWOPI * SDR_AM_DC_BLOCK_FREQ / SDR_AM_RX_RATE;

    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
}

static void apply_pending(void) {
    if (s_pending.reset_dc) {
        s_pending.reset_dc = false;
        Sdr_DC_Filter_Reset(&s_dc);
    }
    if (s_pending.reset_iq_iir) {
        s_pending.reset_iq_iir = false;
        arm_fill_f32(0.0f, s_iq_filter_state[0], SDR_AM_FILTER_NUM_STAGES * 4);
        arm_fill_f32(0.0f, s_iq_filter_state[1], SDR_AM_FILTER_NUM_STAGES * 4);
    }
    if (s_pending.reset_audio_iir) {
        s_pending.reset_audio_iir = false;
        arm_fill_f32(0.0f, s_audio_filter_state, SDR_AM_FILTER_NUM_STAGES * 4);
    }
    if (s_pending.reset_pll) {
        s_pending.reset_pll = false;
        reset_pll();
    }
    if (s_pending.filter_switch) {
        s_pending.filter_switch = false;
        s_filter_coeffs_id = s_filter_coeffs_id ^ 1;
        s_iq_filter_inst[0].pCoeffs = s_filter_coeffs[s_filter_coeffs_id];
        s_iq_filter_inst[1].pCoeffs = s_filter_coeffs[s_filter_coeffs_id];
        s_audio_filter_inst.pCoeffs = s_filter_coeffs[s_filter_coeffs_id];
    }
}

/* Carrier-to-noise metric shared by AM and SAM. An invalid or non-positive
 * carrier (first sample, unlocked SAM PLL) and an invalid noise estimate both
 * map to 0 (no data). */
static void am_update_metric(void) {
    float carrier = Sdr_AM_GetCarrierLevel();

    if (!Sdr_Spectrum_Is_Noise_Valid() || carrier <= 0.0f) {
        s_signal_metric = 0.0f;
        return;
    }
    s_signal_metric = Sdr_Lin2Db(carrier) - Sdr_Spectrum_Get_Demod_Noise_Est_Db();
    s_signal_metric = MAX(s_signal_metric, 0.0f);
}

void Sdr_AM_Demod(const Sdr_IQ_Data *data, float *audio) {

    if (!data || !audio || data->size != SDR_AM_RX_BLOCK_SIZE) {
        return;
    }
    apply_pending();

    // Filter IQ
    arm_biquad_cascade_df1_f32(&s_iq_filter_inst[0], data->i, data->i, data->size);
    arm_biquad_cascade_df1_f32(&s_iq_filter_inst[1], data->q, data->q, data->size);
    // Demod
    for (size_t i = 0; i < data->size; i++) {
        float v = sqrtf(data->i[i] * data->i[i] + data->q[i] * data->q[i]);
        audio[i] = Sdr_DC_Filter_Apply(&s_dc, v);
    }
    arm_biquad_cascade_df1_f32(&s_audio_filter_inst, audio, audio, data->size);
    am_update_metric();
}

void Sdr_SAM_Demod(const Sdr_IQ_Data *data, float *audio) {

    if (!data || !audio || data->size != SDR_AM_RX_BLOCK_SIZE) {
        return;
    }
    apply_pending();

    float noise = 0.0f;
    if (Sdr_Spectrum_Is_Noise_Valid()) {
       noise = Sdr_Spectrum_Get_Noise_Lin() * 0.01f;
    }
    noise = MAX(noise, 1e-5f);

    const float OMEGA_MAX = 2.0f * M_PI * SDR_SAM_PLL_INT_MAX_FREQ / SDR_AM_RX_RATE;

    for (size_t n = 0; n < data->size; n++) {
        // Apply NCO
        float32_t I = data->i[n] * s_sam.nco_re + data->q[n] * s_sam.nco_im;
        float32_t Q = data->q[n] * s_sam.nco_re - data->i[n] * s_sam.nco_im;

        // Save audio
        audio[n] = Sdr_DC_Filter_Apply(&s_dc, I);

        // Get err
        float denom = fabsf(I) + fabsf(Q) + noise;
        float err = Q / denom;
        // Average
        s_sam.err_avg += 0.5f * (err - s_sam.err_avg);
        // PI
        s_sam.omega += s_sam.ki * s_sam.err_avg;
        if (s_sam.omega > OMEGA_MAX)  s_sam.omega = OMEGA_MAX;
        if (s_sam.omega < -OMEGA_MAX) s_sam.omega = -OMEGA_MAX;

        float total_step_angle = s_sam.omega + s_sam.kp * s_sam.err_avg;

        // Pade approximation
        float t = total_step_angle * 0.5f;
        float t2 = t * t;
        float dev = 1.0f / (1.0f + t2);

        float step_re = (1.0f - t2) * dev;
        float step_im = 2.0f * t * dev;

        float next_nco_re = s_sam.nco_re * step_re - s_sam.nco_im * step_im;
        float next_nco_im = s_sam.nco_im * step_re + s_sam.nco_re * step_im;

        s_sam.nco_re = next_nco_re;
        s_sam.nco_im = next_nco_im;
    }
    float norm = 1.0f / sqrtf(s_sam.nco_re * s_sam.nco_re + s_sam.nco_im * s_sam.nco_im);
    s_sam.nco_re *= norm;
    s_sam.nco_im *= norm;
    arm_biquad_cascade_df1_f32(&s_audio_filter_inst, audio, audio, data->size);
    am_update_metric();
}

float Sdr_AM_GetCarrierLevel(void) {
    return s_dc.invalid ? 0.0f : fabsf(s_dc.val);
}

float Sdr_AM_GetSignalMetric(void) {
    return s_signal_metric;
}

void Sdr_AM_Mod(const float *audio, Sdr_IQ_Data *data) {

    if (!data || !audio || data->size != SDR_AM_TX_BLOCK_SIZE) {
        return;
    }
    apply_pending();

    float k = 0.5f;
    for (size_t n = 0; n < data->size; n++) {
        data->i[n] = k * audio[n] + k;
        data->q[n] = 0.0f;
    }
}
