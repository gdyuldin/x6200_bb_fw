#include "sdr_ssb.h"

#include <stdbool.h>
#include <stdint.h>

#include <arm_math.h>

#include "board_config.h"
#include "drv_codec.h"
#include "bsp_log.h"
#include "bsp_irq.h"
#include "sdr_iir.h"
#include "sdr_sideband.h"
#include "sdr_spectrum.h"
#include "sdr_squelch_filters.h"
#include "app_event.h"

#define LOG_TAG "app.sdr.ssb"

#define SDR_SSB_RX_SAMPLE_RATE_DIVISOR 4u
#define SDR_SSB_RX_RATE (BOARD_AUDIO_SAMPLE_RATE_HZ / SDR_SSB_RX_SAMPLE_RATE_DIVISOR)

#define SDR_SSB_RX_BLOCK_SIZE (DRV_CODEC_BLOCK_FRAMES / SDR_SSB_RX_SAMPLE_RATE_DIVISOR)
#define SDR_SSB_TX_BLOCK_SIZE SDR_SSB_RX_BLOCK_SIZE
#define SDR_SSB_LEVEL_BLOCK_RATE ((float)SDR_SSB_RX_RATE / (float)SDR_SSB_RX_BLOCK_SIZE)

#define SDR_SSB_FILTER_NUM_STAGES SDR_IIR_STAGES_6

/* S-meter tap: short EMA on the per-block in-band power, peak-held between
 * meter reads. */
#define SDR_SSB_LEVEL_EMA_MS   3u
#define SDR_SSB_LEVEL_DB_OFFSET (-0.0f)

/* Squelch marker: the 400 Hz reference band of the low metric band (300..700).
 * s_state.filter_low/high give the user passband; the noise estimate is scaled
 * from that width down to the reference band. */
#define SDR_SSB_SQUELCH_REF_HZ 400.0f
/* Shifts the flat-noise metric to ~0; a voiced signal lands above it. */
#define SDR_SSB_SQUELCH_BIAS   7.0f

/* LPF filter */
static arm_biquad_casd_df1_inst_f32 s_lpf;
static volatile uint32_t s_lpf_id;
static uint32_t s_lpf_freq[2];
static float s_lpf_coeffs[2][SDR_SSB_FILTER_NUM_STAGES * 5];
static float s_lpf_state[SDR_SSB_FILTER_NUM_STAGES * 4];


/* HPF filter */
static arm_biquad_casd_df1_inst_f32 s_hpf;
static volatile uint32_t s_hpf_id;
static uint32_t s_hpf_freq[2];
static float s_hpf_coeffs[2][SDR_SSB_FILTER_NUM_STAGES * 5];
static float s_hpf_state[SDR_SSB_FILTER_NUM_STAGES * 4];

/* Squelch metric: two fixed band-pass filters on the pre-IIR tap, and the
 * per-block metric published to the squelch (ISR, same pass as the demod). */
static arm_biquad_casd_df1_inst_f32 s_metric_low;
static arm_biquad_casd_df1_inst_f32 s_metric_high;
static float s_metric_low_state[SDR_SQUELCH_SSB_LOW_STAGES * 4];
static float s_metric_high_state[SDR_SQUELCH_SSB_HIGH_STAGES * 4];
static float s_metric_tap[SDR_SSB_RX_BLOCK_SIZE] __attribute__((aligned(32)));
static float s_signal_metric;

/* S-meter: per-block in-band power (post-filter audio), EMA-smoothed and
 * peak-held between reads. The last dB value is held while no block runs (TX). */
static float s_level_alpha;
static float s_level_ema;
static float s_level_peak;
static float s_level_db_last = -127.0f;

static struct {
    SdrModulation rx_mode;
    AppRadioMode radio_mode;
    uint32_t filter_low;
    uint32_t filter_high;
} s_state;

static struct {
    bool fir_reset;
    bool lpf_reset;
    bool hpf_reset;
    bool lpf_switch;
    bool hpf_switch;
    bool metric_reset;
} s_pending;

static void update_lpf(void) {
    // Prevent switching from ISR
    uint32_t key = Bsp_Irq_SaveLow();
    s_pending.lpf_switch = false;
    if (s_lpf_freq[s_lpf_id] == s_state.filter_high) {
        Bsp_Irq_RestoreLow(key);
        return;
    }
    Bsp_Irq_RestoreLow(key);

    uint32_t shadow_id = s_lpf_id ^ 1u;

    if (s_lpf_freq[shadow_id] != s_state.filter_high) {
        SdrIirSpec spec = {
            .edge = SDR_IIR_EDGE_NOISE_BW,
            .f_edge_hz = s_state.filter_high,
            .fs_hz = (float)(SDR_SSB_RX_RATE),
            .stages = SDR_SSB_FILTER_NUM_STAGES,
            .type = SDR_IIR_TYPE_LPF
        };
        uint8_t stages;
        SdrIirResponse resp;
        int res = Sdr_Iir_DesignEllipEx(&spec, s_lpf_coeffs[shadow_id], &stages,
                                         &resp);
        if (res != 0) {
            LOG_WARN("Sdr_Iir_DesignEllipEx LPF error: %i", res);
            return;
        }
        LOG_DEBUG("iir lpf ref %.1f stop %.1f nbw %.1f gain %.2f level %.1f",
                  resp.f_ref_hz, resp.f_stop_hz, resp.noise_bw_hz,
                  resp.enbw_gain_db, resp.level_hz);
        s_lpf_freq[shadow_id] = s_state.filter_high;
    }
    key = Bsp_Irq_SaveLow();
    s_pending.lpf_switch = true;
    Bsp_Irq_RestoreLow(key);
}

static void update_hpf(void) {
    // Prevent switching from ISR
    uint32_t key = Bsp_Irq_SaveLow();
    s_pending.hpf_switch = false;
    if (s_hpf_freq[s_hpf_id] == s_state.filter_low) {
        Bsp_Irq_RestoreLow(key);
        return;
    }
    Bsp_Irq_RestoreLow(key);

    uint32_t shadow_id = s_hpf_id ^ 1u;

    if (s_hpf_freq[shadow_id] != s_state.filter_low) {
        SdrIirSpec spec = {
            .edge = SDR_IIR_EDGE_NOISE_BW,
            .f_edge_hz = s_state.filter_low,
            .fs_hz = (float)(SDR_SSB_RX_RATE),
            .stages = SDR_SSB_FILTER_NUM_STAGES,
            .type = SDR_IIR_TYPE_HPF
        };
        uint8_t stages;
        SdrIirResponse resp;
        int res = Sdr_Iir_DesignEllipEx(&spec, s_hpf_coeffs[shadow_id], &stages,
                                         &resp);
        if (res != 0) {
            LOG_WARN("Sdr_Iir_DesignEllipEx HPF error: %i", res);
            return;
        }
        LOG_INFO("iir hpf ref %.1f stop %.1f nbw %.1f level %.1f",
                  resp.f_ref_hz, resp.f_stop_hz, resp.noise_bw_hz,
                  resp.level_hz);
        s_hpf_freq[shadow_id] = s_state.filter_low;
    }
    key = Bsp_Irq_SaveLow();
    s_pending.hpf_switch = true;
    Bsp_Irq_RestoreLow(key);
}


/* Events subscriptions */

static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;
    uint32_t high = cfg->filter_high_hz > 0 ? (uint32_t)cfg->filter_high_hz
                                            : (uint32_t)(-cfg->filter_high_hz);
    uint32_t low  = cfg->filter_low_hz > 0 ? (uint32_t)cfg->filter_low_hz
                                           : (uint32_t)(-cfg->filter_low_hz);
    if (low > high) {
        uint32_t temp = low;
        low = high;
        high = temp;
    }
    bool lpf_changed = s_state.filter_high != high;
    bool hpf_changed = s_state.filter_low != low;
    s_state.filter_low  = low;
    s_state.filter_high = high;

    bool new_lsb = cfg->rx_mode == SDR_MOD_LSB || cfg->rx_mode == SDR_MOD_LSB_D;
    bool new_usb = cfg->rx_mode == SDR_MOD_USB || cfg->rx_mode == SDR_MOD_USB_D;
    bool old_lsb = s_state.rx_mode == SDR_MOD_LSB || s_state.rx_mode == SDR_MOD_LSB_D;
    bool old_usb = s_state.rx_mode == SDR_MOD_USB || s_state.rx_mode == SDR_MOD_USB_D;

    bool leaving_ssb = (old_lsb || old_usb) && (!new_lsb && !new_usb);
    bool entering_ssb = (!old_lsb && !old_usb) && (new_lsb || new_usb);
    bool ssb = new_lsb || new_usb;

    s_state.rx_mode = cfg->rx_mode;

    if (leaving_ssb) {
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.lpf_reset = true;
        s_pending.hpf_reset = true;
        s_pending.fir_reset = true;
        s_pending.metric_reset = true;
        Bsp_Irq_RestoreLow(key);
    } else if (entering_ssb) {
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.fir_reset = true;
        s_pending.metric_reset = true;
        Bsp_Irq_RestoreLow(key);
        update_hpf();
        update_lpf();
    } else if (ssb) {
        if (lpf_changed) {
            update_lpf();
        }
        if (hpf_changed) {
            update_hpf();
        }
    }
}

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (s_state.radio_mode == APP_RADIO_MODE_RX) {
        // Leaving RX
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.lpf_reset = true;
        s_pending.hpf_reset = true;
        s_pending.fir_reset = true;
        s_pending.metric_reset = true;
        Bsp_Irq_RestoreLow(key);
    } else {
        // Leaving TX/OFF
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.fir_reset = true;
        s_pending.metric_reset = true;
        Bsp_Irq_RestoreLow(key);
    }
    s_state.radio_mode = evt->mode;
}

void Sdr_SSB_Init(void)
{
    Sdr_Sideband_Init();

    s_lpf_id = 0;
    s_hpf_id = 0;
    s_state.filter_high = 2900;
    s_state.filter_low = 300;
    arm_biquad_cascade_df1_init_f32(&s_lpf, SDR_SSB_FILTER_NUM_STAGES,
        s_lpf_coeffs[s_lpf_id], s_lpf_state);
    arm_biquad_cascade_df1_init_f32(&s_hpf, SDR_SSB_FILTER_NUM_STAGES,
        s_hpf_coeffs[s_hpf_id], s_hpf_state);
    update_hpf();
    update_lpf();

    arm_biquad_cascade_df1_init_f32(&s_metric_low, SDR_SQUELCH_SSB_LOW_STAGES,
        sdr_squelch_ssb_low_coeffs, s_metric_low_state);
    arm_biquad_cascade_df1_init_f32(&s_metric_high, SDR_SQUELCH_SSB_HIGH_STAGES,
        sdr_squelch_ssb_high_coeffs, s_metric_high_state);
    s_signal_metric = 0.0f;
    s_pending.metric_reset = true;

    s_level_alpha = 1.0f - expf(-1.0f / ((float)SDR_SSB_LEVEL_EMA_MS *
                              SDR_SSB_LEVEL_BLOCK_RATE / 1000.0f));
    s_level_ema     = 0.0f;
    s_level_peak    = 0.0f;
    s_level_db_last = -127.0f;

    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
}

static void apply_pending(void) {
    if (s_pending.lpf_reset) {
        s_pending.lpf_reset = false;
        arm_fill_f32(0.0f, s_lpf_state, SDR_SSB_FILTER_NUM_STAGES * 4);
    }
    if (s_pending.hpf_reset) {
        s_pending.hpf_reset = false;
        arm_fill_f32(0.0f, s_hpf_state, SDR_SSB_FILTER_NUM_STAGES * 4);
    }
    if (s_pending.fir_reset) {
        s_pending.fir_reset = false;
        Sdr_Sideband_Reset();
        s_level_ema  = 0.0f;
        s_level_peak = 0.0f;
    }
    if (s_pending.lpf_switch) {
        s_pending.lpf_switch = false;
        s_lpf_id = s_lpf_id ^ 1;
        s_lpf.pCoeffs = s_lpf_coeffs[s_lpf_id];
    }
    if (s_pending.hpf_switch) {
        s_pending.hpf_switch = false;
        s_hpf_id = s_hpf_id ^ 1;
        s_hpf.pCoeffs = s_hpf_coeffs[s_hpf_id];
    }
    if (s_pending.metric_reset) {
        s_pending.metric_reset = false;
        arm_fill_f32(0.0f, s_metric_low_state, SDR_SQUELCH_SSB_LOW_STAGES * 4);
        arm_fill_f32(0.0f, s_metric_high_state, SDR_SQUELCH_SSB_HIGH_STAGES * 4);
    }
}


static void apply_filters_rx(float *audio, uint32_t size) {
    arm_biquad_cascade_df1_f32(&s_lpf, audio, audio, size);
    arm_biquad_cascade_df1_f32(&s_hpf, audio, audio, size);
}

static float block_peak(const float *x, uint32_t size) {
    float peak = 0.0f;

    for (uint32_t n = 0; n < size; ++n) {
        float mag = fabsf(x[n]);
        if (mag > peak) {
            peak = mag;
        }
    }
    return peak;
}

/* Squelch metric on the pre-IIR tap: a spectral-slope term (low vs high band)
 * plus the in-band SNR of the low band. Noise invalid -> 0 (no data). */
static void ssb_update_metric(const float *tap, uint32_t size) {
    float low_db;
    float high_db;
    float pasband;
    float band_noise_db;

    arm_copy_f32(tap, s_metric_tap, size);
    arm_biquad_cascade_df1_f32(&s_metric_low, s_metric_tap, s_metric_tap, size);
    float peak_low = block_peak(s_metric_tap, size);

    arm_copy_f32(tap, s_metric_tap, size);
    arm_biquad_cascade_df1_f32(&s_metric_high, s_metric_tap, s_metric_tap, size);
    float peak_high = block_peak(s_metric_tap, size);

    if (!Sdr_Spectrum_Is_Noise_Valid()) {
        s_signal_metric = 0.0f;
        return;
    }

    low_db  = Sdr_Lin2Db(peak_low);
    high_db = Sdr_Lin2Db(peak_high);

    pasband = (float)s_state.filter_high - (float)s_state.filter_low;
    if (pasband < 1.0f) {
        pasband = 1.0f;
    }
    band_noise_db = Sdr_Spectrum_Get_Demod_Noise_Est_Db() -
                    Sdr_Pow2Db(pasband / SDR_SSB_SQUELCH_REF_HZ);

    s_signal_metric = (low_db - band_noise_db) + (low_db - high_db) -
                       SDR_SSB_SQUELCH_BIAS;
}

/* Fold one block of in-band power into the S-meter: EMA for noise rejection,
 * then peak-hold so the level survives between meter reads. */
static void ssb_level_update(float power) {
    s_level_ema += (power - s_level_ema) * s_level_alpha;
    if (s_level_ema > s_level_peak) {
        s_level_peak = s_level_ema;
    }
}

static void process_ssb_demod(const Sdr_IQ_Data *data, float *audio, bool lower)
{
    if (!data || !audio || data->size != SDR_SSB_RX_BLOCK_SIZE){
        return;
    }
    apply_pending();

    Sdr_Sideband_Demod(data, audio, lower);

    ssb_update_metric(audio, data->size);
    apply_filters_rx(audio, data->size);

    /* In-band power after the selective IIR pair, i.e. within the user's
     * passband, not the wide product-detector output. */
    float power;
    arm_power_f32(audio, data->size, &power);
    ssb_level_update(power / (float)data->size);
}

void Sdr_LSB_Demod(const Sdr_IQ_Data *data, float *audio)
{
    process_ssb_demod(data, audio, true);
}

void Sdr_USB_Demod(const Sdr_IQ_Data *data, float *audio)
{
    process_ssb_demod(data, audio, false);
}

static void process_ssb_mod(const float *audio, Sdr_IQ_Data *data, bool lower)
{
    if (!data || !audio || data->size != SDR_SSB_TX_BLOCK_SIZE){
        return;
    }

    apply_pending();

    // Filters for audio should be already applied
    Sdr_Sideband_Mod(audio, data, lower);
}

void Sdr_LSB_Mod(const float *audio, Sdr_IQ_Data *data)
{
    process_ssb_mod(audio, data, true);
}

void Sdr_USB_Mod(const float *audio, Sdr_IQ_Data *data)
{
    process_ssb_mod(audio, data, false);
}

float Sdr_SSB_GetSignalMetric(void)
{
    return s_signal_metric;
}

float Sdr_SSB_Get_Rx_Level_Db(void)
{
    uint32_t key = Bsp_Irq_SaveLow();
    float    peak = s_level_peak;

    s_level_peak = 0.0f;
    Bsp_Irq_RestoreLow(key);

    /* peak == 0 means no demod block since the last read (TX): hold the last
     * value instead of publishing the empty-window floor. */
    if (peak > 0.0f) {
        s_level_db_last = Sdr_Pow2Db(peak) + SDR_SSB_LEVEL_DB_OFFSET;
    }
    return s_level_db_last;
}
