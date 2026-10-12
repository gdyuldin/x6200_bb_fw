#include "sdr_spectrum.h"

#include <stdint.h>
#include <string.h>

#include "arm_math.h"
#include "dsp/window_functions.h"

#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_memory.h"
#include "bsp_prof.h"
#include "drv_codec.h"
#include "drv_fpga.h"
#include "sdr_iir_decim.h"
#include "sdr_resampling_filters.h"

#define LOG_TAG "app.sdr.spectrum"

#define Q31_MAX_INV (1.0f / Q31_ABSMAX)
#define SDR_SPECTRUM_RX_DB_OFFSET (-47.0f)
/* TX display offset, applied after the dB conversion. It is intentionally
 * independent of the RX fftdec-derived offset: the TX branch always runs at
 * 48 kHz. Initial value equals the base RX offset; tune it by measurement on
 * the device. */
#define SDR_SPECTRUM_TX_DB_OFFSET (-50.0f)
/* Calibration offsets that map the spectrum noise estimate (dB, 10*log10 of the
 * in-band power) into the demodulated-audio (peak, 20log) domain, indexed by
 * modulation group. Measured on the device in silence; scatter ~±2 dB and
 * ~1.5 dB drift across the narrow CW filter width. The AGC and the squelch
 * share this estimate (see Sdr_Spectrum_Get_Demod_Noise_Est_Db). */
#define SDR_SPECTRUM_NOISE_SPEC_OFFSET_CW  (-54.0f)
#define SDR_SPECTRUM_NOISE_SPEC_OFFSET_SSB (-44.0f)
#define SDR_SPECTRUM_NOISE_SPEC_OFFSET_AM  (-52.0f)
#define SDR_SPECTRUM_IQ_BUF_FLOATS 4096u
/* Keep the raw read one I2S block behind the FPGA write index so the last,
 * possibly partial, block is never read. */
#define SDR_SPECTRUM_RAW_LAG DRV_CODEC_BLOCK_FRAMES

static float s_fft_buf[SDR_SPECTRUM_FFT_SIZE * 2u] BSP_RAM_D1_BSS_32;
static float s_hann_window[SDR_SPECTRUM_FFT_SIZE * 2u] BSP_RAM_D1_BSS_32;
static float s_db[SDR_SPECTRUM_FFT_SIZE] BSP_RAM_D1_BSS_32;
static float s_raw[SDR_SPECTRUM_FFT_SIZE] BSP_RAM_D1_BSS_32;
static float s_iq_buf[SDR_SPECTRUM_IQ_BUF_FLOATS] BSP_RAM_D1_BSS_32;

/* Constant driver view of the raw ring; set once in Sdr_Spectrum_Init. */
static const DrvFpgaRawWindow *s_raw_iq;

static volatile uint32_t s_iq_w;
static uint32_t          s_raw_r;
static uint32_t          s_raw_gen_seen;
static volatile bool     s_capture_paused;
/* TX mode: the raw ring is not read; the DMA-side AppendTx fills s_iq_buf with
 * the generated TX signal instead. Set/cleared under Bsp_Irq_SaveLow. */
static volatile bool     s_tx_mode;
static volatile bool     s_resync_pending;

static float s_meter = -127.0f;

/* Noise-floor estimator state. s_noise_bins is the per-bin power average; the
 * rest is published to the AGC (see Sdr_Spectrum_Get_Noise_Db). */
static float          s_noise_bins[SDR_SPECTRUM_FFT_SIZE] BSP_RAM_D1_BSS_32;
static volatile float s_noise_db;
static volatile float s_noise_lin;
static volatile bool  s_noise_valid;
static volatile uint32_t s_noise_skip;
static uint32_t       s_noise_hold;
static bool           s_noise_warm;
static float          s_noise_alpha;

/* Runtime configuration pushed from App_RadioState. The accumulation path runs
 * from the I2S DMA ISR, so keep the fields volatile until decimation/filtering
 * actually consumes them there. */
static volatile SdrModulation s_modulation;
static volatile uint8_t       s_fftdec;
static volatile int32_t       s_filter_low_hz;
static volatile int32_t       s_filter_high_hz;

static uint32_t s_bin_from = 0;
static uint32_t s_bin_to = 1;

static arm_cfft_instance_f32 s_cfft;

/* Spectrum zoom. A cascade of decimation-by-2 IIR stages runs on the raw stream
 * in the append path; s_zoom_active is the number of active stages (0 = no
 * decimation). I and Q are filtered by separate instances that share one scratch
 * buffer, since the stages run one after another. s_zoom_a/s_zoom_b are the
 * ping-pong chain buffers and only need half the input size (the first stage
 * output). */
static volatile uint8_t s_fftzoomcw;
static volatile uint8_t s_zoom_active;
static SdrIirDecim s_zoom_iir[SDR_SPECTRUM_ZOOM_MAX_STAGES][2];
static float s_zoom_state[SDR_SPECTRUM_ZOOM_MAX_STAGES][2]
                         [SDR_SPECTRUM_ZOOM_IIR_SECTIONS * 4u];
static float s_zoom_scratch[DRV_FPGA_RAW_WINDOW_SAMPLES] BSP_RAM_D1_BSS_32;
static float s_zoom_i_in[DRV_FPGA_RAW_WINDOW_SAMPLES] BSP_RAM_D1_BSS_32;
static float s_zoom_q_in[DRV_FPGA_RAW_WINDOW_SAMPLES] BSP_RAM_D1_BSS_32;
static float s_zoom_a[DRV_FPGA_RAW_WINDOW_SAMPLES / 2u] BSP_RAM_D1_BSS_32;
static float s_zoom_b[DRV_FPGA_RAW_WINDOW_SAMPLES / 2u] BSP_RAM_D1_BSS_32;

/* fftdec the display actually runs at: the TX branch is fixed at the maximum
 * fftdec (48 kHz), RX uses the committed RX value. */
static uint8_t effective_fftdec(void) {
    return s_tx_mode ? DRV_FPGA_FFTDEC_MAX : s_fftdec;
}

/* Sample rate the display FFT actually runs at: the FPGA rate halved once per
 * active zoom stage. */
static uint32_t effective_rate_hz(void) {
    uint32_t rate = BOARD_AUDIO_SAMPLE_RATE_HZ *
                    (DRV_FPGA_SAMPLES_PER_TICK_BASE >> effective_fftdec());

    return rate >> s_zoom_active;
}

/* Map a modulation onto the calibration group of its demodulated-audio noise. */
static float noise_spec_offset(SdrModulation modulation) {
    switch (modulation) {
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        return SDR_SPECTRUM_NOISE_SPEC_OFFSET_CW;
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        return SDR_SPECTRUM_NOISE_SPEC_OFFSET_SSB;
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        return SDR_SPECTRUM_NOISE_SPEC_OFFSET_AM;
    default:
        return SDR_SPECTRUM_NOISE_SPEC_OFFSET_SSB;
    }
}

static void update_filters(void) {
    int32_t bw = (int32_t)effective_rate_hz();
    int32_t freq_from, freq_to;
    int32_t nbfm_half = (s_filter_high_hz - s_filter_low_hz) / 2;
    if (nbfm_half < 0) {
        nbfm_half = -nbfm_half;
    } else if (nbfm_half == 0) {
        nbfm_half = 7500;
    }
    switch (s_modulation)
    {
    case SDR_MOD_LSB:
    case SDR_MOD_LSB_D:
        freq_from = -s_filter_high_hz;
        freq_to = -s_filter_low_hz;
        break;
    case SDR_MOD_USB:
    case SDR_MOD_USB_D:
        freq_from = s_filter_low_hz;
        freq_to = s_filter_high_hz;
        break;

    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        freq_from = s_filter_low_hz;
        freq_to = s_filter_high_hz;
        break;

    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        freq_from = s_filter_low_hz;
        freq_to = s_filter_high_hz;
        break;

    case SDR_MOD_NFM:
        freq_from = -nbfm_half;
        freq_to   = nbfm_half;
        break;

    case SDR_MOD_WFM:
        freq_from = -45000;
        freq_to = 45000;
        break;
    default:
        break;
    }
    s_bin_from = (freq_from * (int32_t)SDR_SPECTRUM_FFT_SIZE) / bw + SDR_SPECTRUM_FFT_SIZE / 2;
    s_bin_to = (freq_to * (int32_t)SDR_SPECTRUM_FFT_SIZE) / bw + SDR_SPECTRUM_FFT_SIZE / 2;
}

/* Zoom stages the display actually runs: only in RX and only for CW/CWR. */
static uint8_t zoom_effective(void) {
    if (s_tx_mode) {
        return 0u;
    }
    switch (s_modulation) {
    case SDR_MOD_CW:
    case SDR_MOD_CWR:
        return s_fftzoomcw;
    default:
        return 0u;
    }
}

static void zoom_reset_stages(uint8_t from, uint8_t to) {
    for (uint8_t s = from; s < to; ++s) {
        Sdr_IirDecim_Reset(&s_zoom_iir[s][0]);
        Sdr_IirDecim_Reset(&s_zoom_iir[s][1]);
    }
}

/* Adopt the effective zoom stage count. Only the newly activated stages are
 * reset: the lower stages stay valid. Any change invalidates the accumulated
 * data, which is at the previous rate. */
static void zoom_reconfigure(void) {
    uint8_t  want = zoom_effective();
    uint32_t key;

    if (want == s_zoom_active) {
        return;
    }

    key = Bsp_Irq_SaveLow();
    if (want > s_zoom_active) {
        zoom_reset_stages(s_zoom_active, want);
    }
    s_zoom_active = want;
    s_iq_w        = 0u;
    s_noise_skip  = SDR_SPECTRUM_NOISE_SKIP_FRAMES;
    s_noise_warm  = false;
    Bsp_Irq_RestoreLow(key);

    update_filters();
}

void Sdr_Spectrum_SetModulation(SdrModulation modulation) {
    s_modulation = modulation;
    zoom_reconfigure();
    update_filters();
}

void Sdr_Spectrum_SetFftdec(uint8_t fftdec) {
    /* RX value only. In TX the display keeps the TX offset and TX fftdec, so a
     * host rate change while transmitting does not disturb the display. */
    s_fftdec = fftdec;
    if (!s_tx_mode) {
        update_filters();
    }
}

void Sdr_Spectrum_SetFftzoomcw(uint8_t n) {
    if (n > SDR_SPECTRUM_ZOOM_MAX_STAGES) {
        n = SDR_SPECTRUM_ZOOM_MAX_STAGES;
    }
    s_fftzoomcw = n;
    zoom_reconfigure();
}

void Sdr_Spectrum_SetFilter(int32_t low_hz, int32_t high_hz) {
    s_filter_low_hz  = low_hz;
    s_filter_high_hz = high_hz;
    update_filters();
}

void Sdr_Spectrum_NotifyRetune(void) {
    s_noise_hold = SDR_SPECTRUM_NOISE_RETUNE_FRAMES;
}

void Sdr_Spectrum_SetTxMode(bool tx) {
    uint32_t key = Bsp_Irq_SaveLow();

    s_tx_mode = tx;
    if (tx) {
        /* Drop RX accumulation; the TX signal is fed by AppendTx. */
        s_iq_w           = 0u;
        s_resync_pending = false;
    } else {
        /* The next Process call resyncs to the raw ring write position instead
         * of reading data left over from before TX. */
        s_resync_pending = true;
    }
    Bsp_Irq_RestoreLow(key);

    /* Zoom is RX/CW-only, so entering or leaving TX changes the effective stage
     * count. Recompute the bin mapping with the new effective fftdec and zoom. */
    zoom_reconfigure();
    update_filters();
}

void Sdr_Spectrum_AppendTx(const float *i, const float *q, uint32_t count) {
    uint32_t room;

    if (!s_tx_mode || s_capture_paused || i == NULL || q == NULL ||
        count == 0u) {
        return;
    }

    room = SDR_SPECTRUM_IQ_BUF_FLOATS - s_iq_w;
    if (count * 2u > room) {
        count = room / 2u;
    }
    for (uint32_t n = 0; n < count; ++n) {
        s_iq_buf[s_iq_w++] = i[n];
        s_iq_buf[s_iq_w++] = q[n];
    }
}

/* Run one channel through `stages` decimation-by-2 stages. Returns the final
 * decimated buffer; its length is in_size >> stages. The ping-pong buffers are
 * reused between the I and Q passes, so the caller must consume each result
 * before the next call. */
static float *zoom_run_channel(uint8_t channel, uint8_t stages,
                               const float *in, uint32_t in_size) {
    const float *src   = in;
    float       *dst   = s_zoom_a;
    float       *spare = s_zoom_b;
    uint32_t     n     = in_size;

    for (uint8_t s = 0; s < stages; ++s) {
        Sdr_IirDecim_Process(&s_zoom_iir[s][channel], src, dst, n);
        n /= 2u;
        src = dst;
        float *t = dst;
        dst   = spare;
        spare = t;
    }
    return (float *)src;
}

/* Frontend: Q1.31 -> float. Without zoom the raw stream is interleaved straight
 * into the accumulation buffer; with zoom it is decimated first. */
static void spectrum_append(uint32_t from,
                            uint32_t count) {
    uint32_t room = SDR_SPECTRUM_IQ_BUF_FLOATS - s_iq_w;
    uint32_t stages = s_zoom_active;

    if (stages == 0u) {
        if (count * 2u > room) {
            count = room / 2u;
        }
        for (uint32_t n = 0; n < count; ++n) {
            uint32_t idx = (from + n) & (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u);

            s_iq_buf[s_iq_w++] = (float)s_raw_iq->i[idx] * Q31_MAX_INV;
            s_iq_buf[s_iq_w++] = (float)s_raw_iq->q[idx] * Q31_MAX_INV;
        }
        return;
    }

    /* Decimate the whole raw block to keep the IIR state continuous, but only
     * store what fits. */
    uint32_t out_len = count >> stages;
    uint32_t store   = out_len;
    if (store * 2u > room) {
        store = room / 2u;
    }

    for (uint32_t n = 0; n < count; ++n) {
        uint32_t idx = (from + n) & (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u);

        s_zoom_i_in[n] = (float)s_raw_iq->i[idx] * Q31_MAX_INV;
        s_zoom_q_in[n] = (float)s_raw_iq->q[idx] * Q31_MAX_INV;
    }

    uint32_t base = s_iq_w;
    const float *out_i = zoom_run_channel(0u, (uint8_t)stages, s_zoom_i_in, count);
    for (uint32_t m = 0; m < store; ++m) {
        s_iq_buf[base + 2u * m] = out_i[m];
    }
    const float *out_q = zoom_run_channel(1u, (uint8_t)stages, s_zoom_q_in, count);
    for (uint32_t m = 0; m < store; ++m) {
        s_iq_buf[base + 2u * m + 1u] = out_q[m];
    }
    s_iq_w += store * 2u;
}

void Sdr_Spectrum_Process(void) {
    uint32_t         raw_wr;
    uint32_t         delta;
    uint32_t         count;
    uint32_t         gen;

    /* In TX the raw ring is not updated; the generated TX signal is appended by
     * Sdr_Spectrum_AppendTx from the same DMA callback. */
    if (s_tx_mode) {
        return;
    }

    if (s_capture_paused) {
        return;
    }

    if (s_resync_pending) {
        /* Returning to RX: start from the current raw write position so data
         * from before TX is never read. */
        s_resync_pending = false;
        s_iq_w           = 0u;
        s_raw_r          = Drv_Fpga_GetPos();
        s_raw_gen_seen   = Drv_Fpga_GetRawGeneration();
        s_capture_paused = false;
        s_noise_skip     = SDR_SPECTRUM_NOISE_SKIP_FRAMES;
        return;
    }

    gen = Drv_Fpga_GetRawGeneration();
    if (gen != s_raw_gen_seen) {
        /* Raw ring invalidated (fftdec changed and control frame sent).
         * Drop accumulated data and start from the current write position. */
        s_raw_gen_seen   = gen;
        s_iq_w           = 0u;
        s_raw_r          = Drv_Fpga_GetPos();
        s_capture_paused = false;
        s_noise_skip     = SDR_SPECTRUM_NOISE_SKIP_FRAMES;
        /* The input rate changed; restart the active zoom stages. */
        zoom_reset_stages(0u, s_zoom_active);
        return;
    }

    raw_wr = Drv_Fpga_GetPos();
    delta  = (raw_wr - s_raw_r) & (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u);
    if (delta <= SDR_SPECTRUM_RAW_LAG) {
        return;
    }

    count  = delta - SDR_SPECTRUM_RAW_LAG;
    /* The zoom cascade decimates by 2 per stage anchored on the block it is
     * handed (Sdr_IirDecim_Process keeps every 2nd sample from the block start),
     * so the decimation grid only stays uniform while each appended block is a
     * multiple of 2^stages. A single sample of block-to-block jitter (lost or
     * doubled FPGA tick) shifts the grid and splatters the FFT. Round the block
     * down and leave the sub-grid remainder in the ring for the next call; at
     * zoom 0 the alignment is 1 and this is a no-op. */
    count -= count % (1u << s_zoom_active);
    if (count == 0u) {
        return;
    }

    spectrum_append(s_raw_r, count);
    s_raw_r = (s_raw_r + count) & (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u);
}

/* Independent noise-floor estimate from the RX spectrum. Per-bin power is
 * averaged, then the minimum 32-bin window inside the central span is taken as
 * the noise reference, normalised to 1 Hz and scaled to the demodulation
 * passband. Runs in the spectrum task context; the result is published for the
 * AGC (read from the I2S DMA ISR). */
static void noise_update(void) {
    uint32_t bw;
    uint32_t i;
    float    bin_hz;
    float    band_hz;
    float    sum;
    float    min_sum;
    float    power;
    bool     have;

    if (s_noise_skip > 0u) {
        s_noise_skip--;
        return;
    }

    if (s_noise_hold > 0u) {
        s_noise_hold--;
        return;
    }

    /* Seed on the first frame instead of averaging from zero, so the window
     * minimum is not biased down by untouched bins. Statistics are not reset on
     * a generation change: the spectrum is the same, only the first frame after
     * the switch is skipped to avoid its transient. */
    if (!s_noise_warm) {
        memcpy(s_noise_bins, s_raw, sizeof(s_noise_bins));
        s_noise_warm = true;
    } else {
        for (i = 0u; i < SDR_SPECTRUM_FFT_SIZE; i++) {
            s_noise_bins[i] += (s_raw[i] - s_noise_bins[i]) * s_noise_alpha;
        }
    }

    sum     = 0.0f;
    min_sum = 0.0f;
    have    = false;
    for (i = SDR_SPECTRUM_NOISE_EDGE_BINS;
         i < SDR_SPECTRUM_FFT_SIZE - SDR_SPECTRUM_NOISE_EDGE_BINS; i++) {
        sum += s_noise_bins[i];
        if (i >= SDR_SPECTRUM_NOISE_EDGE_BINS + SDR_SPECTRUM_NOISE_WINDOW_BINS) {
            sum -= s_noise_bins[i - SDR_SPECTRUM_NOISE_WINDOW_BINS];
        }
        if (i + 1u >= SDR_SPECTRUM_NOISE_EDGE_BINS +
                       SDR_SPECTRUM_NOISE_WINDOW_BINS) {
            if (!have || sum < min_sum) {
                min_sum = sum;
                have    = true;
            }
        }
    }

    if (!have || min_sum <= 0.0f) {
        s_noise_valid = false;
        return;
    }

    bw     = effective_rate_hz();
    bin_hz = (float)bw / (float)SDR_SPECTRUM_FFT_SIZE;

    band_hz = (float)(s_bin_to - s_bin_from + 1u) * bin_hz;
    if (band_hz < bin_hz) {
        band_hz = bin_hz;
    } else if (band_hz > (float)bw) {
        band_hz = (float)bw;
    }

    power = min_sum / ((float)SDR_SPECTRUM_NOISE_WINDOW_BINS * bin_hz) * band_hz;
    s_noise_db = Sdr_Pow2Db(power);
    s_noise_lin = sqrtf(power);
    s_noise_valid = true;
}

static void spectrum_calc(const float *src) {
    bool  tx     = s_tx_mode;
    /* Each fftdec step and each zoom stage halves the sample rate, so the
     * per-bin power of the decimated stream drops by a factor of two;
     * compensate to keep the displayed level independent of the selected
     * bandwidth. */
    float offset = tx ? SDR_SPECTRUM_TX_DB_OFFSET
                      : SDR_SPECTRUM_RX_DB_OFFSET +
                            3.0f * (float)(s_fftdec + s_zoom_active);

    memcpy(s_fft_buf, src, sizeof(s_fft_buf));
    arm_mult_f32(s_fft_buf, s_hann_window, s_fft_buf,
                 SDR_SPECTRUM_FFT_SIZE * 2u);
    arm_cfft_f32(&s_cfft, s_fft_buf, 0, 1);
    for (size_t i = 0; i < SDR_SPECTRUM_FFT_SIZE; i++) {
        float v = s_fft_buf[2 * i] * s_fft_buf[2 * i] +
                  s_fft_buf[2 * i + 1] * s_fft_buf[2 * i + 1];

        if (v < 1e-12f) {
            v = 1e-12f;
        }
        s_raw[(i + SDR_SPECTRUM_FFT_SIZE / 2) & (SDR_SPECTRUM_FFT_SIZE - 1)] = v;

        v = Sdr_Pow2Db(v);
        v += offset;
        s_db[(i + SDR_SPECTRUM_FFT_SIZE / 2) & (SDR_SPECTRUM_FFT_SIZE - 1)] = v;
    }
    /* S-meter and noise floor are RX measurements; in TX keep the last RX
     * values and mark the noise estimate invalid for the AGC fallback. */
    if (!tx) {
        float p = 0.0f;
        for (size_t i = s_bin_from; i <= s_bin_to; i++) {
            p += s_raw[i];
        }
        if (p == 0.0f) p = 1e-4f;
        s_meter = Sdr_Pow2Db(p) + SDR_SPECTRUM_RX_DB_OFFSET;

        noise_update();
    } else {
        s_noise_valid = false;
    }
}

/* 40 ms application task: pause the DMA-side accumulation, FFT the last
 * SDR_SPECTRUM_FFT_SIZE complex samples and notify telemetry. */
static void Sdr_Spectrum_Task(void) {
    uint32_t key;
    uint32_t w;
    uint32_t start;

    BSP_PROF_SECTION_BEGIN(BSP_PROF_SECTION_SPECTRUM, prof_start);

    /*
     * The audio DMA ISR (priority 1) appends to s_iq_buf and advances s_iq_w;
     * the FPGA PE10 ISR (priority 0) only fills the raw ring. Bsp_Irq_SaveLow()
     * masks the appender without stopping the FPGA tick. The pause/window latch
     * below is a few instructions long, so it is not heavy work under mask.
     */
    key = Bsp_Irq_SaveLow();
    s_capture_paused = true;
    w = s_iq_w;
    Bsp_Irq_RestoreLow(key);

    if (w < SDR_SPECTRUM_FFT_SIZE * 2u) {
        /* Not enough samples yet: with zoom the decimated rate is lower, so a
         * frame is collected across several periods. Keep accumulating. */
        s_capture_paused = false;
        BSP_PROF_SECTION_END(BSP_PROF_SECTION_SPECTRUM, prof_start);
        return;
    }

    start = w - SDR_SPECTRUM_FFT_SIZE * 2u;
    spectrum_calc(&s_iq_buf[start]);

    App_Scheduler_Notify(APP_SCHED_EV_SPECTRUM_READY);

    s_iq_w = 0u;
    s_capture_paused = false;
    BSP_PROF_SECTION_END(BSP_PROF_SECTION_SPECTRUM, prof_start);
}

void Sdr_Spectrum_Init(void) {
    /* Generate the half-window into the first half of s_hann_window, then
     * expand it in place from the end. Writing 2*i and 2*i+1 while walking
     * downwards never overwrites an element that has not been read yet, so no
     * extra 2 KB stack buffer is needed. */
    arm_hanning_f32(s_hann_window, SDR_SPECTRUM_FFT_SIZE);
    for (size_t i = SDR_SPECTRUM_FFT_SIZE; i-- > 0u;) {
        float v = s_hann_window[i];
        s_hann_window[2 * i] = v;
        s_hann_window[2 * i + 1] = v;
    }

    arm_cfft_init_f32(&s_cfft, SDR_SPECTRUM_FFT_SIZE);

    s_raw_iq = Drv_Fpga_GetRawWindow();
    s_iq_w = 0u;
    s_raw_r = Drv_Fpga_GetPos();
    s_raw_gen_seen = Drv_Fpga_GetRawGeneration();
    s_capture_paused = false;
    s_tx_mode = false;
    s_resync_pending = false;

    s_noise_alpha = 1.0f - expf(-1.0f / (SDR_SPECTRUM_NOISE_AVG_TAU_S *
                                ((float)1000.0f / (float)SDR_SPECTRUM_PERIOD_MS)));
    s_noise_db    = 0.0f;
    s_noise_valid = false;
    s_noise_warm  = false;
    s_noise_skip  = 0u;
    s_noise_hold  = 0u;

    for (uint8_t s = 0u; s < SDR_SPECTRUM_ZOOM_MAX_STAGES; ++s) {
        for (uint8_t ch = 0u; ch < 2u; ++ch) {
            Sdr_IirDecim_Init(&s_zoom_iir[s][ch], sdr_spectrum_zoom_iir_sos,
                              SDR_SPECTRUM_ZOOM_IIR_SECTIONS,
                              s_zoom_state[s][ch], s_zoom_scratch);
        }
    }
    s_fftzoomcw   = 0u;
    s_zoom_active = 0u;

    if (App_Scheduler_Add(Sdr_Spectrum_Task, SDR_SPECTRUM_PERIOD_MS) != 0) {
        LOG_WARN("spectrum task not registered");
    }
}

const float * Sdr_Spectrum_Get_Db(void) {
    return s_db;
}

const float Sdr_Spectrum_Get_S_meter(void) {
    return s_meter;
}

float Sdr_Spectrum_Get_Noise_Db(void) {
    return s_noise_db;
}

float Sdr_Spectrum_Get_Noise_Lin(void) {
    return s_noise_lin;
}

bool Sdr_Spectrum_Is_Noise_Valid(void) {
    return s_noise_valid;
}

float Sdr_Spectrum_Get_Demod_Noise_Est_Db(void) {
    return s_noise_db + noise_spec_offset(s_modulation);
}
