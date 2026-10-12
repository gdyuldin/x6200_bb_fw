#include "sdr_nb.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "arm_math.h"

#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_codec.h"

#define LOG_TAG "app.sdr.nb"

/* Delay line for the raw I/Q. Power of two, must cover width_pre + NB_RAMP_LEN
 * (the look-ahead) for the largest width. */
#define NB_BUF_SIZE    128u
/* Anti-click ramp length, in samples, on entry to and exit from a blank. */
#define NB_RAMP_LEN    8u

/* Noise-floor averaging window: NB_SMA_BASE samples at 48 kHz, scaled by M so
 * the physical time constant stays ~1 ms at every FPGA decimation. */
#define NB_SMA_BASE      64u
#define NB_SMA_SIZE_MAX 512u

/* Hard caps on a single blank, in time so they follow the sampling rate: the
 * mute is held at most NB_MAX_PULSE_MS, then the blanker waits NB_HOLD_OFF_MS
 * before it may trigger again (during which the entry threshold is raised). */
#define NB_MAX_PULSE_MS 2u
#define NB_HOLD_OFF_MS  8u

/* threshold -> k_on is geometric in base 2: k_on = 2^(offset + step * nbthr).
 * The offset shifts the whole host range and is the only knob to move on
 * hardware. */
#define NB_THR_OFFSET 1.0f
#define NB_THR_STEP   0.08f

_Static_assert((NB_BUF_SIZE & (NB_BUF_SIZE - 1u)) == 0u,
               "NB_BUF_SIZE must be a power of two");
_Static_assert((NB_SMA_SIZE_MAX & (NB_SMA_SIZE_MAX - 1u)) == 0u,
               "NB_SMA_SIZE_MAX must be a power of two");

/* One-shot capture of a single impulse for on-device tuning. Off in normal
 * builds; the log format is parsed by analysis/pulse/parse_pulse.py. */
#define NB_DEBUG_CAPTURE 0
#if NB_DEBUG_CAPTURE
#define NB_CAP_PRE    16u
#define NB_CAP_POST   48u
#define NB_CAP_LEN    (NB_CAP_PRE + NB_CAP_POST)
#define NB_CAP_CHUNK  8u
#define NB_CAP_TRIG_K 16.0f
#define NB_CAP_SETTLE 2048u
#endif

typedef struct {
    float re;
    float im;
} SdrNbIQ;

typedef struct {
    SdrNbIQ  data[NB_BUF_SIZE];
    uint32_t w;
} SdrNbDelayLine;

typedef enum {
    NB_STATE_WAIT,
    NB_STATE_ENTER_1,
    NB_STATE_ENTER_2,
    NB_STATE_MUTE,
    NB_STATE_EXIT,
} SdrNbState;

#if NB_DEBUG_CAPTURE
typedef struct {
    float    ring[NB_CAP_PRE];
    uint32_t ring_w;
    float    buf[NB_CAP_LEN];
    uint32_t post_left;
    uint32_t settle;
    uint32_t last_m;
    float    avg;
    float    trig_avg;
    bool     armed;
    bool     active;
} NbCapture;
#endif

/* Raised-cosine ramps, symmetric so the same transition looks the same on entry
 * and exit: enter fades the sample out to mute, exit fades it back to unity. */
static const float k_nb_enter[NB_RAMP_LEN] = {
    1.00000f, 0.95048f, 0.81174f, 0.61126f,
    0.38874f, 0.18826f, 0.04952f, 0.00000f,
};
static const float k_nb_exit[NB_RAMP_LEN] = {
    0.00000f, 0.04952f, 0.18826f, 0.38874f,
    0.61126f, 0.81174f, 0.95048f, 1.00000f,
};

static struct {
    SdrNbDelayLine dline;

    SdrNbState state;
    uint32_t   counter;
    float      pwr_avg_last;

    /* Derived by Sdr_NB_SetParams(): entry/exit thresholds and the blank width. */
    float    k_on;
    float    k_off;
    uint32_t width_pre;
    uint32_t width_post;
    uint32_t delay;

    /* Derived by nb_set_rate(): time caps converted to samples and the SMA. */
    uint32_t max_pulse;
    uint32_t hold_off;

    struct {
        float    buf[NB_SMA_SIZE_MAX];
        uint32_t mask;
        uint32_t w;
        uint32_t r;
        float    acc;
        float    k;
    } sma;

    uint32_t m;
    volatile bool enabled;
} s_nb;

#if NB_DEBUG_CAPTURE
static NbCapture s_cap;
#endif


static void nb_sma_reset(void) {
    s_nb.sma.w = 0u;
    s_nb.sma.r = 0u;
    s_nb.sma.acc = 0.0f;
}

static void nb_dline_reset(void) {
    s_nb.dline.w = 0u;
    memset(s_nb.dline.data, 0, sizeof(s_nb.dline.data));
}

static void nb_dline_put(float re, float im) {
    s_nb.dline.data[s_nb.dline.w] = (SdrNbIQ){re, im};
    s_nb.dline.w = (s_nb.dline.w + 1u) & (NB_BUF_SIZE - 1u);
}

static SdrNbIQ nb_dline_get(void) {
    uint32_t r = (s_nb.dline.w - s_nb.delay) & (NB_BUF_SIZE - 1u);
    return s_nb.dline.data[r];
}

/* Sliding sum of the last window samples; `buf` is overwritten before it is
 * ever read, so a reset only has to clear the indices. */
static void nb_sma_update(float pwr, float *pwr_avg) {
    s_nb.sma.acc += pwr;
    s_nb.sma.buf[s_nb.sma.w] = pwr;
    s_nb.sma.w = (s_nb.sma.w + 1u) & s_nb.sma.mask;

    float sum = s_nb.sma.acc;
    if (s_nb.sma.w == s_nb.sma.r) {
        s_nb.sma.acc -= s_nb.sma.buf[s_nb.sma.r];
        s_nb.sma.r = (s_nb.sma.r + 1u) & s_nb.sma.mask;
    }
    *pwr_avg = sum * s_nb.sma.k;
}

/* Recompute everything that follows the sampling rate when M changes. Runs in
 * the I2S ISR on a rate change only, never per sample. */
static void nb_set_rate(uint32_t m) {
    if (m == 0u) {
        return;
    }

    uint32_t n = NB_SMA_BASE * m;
    if (n > NB_SMA_SIZE_MAX) {
        n = NB_SMA_SIZE_MAX;
    }
    s_nb.sma.mask = n - 1u;
    s_nb.sma.k    = 1.0f / (float)n;
    nb_sma_reset();

    /* One 48 kHz frame per FPGA tick: frames = ms * 48 * m. */
    uint32_t frames_per_ms = BOARD_AUDIO_SAMPLE_RATE_HZ / 1000u;
    s_nb.max_pulse = NB_MAX_PULSE_MS * frames_per_ms * m;
    s_nb.hold_off  = NB_HOLD_OFF_MS * frames_per_ms * m;

    s_nb.state        = NB_STATE_WAIT;
    s_nb.counter      = s_nb.hold_off;
    s_nb.pwr_avg_last = 0.0f;

    s_nb.m = m;
}

#if NB_DEBUG_CAPTURE
static void cap_reset(void) {
    for (uint32_t i = 0u; i < NB_CAP_PRE; ++i) {
        s_cap.ring[i] = 0.0f;
    }
    s_cap.ring_w    = 0u;
    s_cap.post_left = 0u;
    s_cap.active    = false;
}

static void cap_arm(uint32_t m) {
    cap_reset();
    s_cap.avg    = 0.0f;
    s_cap.settle = NB_CAP_SETTLE;
    s_cap.armed  = true;
    LOG_INFO("---- fftdec changed: fs=%u Hz, m=%u, block=%u, capture armed",
             (unsigned)(BOARD_AUDIO_SAMPLE_RATE_HZ * m), (unsigned)m,
             (unsigned)(DRV_CODEC_BLOCK_FRAMES * m));
}

static void cap_print(void) {
    float peak = 0.0f;

    for (uint32_t i = 0u; i < NB_CAP_LEN; ++i) {
        if (s_cap.buf[i] > peak) {
            peak = s_cap.buf[i];
        }
    }

    LOG_INFO("impulse: fs=%u Hz, m=%u, avg=%g, peak=%g, ratio=%g, pre=%u, post=%u",
             (unsigned)(BOARD_AUDIO_SAMPLE_RATE_HZ * s_cap.last_m),
             (unsigned)s_cap.last_m, (double)s_cap.trig_avg, (double)peak,
             (double)(s_cap.trig_avg > 0.0f ? peak / s_cap.trig_avg : 0.0f),
             (unsigned)NB_CAP_PRE, (unsigned)NB_CAP_POST);

    for (uint32_t off = 0u; off < NB_CAP_LEN; off += NB_CAP_CHUNK) {
        LOG_INFO("env[%u]: %g %g %g %g %g %g %g %g",
                 (unsigned)off,
                 (double)s_cap.buf[off + 0u], (double)s_cap.buf[off + 1u],
                 (double)s_cap.buf[off + 2u], (double)s_cap.buf[off + 3u],
                 (double)s_cap.buf[off + 4u], (double)s_cap.buf[off + 5u],
                 (double)s_cap.buf[off + 6u], (double)s_cap.buf[off + 7u]);
    }
}

static void cap_push(float pwr) {
    if (s_cap.active) {
        s_cap.buf[NB_CAP_PRE + (NB_CAP_POST - s_cap.post_left)] = pwr;
        if (--s_cap.post_left == 0u) {
            s_cap.active = false;
            cap_print();
        }
        return;
    }

    s_cap.ring[s_cap.ring_w] = pwr;
    s_cap.ring_w = (s_cap.ring_w + 1u) % NB_CAP_PRE;

    if (!s_cap.armed) {
        return;
    }
    if (s_cap.settle > 0u) {
        s_cap.settle--;
        return;
    }
    if (s_cap.avg <= 0.0f || pwr <= s_cap.avg * NB_CAP_TRIG_K) {
        return;
    }

    /* Oldest-first copy of the pre-trigger window; the trigger sample is the
     * youngest entry, so it ends up at buf[NB_CAP_PRE - 1]. */
    for (uint32_t i = 0u; i < NB_CAP_PRE; ++i) {
        s_cap.buf[i] = s_cap.ring[(s_cap.ring_w + i) % NB_CAP_PRE];
    }
    s_cap.trig_avg  = s_cap.avg;
    s_cap.post_left = NB_CAP_POST;
    s_cap.active    = true;
    s_cap.armed     = false;
}
#endif


void Sdr_NB_Init(void) {
    s_nb.m        = 0u;
    s_nb.enabled  = false;
    s_nb.state    = NB_STATE_WAIT;
    s_nb.counter  = 0u;
    s_nb.pwr_avg_last = 0.0f;

    /* Start with the default width and the lowest threshold; the host sends the
     * real values with the first command block. */
    Sdr_NB_SetParams(false, 24u, 0u);
}

/* Called from the main loop while Sdr_NB_Process() runs in the I2S DMA ISR.
 * The transcendental work stays outside the critical section; only the state
 * swap is serialized, so the ISR never sees a half-applied configuration. */
void Sdr_NB_SetParams(bool enabled, uint8_t width, uint8_t threshold) {
    float k_on  = exp2f(NB_THR_OFFSET + NB_THR_STEP * (float)threshold);
    float k_off = 1.0f + (k_on - 1.0f) * 0.5f;

    uint32_t w          = (uint32_t)width + 1u;
    uint32_t width_pre  = MIN(w / 2u + 5u, NB_BUF_SIZE / 2u - 1u);
    uint32_t width_post = w;

    uint32_t key = Bsp_Irq_SaveLow();

    bool transition = (enabled != s_nb.enabled);
    s_nb.enabled = enabled;

    if (enabled) {
        s_nb.k_on       = k_on;
        s_nb.k_off      = k_off;
        s_nb.width_pre  = width_pre;
        s_nb.width_post = width_post;
        s_nb.delay      = width_pre + NB_RAMP_LEN;
    }

    if (transition) {
        /* A fresh start must not leak the buffered I/Q of the previous state. */
        nb_dline_reset();
        nb_sma_reset();
        s_nb.state        = NB_STATE_WAIT;
        s_nb.counter      = s_nb.hold_off;
        s_nb.pwr_avg_last = 0.0f;
    }

    Bsp_Irq_RestoreLow(key);
}

void Sdr_NB_Process(const Sdr_IQ_Data *iq) {
    uint32_t m = iq->size / DRV_CODEC_BLOCK_FRAMES;

    if (m != s_nb.m) {
        nb_set_rate(m);
#if NB_DEBUG_CAPTURE
        s_cap.last_m = m;
        cap_arm(m);
#endif
    }

    if (!s_nb.enabled) {
#if NB_DEBUG_CAPTURE
        for (uint32_t n = 0u; n < iq->size; ++n) {
            cap_push(iq->i[n] * iq->i[n] + iq->q[n] * iq->q[n]);
        }
#endif
        return;
    }

    for (uint32_t n = 0u; n < iq->size; ++n) {
        float re  = iq->i[n];
        float im  = iq->q[n];
        float pwr = re * re + im * im;

        nb_dline_put(re, im);

        float pwr_avg;
        float scale = 1.0f;
        float k     = s_nb.k_on;

        nb_sma_update(pwr, &pwr_avg);

        switch (s_nb.state) {
        case NB_STATE_WAIT:
            if (s_nb.counter > 0u) {
                s_nb.counter--;
                k *= 4.0f;
            }
            if (pwr > pwr_avg * k) {
                s_nb.state   = NB_STATE_ENTER_1;
                s_nb.counter = 0u;
            } else {
                s_nb.pwr_avg_last = pwr_avg;
            }
            break;

        case NB_STATE_ENTER_1:
            scale = k_nb_enter[s_nb.counter];
            s_nb.counter++;
            if (s_nb.counter == NB_RAMP_LEN) {
                s_nb.state   = NB_STATE_ENTER_2;
                s_nb.counter = 0u;
            }
            break;

        case NB_STATE_ENTER_2:
            scale = 0.0f;
            s_nb.counter++;
            if (s_nb.counter == s_nb.width_pre) {
                s_nb.state   = NB_STATE_MUTE;
                s_nb.counter = 0u;
            }
            break;

        case NB_STATE_MUTE:
            scale = 0.0f;
            s_nb.counter++;
            if (s_nb.counter >= s_nb.width_post) {
                if (pwr < s_nb.pwr_avg_last * s_nb.k_off ||
                    s_nb.counter >= s_nb.max_pulse) {
                    s_nb.state   = NB_STATE_EXIT;
                    s_nb.counter = 0u;
                }
            }
            break;

        case NB_STATE_EXIT:
            scale = k_nb_exit[s_nb.counter];
            s_nb.counter++;
            if (s_nb.counter == NB_RAMP_LEN) {
                s_nb.state   = NB_STATE_WAIT;
                s_nb.counter = s_nb.hold_off;
            }
            break;

        default:
            break;
        }

        SdrNbIQ d = nb_dline_get();
        if (scale == 0.0f) {
            d.re = 0.0f;
            d.im = 0.0f;
        } else if (scale != 1.0f) {
            d.re *= scale;
            d.im *= scale;
        }
        iq->i[n] = d.re;
        iq->q[n] = d.im;

#if NB_DEBUG_CAPTURE
        cap_push(pwr);
#endif
    }
}
