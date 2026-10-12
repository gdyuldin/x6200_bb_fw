#include "sdr_cw.h"

#include <stdint.h>

#include <arm_math.h>

#include "board_config.h"
#include "drv_codec.h"
#include "bsp_board.h"
#include "bsp_log.h"
#include "bsp_irq.h"
#include "bsp_exti.h"
#include "bsp_gpio.h"
#include "bsp_time.h"
#include "app_event.h"
#include "app_ptt.h"
#include "app_radio.h"
#include "sdr_iir.h"
#include "sdr_sideband.h"
#include "sdr_spectrum.h"
#include "main.h"

#define LOG_TAG "app.sdr.cw"


/* ------------------------------------------------------------------------- */
/* Constants                                                                 */
/* ------------------------------------------------------------------------- */

#define SDR_CW_RX_SAMPLE_RATE_DIVISOR 4u
#define SDR_CW_RX_BLOCK_SIZE (DRV_CODEC_BLOCK_FRAMES / SDR_CW_RX_SAMPLE_RATE_DIVISOR)
#define SDR_CW_RX_RATE (BOARD_AUDIO_SAMPLE_RATE_HZ / SDR_CW_RX_SAMPLE_RATE_DIVISOR)
#define SDR_CW_LEVEL_BLOCK_RATE ((float)SDR_CW_RX_RATE / (float)SDR_CW_RX_BLOCK_SIZE)

/* S-meter tap: short EMA on the per-block in-band power, peak-held between
 * meter reads so a 50 WPM dot (shorter than the telemetry period) is not
 * averaged against the following space. */
#define SDR_CW_LEVEL_EMA_MS   3u
#define SDR_CW_LEVEL_DB_OFFSET (+3.0f)

#define SDR_CW_FILTER_NUM_STAGES SDR_IIR_STAGES_6

/* K = lag + artificial delay. The full lag range requires K = BLOCK - 1. */
#define SDR_CW_EVENT_DELAY_FRAMES 63u

#define SDR_CW_EDGE_RAMP_MS 3u
#define SDR_CW_EDGE_RAMP_LEN \
    ((uint32_t)BOARD_AUDIO_SAMPLE_RATE_HZ * SDR_CW_EDGE_RAMP_MS / 1000u) /* 144 */

/* Paddle debounce: a level change is committed only after the raw line has
 * been stable (no further edge) for this long. The wait is timed in CPU cycles
 * so it is independent of the sampling rate. */
#define SDR_CW_KEY_DEBOUNCE_MS 2u

_Static_assert(SDR_CW_EVENT_DELAY_FRAMES <= DRV_CODEC_BLOCK_FRAMES - 1u,
               "artificial delay must fit in one block");

#define SDR_CW_ENV_INF  UINT32_MAX

/* dot = 1200/WPM ms = 57600/WPM frames at 48 kHz (20 WPM -> 2880). */
#define SDR_CW_DOT_FRAMES_1WPM \
    ((uint32_t)BOARD_AUDIO_SAMPLE_RATE_HZ * 12u / 10u)
/* Defaults until the host configures them (cmd34 qsktime_kr). */
#define SDR_CW_QSK_TIME_MS_DEFAULT 100u
#define SDR_CW_DAH_MULT_DEFAULT    3.0f
/* Keep both ramps inside one element. */
#define SDR_CW_DOT_FRAMES_MIN (2u * SDR_CW_EDGE_RAMP_LEN)


/* ------------------------------------------------------------------------- */
/* Types                                                                     */
/* ------------------------------------------------------------------------- */

typedef enum {
    SDR_CW_KEY_TIP  = 0, /* jack tip is PH4 */
    SDR_CW_KEY_RING = 1, /* jack ring is PH3 */
    SDR_CW_KEY_COUNT
} SdrCwKeyLine;

typedef enum {
    SDR_CW_PADDLE_DIT = 0,
    SDR_CW_PADDLE_DAH
} SdrCwPaddle;

typedef enum {
    SDR_CW_ELEM_NONE = 0,
    SDR_CW_ELEM_DIT,
    SDR_CW_ELEM_DAH
} SdrCwElement;

typedef struct {
    bool     raw;         /* last raw level seen by an edge, true = key down */
    bool     level;       /* debounced committed 0/1, true = key down */
    bool     pending;     /* raw differs from level, waiting for the debounce */
    uint32_t lag;         /* 0..BLOCK-1: lag of the last edge, for placement */
    bool     fresh;       /* committed level changed, not consumed by keyer yet */
    uint32_t edge_time;   /* CPU cycles at the last edge */
} SdrCwKeyState;

/* Host key is polled from the key tick instead of an EXTI and needs no
 * debounce: the level is committed on the poll that sees it. */
typedef struct {
    bool     level;       /* polled level, true = key down */
    bool     fresh;       /* level changed, not consumed by keyer yet */
    uint32_t lag;         /* 0..BLOCK-1: edge position, for placement */
} SdrCwHostKeyState;


/* ------------------------------------------------------------------------- */
/* Module state                                                              */
/* ------------------------------------------------------------------------- */

/* Paddle lines: index matches SdrCwKeyLine, both active low (pull-up). */
static const BspPin s_key_pins[SDR_CW_KEY_COUNT] = {
    [SDR_CW_KEY_TIP] = { CW_PADDLE_TIP_GPIO_Port, CW_PADDLE_TIP_Pin },
    [SDR_CW_KEY_RING] = { CW_PADDLE_RING_GPIO_Port, CW_PADDLE_RING_Pin },
};

static SdrCwHostKeyState s_host_key;

static struct {
    SdrCwKeyState line[SDR_CW_KEY_COUNT];
    /* --- iambic state machine --- */
    bool          element_in_progress;
    bool          priming;        /* pre-roll of the first element to the boundary */
    SdrCwElement  element;        /* current/last element, for alternation */
    bool          mem_dit;        /* mode B: dit seen during another element */
    bool          mem_dah;        /* mode B: dah seen during another element */
    bool          training;       /* training mode: key does not start TX */
    bool          host_owned;     /* host key owns the envelope, paddles ignored */
    /* --- TX request --- */
    bool          tx_req;         /* last value pushed to App_Ptt_SetCwKey */
    uint32_t      tx_tail;        /* qsk hold-off frames after the last element */
} s_keyer;

static struct {
    bool     state;     /* current phase */
    uint32_t counter;   /* 0..UINT32_MAX-1, saturation */
    uint32_t duration;  /* INF is UINT32_MAX */
} s_env;

static struct {
    int32_t  band_low;   /* host passband low edge, relative to DC */
    int32_t  band_high;  /* host passband high edge, relative to DC */
    uint32_t audio_low;  /* keytone-mapped audio band, cw_compute_audio_band */
    uint32_t audio_high;
    uint32_t key_tone_hz;
    SdrModulation rx_mode;
    SdrModulation tx_mode;
    AppRadioMode radio_mode;
    float key_vol;
    SdrCwKeyMode key_mode;
    SdrCwIambicMode iambic_mode;
    uint32_t speed_wpm;
    uint32_t qsk_time_ms;
    float    dash_mult;
} s_state;

static struct {
    bool lpf_switch;
    bool hpf_switch;
    bool iir_reset;
    bool fir_reset;
} s_pending;

static float s_ramp[SDR_CW_EDGE_RAMP_LEN];

/* One block of envelope values, recomputed once per codec block in the keyer
 * update so the sidetone can reuse it without entering Sdr_CW_Mod. */
static float s_env_block[DRV_CODEC_BLOCK_FRAMES];

static float s_sidetone[DRV_CODEC_BLOCK_FRAMES];

/* Paddle debounce: wait in CPU cycles (set in Sdr_CW_Init) and whether the
 * 3 kHz key tick is driving it (else the per-block update is the fallback). */
static uint32_t      s_key_debounce_cycles;
static volatile bool s_key_tick_active;

/* RX BFO: one shift that moves the tuned carrier from DC to the sidetone pitch
 * (CWR shifts down, i.e. the lower sideband). The NCO keeps its phase across
 * retunes. */
static Sdr_Nco s_nco_rx;
static Sdr_Nco s_nco_sidetone;

/* Audio band-pass that follows demodulation: the user passband mapped around
 * the sidetone, as a real LPF + HPF on the demodulated audio. Each filter has
 * its own coefficient set, toggled at most once per block in apply_pending;
 * update_lpf_cw/update_hpf_cw fill the idle (shadow) set. */
static arm_biquad_casd_df1_inst_f32 s_lpf;
static arm_biquad_casd_df1_inst_f32 s_hpf;
static volatile uint32_t s_lpf_id;
static volatile uint32_t s_hpf_id;
static uint32_t s_lpf_edge[2]; /* [set] = high edge designed into this set */
static uint32_t s_hpf_edge[2]; /* [set] = low edge designed into this set */
static float s_lpf_coeffs[2][SDR_CW_FILTER_NUM_STAGES * 5];
static float s_hpf_coeffs[2][SDR_CW_FILTER_NUM_STAGES * 5];
static float s_lpf_state[SDR_CW_FILTER_NUM_STAGES * 4];
static float s_hpf_state[SDR_CW_FILTER_NUM_STAGES * 4];

/* Demod scratch: the input I/Q shifted to the sidetone, fed to the shared
 * sideband FIR. */
static float s_rx_i[SDR_CW_RX_BLOCK_SIZE] __attribute__((aligned(32)));
static float s_rx_q[SDR_CW_RX_BLOCK_SIZE] __attribute__((aligned(32)));

/* Squelch metric: tone peak after the FIR vs the audio-domain noise. Updated
 * once per demod block, read by the squelch in the same ISR pass. */
static float s_signal_metric;

/* S-meter: per-block in-band power (post-filter audio), EMA-smoothed and
 * peak-held between reads. The last dB value is held while no block runs (TX). */
static float s_level_alpha;
static float s_level_ema;
static float s_level_peak;
static float s_level_db_last = -127.0f;


/* ------------------------------------------------------------------------- */
/* Timing and mode helpers                                                   */
/* ------------------------------------------------------------------------- */

static uint32_t cw_dot_frames(void)
{
    uint32_t wpm = s_state.speed_wpm;
    uint32_t dot;

    if (wpm == 0u) {
        wpm = 20u;
    }
    dot = SDR_CW_DOT_FRAMES_1WPM / wpm;
    if (dot < SDR_CW_DOT_FRAMES_MIN) {
        dot = SDR_CW_DOT_FRAMES_MIN;
    }
    return dot;
}

static uint32_t cw_qsk_frames(void)
{
    return (uint32_t)BOARD_AUDIO_SAMPLE_RATE_HZ * s_state.qsk_time_ms / 1000u;
}

static uint32_t cw_element_frames(SdrCwElement e)
{
    uint32_t dot = cw_dot_frames();

    if (e != SDR_CW_ELEM_DAH) {
        return dot;
    }
    return (uint32_t)((float)dot * s_state.dash_mult + 0.5f);
}

static bool cw_mode_selected(void)
{
    return s_state.rx_mode == SDR_MOD_CW || s_state.rx_mode == SDR_MOD_CWR ||
           s_state.tx_mode == SDR_MOD_CW || s_state.tx_mode == SDR_MOD_CWR;
}


/* ------------------------------------------------------------------------- */
/* Keyer: element selection                                                  */
/* ------------------------------------------------------------------------- */

/* Map a logical paddle to the physical line for the active key mode. */
static SdrCwKeyLine cw_paddle_line(SdrCwPaddle p)
{
    switch (s_state.key_mode) {
    case SDR_CW_KEY_DUAL_RIGHT:
        return (p == SDR_CW_PADDLE_DIT) ? SDR_CW_KEY_RING : SDR_CW_KEY_TIP;
    case SDR_CW_KEY_DUAL_LEFT:
    default:
        return (p == SDR_CW_PADDLE_DIT) ? SDR_CW_KEY_TIP : SDR_CW_KEY_RING;
    }
}

static bool cw_paddle_down(SdrCwPaddle p)
{
    return s_keyer.line[cw_paddle_line(p)].level;
}

/* Squeeze policy: pick the next element from the effective paddle levels.
 * `last` is the element just emitted; with both paddles present the iambic
 * alternation keeps the dit/dah sequence. Pure: reads no module state, which
 * keeps the policy testable in isolation. */
static SdrCwElement cw_pick_element(SdrCwElement last, bool dit, bool dah)
{
    if (dit && dah) {
        return (last == SDR_CW_ELEM_DIT) ? SDR_CW_ELEM_DAH : SDR_CW_ELEM_DIT;
    }
    if (dit) {
        return SDR_CW_ELEM_DIT;
    }
    if (dah) {
        return SDR_CW_ELEM_DAH;
    }
    return SDR_CW_ELEM_NONE;
}

/* Next element from the held paddles plus (mode B) latched presses. The latches
 * are consumed here, then the shared squeeze policy decides. */
static SdrCwElement cw_keyer_pick(void)
{
    bool dit = cw_paddle_down(SDR_CW_PADDLE_DIT) || s_keyer.mem_dit;
    bool dah = cw_paddle_down(SDR_CW_PADDLE_DAH) || s_keyer.mem_dah;

    s_keyer.mem_dit = false;
    s_keyer.mem_dah = false;

    return cw_pick_element(s_keyer.element, dit, dah);
}


/* ------------------------------------------------------------------------- */
/* Keyer: envelope                                                           */
/* ------------------------------------------------------------------------- */

/* Raised-cosine output for one sample: `on` is the target level, `counter`
 * samples since the segment started. */
static float cw_env_shape(bool on, uint32_t counter)
{
    if (on) {
        return (counter < SDR_CW_EDGE_RAMP_LEN) ? s_ramp[counter] : 1.0f;
    }
    return (counter < SDR_CW_EDGE_RAMP_LEN)
        ? s_ramp[SDR_CW_EDGE_RAMP_LEN - 1u - counter] /* fall */
        : 0.0f;
}

/* End of the current envelope segment: advance the keyer. Straight just
 * toggles (the physical edge re-seeds); dual runs the iambic machine. */
static void cw_env_segment_end(void)
{
    /* The host key is always straight, regardless of the paddle mode. */
    if (s_state.key_mode == SDR_CW_KEY_STRAIGHT || s_keyer.host_owned) {
        s_env.state    = !s_env.state;
        s_env.counter  = 0;
        s_env.duration = SDR_CW_ENV_INF;
        return;
    }

    if (s_keyer.priming) {
        s_keyer.priming = false;
        s_keyer.element = cw_keyer_pick();
        if (s_keyer.element == SDR_CW_ELEM_NONE) {
            s_env.state    = false;
            s_env.counter  = SDR_CW_EDGE_RAMP_LEN;
            s_env.duration = SDR_CW_ENV_INF;
            s_keyer.element_in_progress = false;
            return;
        }
        s_env.state    = true;
        s_env.counter  = 0;
        s_env.duration = cw_element_frames(s_keyer.element);
        return;
    }

    if (s_env.state) {
        /* mark ended: always one dit of space before the next element */
        s_env.state    = false;
        s_env.counter  = 0;
        s_env.duration = cw_dot_frames();
        return;
    }

    /* space ended: pick the next element or stop */
    s_keyer.element = cw_keyer_pick();
    if (s_keyer.element == SDR_CW_ELEM_NONE) {
        s_env.state    = false;
        s_env.counter  = SDR_CW_EDGE_RAMP_LEN;
        s_env.duration = SDR_CW_ENV_INF;
        s_keyer.element_in_progress = false;
        return;
    }
    s_env.state    = true;
    s_env.counter  = 0;
    s_env.duration = cw_element_frames(s_keyer.element);
}

/* Fill one block of the raised-cosine envelope. `state` is the phase
 * (false = falling/idle, true = rising/hold) and `counter` counts samples in
 * it: below SDR_CW_EDGE_RAMP_LEN it indexes the ramp, above it the output is
 * flat. A finite `duration` ends the segment when the counter reaches it; the
 * transition is applied before the sample so the transition sample is the first
 * sample of the new segment. */
static void cw_env_fill_block(void)
{
    for (uint32_t k = 0; k < DRV_CODEC_BLOCK_FRAMES; ++k) {
        if (s_env.duration != SDR_CW_ENV_INF
            && s_env.counter >= s_env.duration) {
            cw_env_segment_end();
        }

        s_env_block[k] = cw_env_shape(s_env.state, s_env.counter);

        if (s_env.counter != UINT32_MAX - 1u) {
            s_env.counter++;
        }
    }
}

static void cw_reset_envelope(void)
{
    s_env.state     = false;
    s_env.counter   = SDR_CW_EDGE_RAMP_LEN;
    s_env.duration  = SDR_CW_ENV_INF;
}

static void cw_reset_keyer(void)
{
    s_keyer.element_in_progress = false;
    s_keyer.priming             = false;
    s_keyer.element             = SDR_CW_ELEM_NONE;
    s_keyer.mem_dit             = false;
    s_keyer.mem_dah             = false;
    s_keyer.host_owned          = false;
}

/* Seed a straight-key edge: `down` picks the target phase, `lag` places the
 * edge inside the current codec block and `guard` delays a press until the TX
 * path has settled. A finite duration carries the phase to its opposite at the
 * segment end, where cw_env_segment_end stops the keyer. */
static void cw_straight_seed(bool down, uint32_t lag, uint32_t guard)
{
    s_env.state    = !down;
    s_env.counter  = SDR_CW_EDGE_RAMP_LEN;
    s_env.duration = s_env.counter + SDR_CW_EVENT_DELAY_FRAMES - lag + guard;
}


/* ------------------------------------------------------------------------- */
/* Keyer: TX request                                                         */
/* ------------------------------------------------------------------------- */

/* Push the TX request to the PTT arbiter, only on change. */
static void cw_tx_request_apply(bool on)
{
    if (on != s_keyer.tx_req) {
        s_keyer.tx_req = on;
        App_Ptt_SetCwKey(on);
    }
}


/* ------------------------------------------------------------------------- */
/* Keyer: paddle input and debounce                                          */
/* ------------------------------------------------------------------------- */

/* Paddle EXTI: short, no logs, no blocking calls. Records the raw level and the
 * edge position on the 48k timeline (from the TX DMA counter); the debounce
 * step below commits the level. */
static void cw_key_exti(void *ctx)
{
    SdrCwKeyLine line = (SdrCwKeyLine)(uintptr_t)ctx;
    bool         down = Bsp_Gpio_Read(s_key_pins[line]) == GPIO_PIN_RESET;
    uint32_t     key;

    key = Bsp_Irq_SaveLow();
    s_keyer.line[line].raw       = down;
    s_keyer.line[line].lag       = Drv_Codec_TxDmaRemaining()
                                   & (DRV_CODEC_BLOCK_FRAMES - 1u);
    s_keyer.line[line].edge_time = Bsp_Time_GetCycles();
    s_keyer.line[line].pending   = (down != s_keyer.line[line].level);
    Bsp_Irq_RestoreLow(key);
}

/* Commit a raw change once it has been stable for SDR_CW_KEY_DEBOUNCE_MS.
 * Driven by the 3 kHz TIM7 key tick; if that timer failed to start,
 * Sdr_CW_KeyerUpdate runs it once per codec block instead. */
static void cw_key_debounce_step(void)
{
    uint32_t key = Bsp_Irq_SaveLow();
    uint32_t now = Bsp_Time_GetCycles();
    bool     down = false;

    for (unsigned i = 0; i < SDR_CW_KEY_COUNT; ++i) {
        if (s_keyer.line[i].pending
            && (now - s_keyer.line[i].edge_time) >= s_key_debounce_cycles) {
            s_keyer.line[i].level   = s_keyer.line[i].raw;
            s_keyer.line[i].pending = false;
            s_keyer.line[i].fresh   = true;
            if (s_keyer.line[i].raw) {
                down = true;
            }
        }
    }
    Bsp_Irq_RestoreLow(key);

    /* Fast TX start: request TX at the debounce commit, so the mode transition
     * fits inside the debounce window and generation starts on the same block
     * as it would in training (no extra block). */
    if (down && cw_mode_selected() && !s_keyer.training) {
        cw_tx_request_apply(true);
    }
}

/* Poll the host-driven straight key. No debounce: the host signal is not a
 * mechanical contact, so the polled level is committed as is. */
static void cw_host_key_poll(void)
{
    bool     down = Bsp_Board_IsCwHostKeyDown();
    bool     was_down;
    uint32_t key = Bsp_Irq_SaveLow();

    was_down = s_host_key.level;
    if (down != was_down) {
        s_host_key.level = down;
        s_host_key.fresh = true;
        s_host_key.lag   = Drv_Codec_TxDmaRemaining()
                           & (DRV_CODEC_BLOCK_FRAMES - 1u);
    }
    Bsp_Irq_RestoreLow(key);

    /* Fast TX start, same as the paddle debounce commit: the settle window is
     * entered before the keyer seeds the first element. */
    if (down && !was_down && cw_mode_selected() && !s_keyer.training) {
        cw_tx_request_apply(true);
    }
}

/* After the host key yields control, treat paddles that are still held as a
 * fresh press so the physical keyer resumes without a re-press. */
static void cw_paddles_rearm(void)
{
    uint32_t key = Bsp_Irq_SaveLow();

    for (unsigned i = 0; i < SDR_CW_KEY_COUNT; ++i) {
        if (s_keyer.line[i].level) {
            s_keyer.line[i].fresh = true;
            s_keyer.line[i].lag   = 0u;
        }
    }
    Bsp_Irq_RestoreLow(key);
}

void Sdr_CW_KeyTick(void)
{
    s_key_tick_active = true;
    cw_host_key_poll();
    cw_key_debounce_step();
}

static void cw_key_tick_cb(void *ctx)
{
    (void)ctx;
    Sdr_CW_KeyTick();
}


/* ------------------------------------------------------------------------- */
/* DSP: NCO and audio filter                                                 */
/* ------------------------------------------------------------------------- */

/* Recompute the RX BFO and sidetone steps. The trig runs outside the IRQ
 * mask; only the publication of the new steps is atomic. `with_sidetone`
 * refreshes the sidetone NCO as well, which only a tone change affects. */
static void cw_update_nco_steps(bool with_sidetone)
{
    float    tone = (float)s_state.key_tone_hz;
    float    side_re, side_im;
    uint32_t key;

    if (s_state.rx_mode == SDR_MOD_CWR) {
        tone = -tone;
    }
    float rx_rad = 2.0f * (float)M_PI * tone / (float)SDR_CW_RX_RATE;
    float rx_re = cosf(rx_rad);
    float rx_im = sinf(rx_rad);
    if (with_sidetone) {
        float sidetone_rad = 2.0f * (float)M_PI * tone / (float)BOARD_AUDIO_SAMPLE_RATE_HZ;
        side_re = cosf(sidetone_rad);
        side_im = sinf(sidetone_rad);
    }

    key = Bsp_Irq_SaveLow();
    Sdr_Nco_SetStep(&s_nco_rx, rx_re, rx_im);
    if (with_sidetone) {
        Sdr_Nco_SetStep(&s_nco_sidetone, side_re, side_im);
    }
    Bsp_Irq_RestoreLow(key);
}

/* Map the user passband P = [band_low, band_high], relative to the carrier,
 * into the audio band around the sidetone: CW keeps P, CWR mirrors it. */
static void cw_compute_audio_band(void)
{
    int32_t k = (int32_t)s_state.key_tone_hz;
    int32_t low;
    int32_t high;

    if (s_state.rx_mode == SDR_MOD_CWR) {
        low  = k - s_state.band_high;
        high = k - s_state.band_low;
    } else {
        low  = k + s_state.band_low;
        high = k + s_state.band_high;
    }

    int32_t max = (int32_t)(SDR_CW_RX_RATE / 2u) - 100;
    if (low < 1) {
        low = 1;
    }
    if (high > max) {
        high = max;
    }
    if (high <= low) {
        high = low + 100;
    }
    s_state.audio_low  = (uint32_t)low;
    s_state.audio_high = (uint32_t)high;
}

/* Design the LPF (high edge) into the idle set when it does not already hold
 * the requested edge; a set that already matches is reused. The active set is
 * checked first so an up-to-date filter is never re-switched, and the pending
 * switch is cancelled while re-validating so the active set cannot be lost. */
static void update_lpf_cw(void) {
    uint32_t key = Bsp_Irq_SaveLow();
    s_pending.lpf_switch = false;
    if (s_lpf_edge[s_lpf_id] == s_state.audio_high) {
        Bsp_Irq_RestoreLow(key);
        return;
    }
    Bsp_Irq_RestoreLow(key);

    uint32_t shadow_id = s_lpf_id ^ 1u;

    if (s_lpf_edge[shadow_id] != s_state.audio_high) {
        SdrIirSpec spec = {
            .edge = SDR_IIR_EDGE_NOISE_BW,
            .f_edge_hz = (float)s_state.audio_high,
            .fs_hz = (float)SDR_CW_RX_RATE,
            .stages = SDR_CW_FILTER_NUM_STAGES,
            .type = SDR_IIR_TYPE_LPF
        };
        uint8_t stages;
        SdrIirResponse resp;
        int res = Sdr_Iir_DesignEllipEx(&spec, s_lpf_coeffs[shadow_id],
                                        &stages, &resp);
        if (res != 0) {
            LOG_WARN("iir lpf design failed (%i)", res);
            return;
        }
        LOG_DEBUG("iir lpf ref %.1f stop %.1f nbw %.1f gain %.2f level %.1f",
                    resp.f_ref_hz, resp.f_stop_hz, resp.noise_bw_hz,
                    resp.enbw_gain_db, resp.level_hz);
        s_lpf_edge[shadow_id] = s_state.audio_high;
    }

    key = Bsp_Irq_SaveLow();
    s_pending.lpf_switch = true;
    Bsp_Irq_RestoreLow(key);
}

/* HPF (low edge) counterpart of update_lpf_cw. */
static void update_hpf_cw(void) {
    uint32_t key = Bsp_Irq_SaveLow();
    s_pending.hpf_switch = false;
    if (s_hpf_edge[s_hpf_id] == s_state.audio_low) {
        Bsp_Irq_RestoreLow(key);
        return;
    }
    Bsp_Irq_RestoreLow(key);

    uint32_t shadow_id = s_hpf_id ^ 1u;

    if (s_hpf_edge[shadow_id] != s_state.audio_low) {
        SdrIirSpec spec = {
            .edge = SDR_IIR_EDGE_NOISE_BW,
            .f_edge_hz = (float)s_state.audio_low,
            .fs_hz = (float)SDR_CW_RX_RATE,
            .stages = SDR_CW_FILTER_NUM_STAGES,
            .type = SDR_IIR_TYPE_HPF
        };
        uint8_t stages;
        SdrIirResponse resp;
        int res = Sdr_Iir_DesignEllipEx(&spec, s_hpf_coeffs[shadow_id],
                                        &stages, &resp);
        if (res != 0) {
            LOG_WARN("iir hpf design failed (%i)", res);
            return;
        }
        LOG_DEBUG("iir hpf ref %.1f stop %.1f nbw %.1f gain %.2f level %.1f",
                    resp.f_ref_hz, resp.f_stop_hz, resp.noise_bw_hz,
                    resp.enbw_gain_db, resp.level_hz);
        s_hpf_edge[shadow_id] = s_state.audio_low;
    }

    key = Bsp_Irq_SaveLow();
    s_pending.hpf_switch = true;
    Bsp_Irq_RestoreLow(key);
}

/* Re-validate both audio filters against the current band. Each edge changes
 * on its own, so only the moved edge pays for a redesign. */
static void update_audio_filter(void) {
    update_lpf_cw();
    update_hpf_cw();
}

static void gen_sidetone(void) {
    for (size_t i = 0; i < DRV_CODEC_BLOCK_FRAMES; i++) {
        s_sidetone[i] = s_nco_sidetone.real * s_env_block[i] * s_state.key_vol;
        Sdr_Nco_Step(&s_nco_sidetone);
    }
}


/* ------------------------------------------------------------------------- */
/* Events                                                                    */
/* ------------------------------------------------------------------------- */

static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;

    bool rx_changed = cfg->rx_mode != s_state.rx_mode;
    bool cw_old = s_state.rx_mode == SDR_MOD_CW || s_state.rx_mode == SDR_MOD_CWR;
    bool cw_new = cfg->rx_mode == SDR_MOD_CW || cfg->rx_mode == SDR_MOD_CWR;

    /* The host passband is the signed baseband band around the carrier. Keep it
     * inside (-K, K): outside, the audio band would fold. */
    int32_t k = (int32_t)s_state.key_tone_hz;
    int32_t low = cfg->filter_low_hz;
    int32_t high = cfg->filter_high_hz;
    if (low > high) {
        int32_t tmp = low;
        low = high;
        high = tmp;
    }
    if (low <= -k) {
        low = -k + 1;
    }
    if (high >= k) {
        high = k - 1;
    }
    bool band_changed = (low != s_state.band_low) || (high != s_state.band_high);
    s_state.band_low  = low;
    s_state.band_high = high;

    if (rx_changed) {
        uint32_t key = Bsp_Irq_SaveLow();
        s_state.rx_mode = cfg->rx_mode;
        if (cw_old || cw_new) {
            s_pending.iir_reset = true;
            s_pending.fir_reset = true;
        }
        cw_reset_envelope();
        cw_reset_keyer();
        Bsp_Irq_RestoreLow(key);
    }

    if (cw_new && (rx_changed || band_changed)) {
        cw_compute_audio_band();
        update_audio_filter();
    }
    if (cw_new && rx_changed) {
        cw_update_nco_steps(false);
    }
}

static void on_tx_cfg(const SdrTxCfg *cfg, void *user) {
    (void)user;
    if (cfg->tx_mode == s_state.tx_mode) {
        return;
    }
    uint32_t key = Bsp_Irq_SaveLow();
    s_state.tx_mode = cfg->tx_mode;
    cw_reset_envelope();
    cw_reset_keyer();
    Bsp_Irq_RestoreLow(key);
}

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if ((s_state.rx_mode == SDR_MOD_CW || s_state.rx_mode == SDR_MOD_CWR) &&
        evt->mode != s_state.radio_mode) {
        /* Clear the audio IIR and the shared sideband FIR across RX/TX. */
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.iir_reset = true;
        s_pending.fir_reset = true;
        Bsp_Irq_RestoreLow(key);
    }
    /* The keyer is not reset on RX/TX: the envelope runs independently of the
     * radio mode, and a reset here would cut the element that just keyed TX. */
    s_state.radio_mode = evt->mode;
}

static void on_key_tone_change(const SdrKeyToneCfg *cfg, void *user) {
    (void)user;
    if (cfg->key_tone_hz == s_state.key_tone_hz) {
        return;
    }
    s_state.key_tone_hz = cfg->key_tone_hz;

    /* Re-clamp the baseband passband for the new tone and remap the audio
     * band; the sidetone NCO always follows the tone. */
    int32_t k = (int32_t)s_state.key_tone_hz;
    if (s_state.band_low <= -k) {
        s_state.band_low = -k + 1;
    }
    if (s_state.band_high >= k) {
        s_state.band_high = k - 1;
    }

    cw_update_nco_steps(true);
    if (s_state.rx_mode == SDR_MOD_CW || s_state.rx_mode == SDR_MOD_CWR) {
        cw_compute_audio_band();
        update_audio_filter();
    }
}


/* ------------------------------------------------------------------------- */
/* Host configuration                                                        */
/* ------------------------------------------------------------------------- */

void Sdr_CW_SetKeySpeed(uint32_t wpm)
{
    if (wpm < SDR_CW_SPEED_WPM_MIN) {
        wpm = SDR_CW_SPEED_WPM_MIN;
    } else if (wpm > SDR_CW_SPEED_WPM_MAX) {
        wpm = SDR_CW_SPEED_WPM_MAX;
    }

    uint32_t key = Bsp_Irq_SaveLow();
    s_state.speed_wpm = wpm;
    Bsp_Irq_RestoreLow(key);
}

void Sdr_CW_SetKeyMode(SdrCwKeyMode mode)
{
    if (mode != SDR_CW_KEY_STRAIGHT && mode != SDR_CW_KEY_DUAL_LEFT
        && mode != SDR_CW_KEY_DUAL_RIGHT) {
        return;
    }

    uint32_t key = Bsp_Irq_SaveLow();
    if (mode != s_state.key_mode) {
        s_state.key_mode = mode;
        cw_reset_envelope();
        cw_reset_keyer();
    }
    Bsp_Irq_RestoreLow(key);
}

void Sdr_CW_SetIambicMode(SdrCwIambicMode mode)
{
    if (mode != SDR_CW_IAMBIC_A && mode != SDR_CW_IAMBIC_B) {
        return;
    }

    uint32_t key = Bsp_Irq_SaveLow();
    s_state.iambic_mode = mode;
    Bsp_Irq_RestoreLow(key);
}

void Sdr_CW_SetSidetoneVolume(uint8_t cwvol)
{
    float vol = (cwvol == 0u)
        ? 0.0f
        : Sdr_Db2Lin(((float)cwvol - 57.0f) * 0.5f);

    uint32_t key = Bsp_Irq_SaveLow();
    s_state.key_vol = vol;
    Bsp_Irq_RestoreLow(key);
}

void Sdr_CW_SetTraining(bool training)
{
    uint32_t key = Bsp_Irq_SaveLow();
    s_keyer.training = training;
    Bsp_Irq_RestoreLow(key);
}

void Sdr_CW_SetQskTimeAndRatio(uint32_t qsk_ms, uint32_t dash_ratio_x10)
{
    if (qsk_ms > 1000u) {
        qsk_ms = 1000u;
    }
    if (dash_ratio_x10 < 25u) {
        dash_ratio_x10 = 25u;
    } else if (dash_ratio_x10 > 45u) {
        dash_ratio_x10 = 45u;
    }

    uint32_t key = Bsp_Irq_SaveLow();
    s_state.qsk_time_ms = qsk_ms;
    s_state.dash_mult   = (float)dash_ratio_x10 * 0.1f;
    Bsp_Irq_RestoreLow(key);
}


/* ------------------------------------------------------------------------- */
/* Init                                                                      */
/* ------------------------------------------------------------------------- */

void Sdr_CW_Init(void)
{
    cw_reset_envelope();
    cw_reset_keyer();

    s_key_tick_active     = false;
    s_key_debounce_cycles = (uint32_t)((uint64_t)Bsp_Time_GetCpuHz()
                                       * SDR_CW_KEY_DEBOUNCE_MS / 1000u);
    for (unsigned i = 0; i < SDR_CW_KEY_COUNT; ++i) {
        bool down = Bsp_Gpio_Read(s_key_pins[i]) == GPIO_PIN_RESET;

        s_keyer.line[i].raw       = down;
        s_keyer.line[i].level     = down;
        s_keyer.line[i].pending   = false;
        s_keyer.line[i].fresh     = false;
        s_keyer.line[i].lag       = 0u;
        s_keyer.line[i].edge_time = 0u;
    }

    s_host_key.level = Bsp_Board_IsCwHostKeyDown();
    s_host_key.fresh = false;
    s_host_key.lag   = 0u;

    for (uint32_t n = 0; n < SDR_CW_EDGE_RAMP_LEN; ++n) {
        s_ramp[n] = 0.5f * (1.0f - cosf((float)M_PI * (float)n /
                                        (float)(SDR_CW_EDGE_RAMP_LEN - 1u)));
    }

    if (Bsp_Exti_Register(CW_PADDLE_RING_Pin, cw_key_exti,
                          (void *)(uintptr_t)SDR_CW_KEY_RING) != 0) {
        LOG_WARN("ring exti register failed");
    }
    if (Bsp_Exti_Register(CW_PADDLE_TIP_Pin, cw_key_exti,
                          (void *)(uintptr_t)SDR_CW_KEY_TIP) != 0) {
        LOG_WARN("tip exti register failed");
    }

    arm_biquad_cascade_df1_init_f32(&s_lpf, SDR_CW_FILTER_NUM_STAGES,
        s_lpf_coeffs[s_lpf_id], s_lpf_state);
    arm_biquad_cascade_df1_init_f32(&s_hpf, SDR_CW_FILTER_NUM_STAGES,
        s_hpf_coeffs[s_hpf_id], s_hpf_state);

    s_state.key_tone_hz = 600;
    s_state.band_low = -250;
    s_state.band_high = 250;
    s_state.audio_low = 0;
    s_state.audio_high = 0;
    s_state.key_vol = 1.0f;
    s_state.key_mode = SDR_CW_KEY_DUAL_RIGHT;
    s_state.iambic_mode = SDR_CW_IAMBIC_B;
    s_state.speed_wpm = 20;
    s_state.qsk_time_ms = SDR_CW_QSK_TIME_MS_DEFAULT;
    s_state.dash_mult = SDR_CW_DAH_MULT_DEFAULT;

    s_pending.lpf_switch = false;
    s_pending.hpf_switch = false;
    s_pending.iir_reset = false;
    s_pending.fir_reset = false;

    Sdr_Nco_Init(&s_nco_rx);
    Sdr_Nco_Init(&s_nco_sidetone);

    Sdr_Sideband_Reset();
    cw_compute_audio_band();
    cw_update_nco_steps(true);
    update_audio_filter();

    s_level_alpha = 1.0f - expf(-1.0f / ((float)SDR_CW_LEVEL_EMA_MS *
                              SDR_CW_LEVEL_BLOCK_RATE / 1000.0f));
    s_level_ema      = 0.0f;
    s_level_peak     = 0.0f;
    s_level_db_last  = -127.0f;

    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
    if (App_Event_SubscribeTxCfg(on_tx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeTxCfg failed");
    }
    if (App_Event_SubscribeKeyTone(on_key_tone_change, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeKeyTone failed");
    }
    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }

    if (Bsp_Time_StartTimer(BSP_TIME_TIMER_CW_KEY, cw_key_tick_cb, 0) != 0) {
        LOG_WARN("tim7 start failed, debounce at block rate");
    }
}


/* ------------------------------------------------------------------------- */
/* Keyer block tick                                                          */
/* ------------------------------------------------------------------------- */

float * Sdr_CW_KeyerUpdate(void)
{
    SdrCwKeyState line[SDR_CW_KEY_COUNT];
    bool          host_held;
    bool          host_fresh;
    uint32_t      host_lag;
    bool          any_down;
    bool          host_mode;
    uint32_t      key;

    if (!s_key_tick_active) {
        /* TIM7 key tick did not start: poll and debounce at block rate. */
        cw_host_key_poll();
        cw_key_debounce_step();
    }

    key = Bsp_Irq_SaveLow();
    for (unsigned i = 0; i < SDR_CW_KEY_COUNT; ++i) {
        line[i]               = s_keyer.line[i];
        s_keyer.line[i].lag   = 0u;
        s_keyer.line[i].fresh = false;
    }
    host_held        = s_host_key.level;
    host_fresh       = s_host_key.fresh;
    host_lag         = s_host_key.lag;
    s_host_key.fresh = false;
    Bsp_Irq_RestoreLow(key);

    if (s_state.key_mode == SDR_CW_KEY_STRAIGHT) {
        any_down = line[SDR_CW_KEY_TIP].level;
    } else {
        any_down = line[SDR_CW_KEY_TIP].level || line[SDR_CW_KEY_RING].level;
    }

    /* The host key owns the whole keying path, including its qsk tail; while
     * it is active the physical paddles are ignored. A fresh release keeps
     * ownership for one block so the falling edge is rendered even with no
     * qsk tail. */
    bool host_active = host_held
                       || (s_keyer.host_owned
                           && (host_fresh || s_keyer.tx_tail > 0u));

    host_mode = s_keyer.host_owned;

    /* Advance the envelope once per codec block. Driven by the key source,
     * independent of RX/TX: training and the TX monitor share the same
     * coefficients while Sdr_CW_Mod is skipped outside TX. */
    if (cw_mode_selected()) {
        /* Delay the first element/priming when the TX path has to settle, so
         * the first element is not cut by the relay switching. */
        uint32_t guard = s_keyer.training ? 0u
                                          : App_Radio_TxSettleRemainingFrames();

        if (!s_keyer.host_owned && host_active) {
            /* Take over from the paddle machine; the host key is straight. */
            cw_reset_keyer();
            s_keyer.host_owned = true;
            host_mode          = true;
            if (host_held) {
                cw_straight_seed(true, host_lag, guard);
            }
        } else if (s_keyer.host_owned && !host_active) {
            /* Host qsk tail ended: release the envelope and let paddles back. */
            cw_reset_envelope();
            cw_reset_keyer();
            cw_paddles_rearm();
            host_mode = false;
        } else if (s_keyer.host_owned && host_fresh) {
            /* Guard only the press edge: the settle delay must shift the start
             * of the first element, not the release. */
            cw_straight_seed(host_held, host_lag, host_held ? guard : 0u);
        }

        if (host_mode) {
            /* Straight key: only the host line shapes the envelope. */
            cw_env_fill_block();
            gen_sidetone();
        } else if (s_state.key_mode == SDR_CW_KEY_STRAIGHT) {
            /* An accepted edge seeds the envelope with the matching phase and
             * a finite duration. The counter reaches it after the DMA-derived
             * delay, then the segment end flips to the opposite phase. */
            if (line[SDR_CW_KEY_TIP].fresh) {
                uint32_t g = line[SDR_CW_KEY_TIP].level ? guard : 0u;

                cw_straight_seed(line[SDR_CW_KEY_TIP].level,
                                 line[SDR_CW_KEY_TIP].lag, g);
            }
            cw_env_fill_block();
            gen_sidetone();
        } else {
            /* Start the iambic machine on an accepted press edge; that edge's
             * lag sets the sub-block position of the first element. */
            if (!s_keyer.element_in_progress) {
                for (unsigned i = 0; i < SDR_CW_KEY_COUNT; ++i) {
                    if (line[i].fresh && line[i].level) {
                        s_keyer.element_in_progress = true;
                        s_keyer.priming             = true;
                        s_env.state    = false;
                        s_env.counter  = SDR_CW_EDGE_RAMP_LEN;
                        s_env.duration = s_env.counter
                                         + SDR_CW_EVENT_DELAY_FRAMES - line[i].lag
                                         + guard;
                        break;
                    }
                }
            }
            /* Mode B memory: an opposite paddle seen while an element sounds
             * yields exactly one extra element after release. */
            if (s_state.iambic_mode == SDR_CW_IAMBIC_B
                && s_keyer.element_in_progress && !s_keyer.priming) {
                if (s_keyer.element != SDR_CW_ELEM_DIT
                    && cw_paddle_down(SDR_CW_PADDLE_DIT)) {
                    s_keyer.mem_dit = true;
                }
                if (s_keyer.element != SDR_CW_ELEM_DAH
                    && cw_paddle_down(SDR_CW_PADDLE_DAH)) {
                    s_keyer.mem_dah = true;
                }
            }
            cw_env_fill_block();
            gen_sidetone();
        }
    } else {
        cw_reset_envelope();
        cw_reset_keyer();
        host_mode = false;
    }

    /* TX request to the PTT arbiter: held while an element is in progress or a
     * paddle is down, extended by the qsk tail after it ends. Training never
     * keys TX. The latch on a refused/forced RX lives in App_Ptt. */
    {
        bool active;
        bool want;

        if (host_mode) {
            active = host_held;
        } else {
            active = s_keyer.element_in_progress || any_down;
        }

        if (active) {
            s_keyer.tx_tail = cw_qsk_frames();
        } else if (s_keyer.tx_tail > DRV_CODEC_BLOCK_FRAMES) {
            s_keyer.tx_tail -= DRV_CODEC_BLOCK_FRAMES;
        } else {
            s_keyer.tx_tail = 0u;
        }

        want = cw_mode_selected() && !s_keyer.training &&
               (active || s_keyer.tx_tail > 0u);
        cw_tx_request_apply(want);
    }

    return cw_mode_selected() ? s_sidetone : NULL;
}


/* ------------------------------------------------------------------------- */
/* Demod / Mod                                                               */
/* ------------------------------------------------------------------------- */

/* Apply queued coefficient/state swaps between blocks. */
static void apply_pending(void) {
    if (s_pending.fir_reset) {
        s_pending.fir_reset = false;
        Sdr_Sideband_Reset();
    }
    if (s_pending.iir_reset) {
        s_pending.iir_reset = false;
        arm_fill_f32(0.0f, s_lpf_state, SDR_CW_FILTER_NUM_STAGES * 4);
        arm_fill_f32(0.0f, s_hpf_state, SDR_CW_FILTER_NUM_STAGES * 4);
        s_level_ema  = 0.0f;
        s_level_peak = 0.0f;
    }
    if (s_pending.lpf_switch) {
        s_pending.lpf_switch = false;
        s_lpf_id = s_lpf_id ^ 1u;
        s_lpf.pCoeffs = s_lpf_coeffs[s_lpf_id];
    }
    if (s_pending.hpf_switch) {
        s_pending.hpf_switch = false;
        s_hpf_id = s_hpf_id ^ 1u;
        s_hpf.pCoeffs = s_hpf_coeffs[s_hpf_id];
    }
}

/* Fold one block of in-band power into the S-meter: EMA for noise rejection,
 * then peak-hold so the mark level survives between meter reads. */
static void cw_level_update(float power) {
    s_level_ema += (power - s_level_ema) * s_level_alpha;
    if (s_level_ema > s_level_peak) {
        s_level_peak = s_level_ema;
    }
}

void Sdr_CW_Demod(const Sdr_IQ_Data *data, float *audio)
{
    if (!data || !audio || data->size != SDR_CW_RX_BLOCK_SIZE) {
        return;
    }
    apply_pending();

    /* Shift the carrier from DC to the sidetone pitch (CWR shifts down), then
     * let the shared one-sided FIR pick the sideband and take the real part. */
    for (uint32_t n = 0; n < data->size; ++n) {
        s_rx_i[n] = data->i[n] * s_nco_rx.real - data->q[n] * s_nco_rx.imag;
        s_rx_q[n] = data->i[n] * s_nco_rx.imag + data->q[n] * s_nco_rx.real;
        Sdr_Nco_Step(&s_nco_rx);
    }

    Sdr_IQ_Data shifted = { s_rx_i, s_rx_q, data->size };
    Sdr_Sideband_Demod(&shifted, audio, s_state.rx_mode == SDR_MOD_CWR);

    arm_biquad_cascade_df1_f32(&s_lpf, audio, audio, data->size);
    arm_biquad_cascade_df1_f32(&s_hpf, audio, audio, data->size);

    /* Squelch metric: tone peak of the band-limited audio vs the noise floor.
     * Taken after the IIR so a signal anywhere in the wide FIR passband does
     * not open the squelch. */
    float peak = 0.0f;
    for (uint32_t n = 0; n < data->size; ++n) {
        float mag = fabsf(audio[n]);
        if (mag > peak) {
            peak = mag;
        }
    }
    if (Sdr_Spectrum_Is_Noise_Valid()) {
        s_signal_metric = Sdr_Lin2Db(peak) - Sdr_Spectrum_Get_Demod_Noise_Est_Db();
    } else {
        s_signal_metric = 0.0f;
    }

    float power;
    arm_power_f32(audio, data->size, &power);
    cw_level_update(power / (float)data->size);
}

float Sdr_CW_GetSignalMetric(void)
{
    // Add offset to align with other modulations
    return s_signal_metric - 6.0f;
}

float Sdr_CW_Get_Rx_Level_Db(void)
{
    uint32_t key = Bsp_Irq_SaveLow();
    float    peak = s_level_peak;

    s_level_peak = 0.0f;
    Bsp_Irq_RestoreLow(key);

    /* peak == 0 means no demod block since the last read (TX): hold the last
     * value instead of publishing the empty-window floor. */
    if (peak > 0.0f) {
        s_level_db_last = Sdr_Pow2Db(peak) + SDR_CW_LEVEL_DB_OFFSET;
    }
    return s_level_db_last;
}

void Sdr_CW_Mod(const float *audio, Sdr_IQ_Data *data) {
    (void)audio;

    if (data == 0 || data->i == 0 || data->q == 0 || data->size == 0u) {
        return;
    }

    apply_pending();

    /* The raised-cosine envelope is produced once per codec block by
     * Sdr_CW_KeyerUpdate (so sidetone and training share it); here it just
     * drives the zero-IF carrier: real part is the envelope, Q stays 0. */
    uint32_t n = data->size;
    if (n > DRV_CODEC_BLOCK_FRAMES) {
        n = DRV_CODEC_BLOCK_FRAMES;
    }
    for (uint32_t k = 0; k < n; ++k) {
        data->i[k] = s_env_block[k];
        data->q[k] = 0.0f;
    }
}
