#include "sdr_notch.h"

#include <stdbool.h>
#include <string.h>

#include "arm_math.h"
#include "board_config.h"
#include "bsp_irq.h"

#define SDR_NOTCH_SAMPLE_RATE_DIVISOR 2u

#define SDR_NOTCH_SAMPLE_RATE_HZ \
    (BOARD_AUDIO_SAMPLE_RATE_HZ / SDR_NOTCH_SAMPLE_RATE_DIVISOR)

#define SDR_NOTCH_NUM_STAGES 1


static arm_biquad_casd_df1_inst_f32 s_filter;
static float s_filter_coeffs[SDR_NOTCH_NUM_STAGES * 5];
static float s_filter_state[SDR_NOTCH_NUM_STAGES * 4];

static struct {
    /* Auto (ANF) tracking state */
    float mean2;
    float an;
    float an_min;
    float an_max;
    /* Per-sample LMS step and power smoothing; constant, set in Init */
    float k;
    float mean2_alpha;
    /* Width-derived notch shape, shared by both modes */
    float gain;
    float k2;
    /* Config */
    SdrNotchMode mode;
    float center;
    float width;
} s_state;


/* t = tan(pi*width/fs); k2 = (1-t)/(1+t) sets the notch shape from the width.
 * The manual and auto coefficients both derive from it, so both respect the
 * requested width. */
static float width_to_k2(float width) {
    float t = tanf((float)M_PI * width / (float)SDR_NOTCH_SAMPLE_RATE_HZ);
    return (1.0f - t) / (1.0f + t);
}

/* an = 2*cos(w) is the frequency-dependent coefficient (zero pair on the unit
 * circle at w = 2*pi*center/fs). */
static float center_to_an(float center) {
    float w = 2.0f * (float)M_PI * center / (float)SDR_NOTCH_SAMPLE_RATE_HZ;
    return 2.0f * cosf(w);
}

/* b0, b2 and a2 are fixed by the width; only b1 and a1 follow an (the zero
 * pair position). Manual mode fixes an, auto mode adapts it per sample. */
static void apply_coeffs(void) {
    float gain = s_state.gain;
    float b1 = -s_state.an * gain;
    s_filter_coeffs[0] = gain;
    s_filter_coeffs[1] = b1;
    s_filter_coeffs[2] = gain;
    s_filter_coeffs[3] = -b1;
    s_filter_coeffs[4] = -s_state.k2;
}

static void reset_filter(void) {
    memset(s_filter_state, 0, sizeof(s_filter_state));
}

/* One auto-mode sample: biquad step plus the LMS coefficient update. */
static float anf_step(float x) {
    float *c = s_filter_coeffs;
    float *st = s_filter_state;

    float y = c[0] * x + c[1] * st[0] + c[2] * st[1] + c[3] * st[2] + c[4] * st[3];

    /* Shift state to the CMSIS layout {x1, x2, y1, y2}. */
    st[1] = st[0];
    st[0] = x;
    st[3] = st[2];
    st[2] = y;

    if (!isfinite(st[2]) || !isfinite(st[3])) {
        reset_filter();
        return 0.0f;
    }

    float inst = st[0] * st[0] + st[1] * st[1];
    s_state.mean2 += (inst - s_state.mean2) * s_state.mean2_alpha;

    float gradient = st[2] * (s_state.gain * st[3] - st[1]);
    float an = s_state.an - s_state.k * gradient / (s_state.mean2 + 1e-12f);
    an = MIN(s_state.an_max, MAX(s_state.an_min, an));
    s_state.an = an;

    float b1 = -an * s_state.gain;
    c[1] = b1;
    c[3] = -b1;

    return y;
}


void Sdr_Notch_Init(void) {
    arm_biquad_cascade_df1_init_f32(&s_filter, SDR_NOTCH_NUM_STAGES,
                                    s_filter_coeffs, s_filter_state);

    /* ANF tracking range in an = 2*cos(2*pi*f/fs) units; f in [4000, 400]. */
    s_state.an_min = center_to_an(4000.0f);
    s_state.an_max = center_to_an(400.0f);

    /* Per-sample LMS step and mean2 smoothing tuned for 12500 Hz, scaled by
     * fs_new/fs_old so the physical tracking time stays the same. */
    s_state.k = 3e-3f * 12500.0f / (float)SDR_NOTCH_SAMPLE_RATE_HZ;
    s_state.mean2_alpha =
        0.01f * 12500.0f / (float)SDR_NOTCH_SAMPLE_RATE_HZ;

    s_state.gain = 1.0f;
    s_state.k2 = 1.0f;
}

/* Called from the main loop while Sdr_Notch_Process() runs in the I2S DMA ISR.
 * The transcendental work stays outside the critical section; only the state
 * swap is serialized, so the ISR never observes a half-applied config or a
 * torn coefficient set. */
void Sdr_Notch_SetParams(SdrNotchMode mode, float center, float width) {
    float k2   = width_to_k2(width);
    float gain = (1.0f + k2) / 2.0f;
    float an   = center_to_an(center);

    uint32_t key = Bsp_Irq_SaveLow();

    bool width_changed = s_state.width != width;
    bool mode_changed  = s_state.mode != mode;

    s_state.mode = mode;
    s_state.center = center;
    s_state.width = width;

    if (width_changed) {
        s_state.k2 = k2;
        s_state.gain = gain;
    }

    switch (mode) {
        case SDR_NOTCH_MODE_OFF:
            reset_filter();
            break;
        case SDR_NOTCH_MODE_MANUAL:
            s_state.an = an;
            apply_coeffs();
            break;
        default: /* SDR_NOTCH_MODE_AUTO */
            if (mode_changed) {
                s_state.an = MIN(s_state.an_max, MAX(s_state.an_min, an));
                s_state.mean2 = 0.0f;
            }
            apply_coeffs();
            break;
    }

    Bsp_Irq_RestoreLow(key);
}


void Sdr_Notch_Process(float *samples, uint32_t size) {
    if (s_state.mode == SDR_NOTCH_MODE_OFF) {
        return;
    }
    if (s_state.mode == SDR_NOTCH_MODE_MANUAL) {
        arm_biquad_cascade_df1_f32(&s_filter, samples, samples, size);
        return;
    }
    for (uint32_t i = 0; i < size; i++) {
        samples[i] = anf_step(samples[i]);
    }
}
