#include "sdr_nr.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "arm_math.h"

#include "app_event.h"
#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_memory.h"
#include "bsp_prof.h"
#include "sdr_common.h"

#define LOG_TAG "app.sdr.nr"

/* FFT, 75% overlap (hop = N/4) on the 24 kHz common RX audio chain. */
#define NR_NFFT       1024u
#define NR_HOP        NR_NFFT / 4
#define NR_OVERLAP    (NR_NFFT - NR_HOP)
#define NR_RATE_HZ    (BOARD_AUDIO_SAMPLE_RATE_HZ / 2u)
#define NR_MASK_SIZE  (NR_NFFT / 2u - 1u)

/* Overlap-add normalization: sum of a periodic Hann window and its three
 * 1/4-hop shifted copies is constant 1.5, so scale the output by 1/1.5. */
#define NR_NORM_VAL   0.6666666666666666f

/* Cap the posterior SNR so an impulse or a strong transient cannot dominate
 * the decision-directed estimate and open the gain for the whole frame. */
#define NR_GAMMA_MAX 100.0f

/* Lower bound of the soft-clip floor: at the deepest threshold the floor
 * already removes the residual, and going further only exposes gain pumping
 * and musical noise. */
#define NR_GAIN_FLOOR_MIN 0.01f

/* MCRA noise tracker: smoothed power, a sliding local minimum over subwindows,
 * a leaking long-term minimum and the speech-presence transition. The local
 * window tracks noise quickly; the long minimum forgets old outliers so a
 * spurious low sample cannot pin the estimate, and its bias lifts it from a
 * minimum to the mean noise level. */
#define NR_MCRA_SMOOTH 0.2f
#define NR_MCRA_SPP_LOW_RATIO 1.5f
#define NR_MCRA_SPP_HIGH_RATIO 6.0f
/* Fast attack opens the speech-presence gate on a rise; slow release keeps the
 * noise estimate held through short gaps between speech frames. */
#define NR_MCRA_SPP_ATTACK_ALPHA 0.9f
#define NR_MCRA_SPP_RELEASE_ALPHA 0.25f
#define NR_MCRA_ALPHA_D0 0.95f
#define NR_MCRA_ALPHA_DOWN 0.85f
#define NR_SUBWIN_FRAMES 12u
#define NR_SUBWIN_COUNT 8u
/* The long minimum drops to a new low at once and rises only by this leak per
 * frame, so its memory is bounded without a long ring. */
#define NR_MCRA_LONG_LEAK 0.001f
/* A window minimum sits below the mean noise power; scale it up so the
 * speech-presence ratio and the floor reference the mean instead. */
#define NR_MCRA_MIN_BIAS 3.7607f
/* Bound the long minimum below the local one, so a stale low sample cannot
 * hold the floor too low while the leak catches up. */
#define NR_MCRA_LONG_RELDROP 0.25f
#define NR_FLOOR_SMOOTH 0.1f
#define NR_WARMUP_FRAMES 32u

/* MMSE-LSA gain shape, exp(0.5 * E1(v)), tabulated on a log-v grid. Below the
 * table the silence asymptote is used; above it the shape is already one. */
#define NR_LSA_TABLE_SIZE 128u
#define NR_LSA_V_MIN 1e-4f
#define NR_LSA_V_MAX 100.0f
#define NR_LSA_LOG_MIN (-9.21034037f)
#define NR_LSA_LOG_SCALE 9.1925665f
#define NR_LSA_SILENCE_COEF 0.7493f
/* Lower bound on the a-priori SNR (power domain) that anchors the gain floor to
 * the LSA silence asymptote, so quiet bins settle on the floor instead of
 * clipping. */
#define NR_LSA_XI_MIN 0.5012f

/* Softening knee just above the gain floor: quiet bins settle without a hard
 * corner; above the knee the gain passes linearly so strong bins are not
 * reshaped. The knee widens with the threshold. */
#define NR_GAIN_KNEE 0.05f
#define NR_GAIN_KNEE_DEPTH 0.05f
/* Temporal gain smoothing: gain rising is slow so noise peaks do not open the
 * gain, and gain falling is fast so it closes promptly after speech ends. */
#define NR_GAIN_ATTACK 0.4f
#define NR_GAIN_RELEASE 0.9f

/* Passband edge taper: a raised-cosine amplitude roll-off on both band edges,
 * holding the edge gain across the outer bins, so a hard gain cut does not
 * narrow the filter transition band. The transition is NR_EDGE_TAPER_BINS wide
 * on each side. */
#define NR_EDGE_TAPER_BINS 5u
/* Donor window for seeding bins that enter the passband: wider than the old
 * eight so the taper zone also starts from a valid profile. */
#define NR_SEED_DONORS 12u

/* Input ring must hold the FFT window plus task-latency slack; the output ring
 * absorbs the ISR/task rate mismatch. Both power-of-two for mask indexing. */
#define NR_IN_CAP     NR_NFFT * 2u
#define NR_OUT_CAP    NR_NFFT * 2u
#define NR_OLA_CAP    NR_NFFT

#define NR_CLIP(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

typedef struct {
    float            *data;
    uint32_t          cap;
    volatile uint32_t w;
    volatile uint32_t r;
} NrRing;

static const float s_lsa_shape[NR_LSA_TABLE_SIZE] = {
    74.9343467f, 70.967805f, 67.211271f, 63.6536281f, 60.2843488f, 57.0934625f, 54.0715269f, 51.2095994f,
    48.4992112f, 45.9323415f, 43.5013945f, 41.1991767f, 39.0188752f, 36.9540382f, 34.9985553f, 33.14664f,
    31.3928121f, 29.7318815f, 28.1589334f, 26.6693131f, 25.2586124f, 23.9226569f, 22.6574932f, 21.4593774f,
    20.3247641f, 19.2502957f, 18.2327927f, 17.2692441f, 16.3567985f, 15.4927559f, 14.6745593f, 13.8997876f,
    13.1661481f, 12.4714698f, 11.813697f, 11.1908832f, 10.6011855f, 10.0428586f, 9.51425061f, 9.01379708f,
    8.54001713f, 8.09150875f, 7.66694471f, 7.26506864f, 6.88469132f, 6.52468713f, 6.18399074f, 5.86159396f,
    5.55654275f, 5.2679344f, 4.99491484f, 4.73667615f, 4.49245413f, 4.26152605f, 4.04320853f, 3.83685547f,
    3.64185618f, 3.45763356f, 3.28364235f, 3.11936757f, 2.96432293f, 2.81804943f, 2.68011393f, 2.55010792f,
    2.42764621f, 2.31236583f, 2.20392486f, 2.10200139f, 2.0062925f, 1.91651327f, 1.83239582f, 1.75368838f,
    1.68015437f, 1.61157145f, 1.54773058f, 1.48843506f, 1.43349946f, 1.38274853f, 1.33601602f, 1.29314333f,
    1.25397809f, 1.21837252f, 1.18618171f, 1.15726166f, 1.13146722f, 1.10864995f, 1.08865594f, 1.07132382f,
    1.05648298f, 1.04395233f, 1.03353985f, 1.02504305f, 1.01825075f, 1.01294608f, 1.00891088f, 1.00593115f,
    1.00380311f, 1.00233927f, 1.00137379f, 1.00076629f, 1.00040362f, 1.00019946f, 1.00009181f, 1.00003905f,
    1.00001521f, 1.00000537f, 1.0000017f, 1.00000048f, 1.00000012f, 1.00000002f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
};

static float s_in_data[NR_IN_CAP]   BSP_RAM_D1_BSS_32;
static float s_out_data[NR_OUT_CAP] BSP_RAM_D1_BSS_32;
static float s_ola_data[NR_OLA_CAP] BSP_RAM_D1_BSS_32;

static float s_z[NR_NFFT]       BSP_RAM_D1_BSS_32;
static float s_signal[NR_NFFT]  BSP_RAM_D1_BSS_32;
static float s_window[NR_NFFT]  BSP_RAM_D1_BSS_32;
static float s_mag2[NR_MASK_SIZE]      BSP_RAM_D1_BSS_32;
static float s_raw_gains[NR_MASK_SIZE] BSP_RAM_D1_BSS_32;
static float s_edge_mask[NR_MASK_SIZE] BSP_RAM_D1_BSS_32;

/* Sliding local minimum: each slot holds the smallest smoothed power seen in
 * one subwindow, and the local minimum spans the whole ring. */
static float s_ring_local[NR_SUBWIN_COUNT][NR_MASK_SIZE] BSP_RAM_D1_BSS_32;

/* s_in: written by the ISR (Process), read by the task (step1).
 * s_out: written by the task (step2), read by the ISR (Process).
 * s_ola: overlap-add accumulator, task-only. */
static NrRing s_in;
static NrRing s_out;
static NrRing s_ola;

static struct {
    arm_rfft_fast_instance_f32 rfft;

    uint32_t N;
    uint32_t hop;
    uint32_t overlap;

    float noise_psd[NR_MASK_SIZE];
    float noise_smooth[NR_MASK_SIZE];
    float noise_min[NR_MASK_SIZE];
    float noise_min_long[NR_MASK_SIZE];
    float sub_min[NR_MASK_SIZE];
    float speech_prob[NR_MASK_SIZE];
    float warmup_sum[NR_MASK_SIZE];

    uint32_t sub_cnt;
    uint32_t ring_idx;

    bool     warmup;
    uint32_t warmup_cnt;

    float gain_smooth[NR_MASK_SIZE];
    float prev_clean_mag2[NR_MASK_SIZE];

    /* Frequency-conversion inputs from the event bus; nr_setup_filters maps
     * them to the bin range. */
    SdrModulation rx_mode;
    int32_t       filt_low_hz;
    int32_t       filt_high_hz;
    int32_t       key_tone_hz;

    uint32_t filter_low_bin;
    uint32_t filter_high_bin;

    /* MMSE-LSA / decision-directed aggressiveness, mapped from the threshold. */
    float alpha_dd;
    float gain_floor;
    float knee;
    float thr;

    volatile bool enabled;
} s_nr;

static inline void nr_ring_put(NrRing *b, float v) {
    b->data[b->w & (b->cap - 1u)] = v;
    b->w++;
}

static inline void nr_ring_add(NrRing *b, float v) {
    b->data[b->w & (b->cap - 1u)] += v;
    b->w++;
}

static inline float nr_ring_get(NrRing *b) {
    float v = b->data[b->r & (b->cap - 1u)];
    b->r++;
    return v;
}

static inline float nr_ring_get_set(NrRing *b, float new_val) {
    uint32_t idx = b->r & (b->cap - 1u);
    float v = b->data[idx];
    b->data[idx] = new_val;
    b->r++;
    return v;
}

static inline void nr_ring_lsr(NrRing *b, uint32_t shift) {
    b->r -= shift;
}

static inline void nr_ring_lsw(NrRing *b, uint32_t shift) {
    b->w -= shift;
}

static inline void nr_ring_reset(NrRing *b) {
    b->w = 0u;
    b->r = 0u;
}

static inline uint32_t nr_ring_ready(const NrRing *b) {
    return (b->w - b->r) & (b->cap - 1u);
}

/* Start the startup profile from scratch: no ring slot has a sample yet and
 * the running mean is cleared. The first passband frames fill the mean. */
static void nr_warmup_restart(void) {
    s_nr.warmup = true;
    s_nr.warmup_cnt = 0u;
    arm_fill_f32(0.0f, s_nr.warmup_sum, NR_MASK_SIZE);
    arm_fill_f32(0.0f, s_nr.speech_prob, NR_MASK_SIZE);
    arm_fill_f32(INFINITY, s_nr.sub_min, NR_MASK_SIZE);
    for (uint32_t k = 0; k < NR_SUBWIN_COUNT; k++) {
        arm_fill_f32(INFINITY, s_ring_local[k], NR_MASK_SIZE);
    }
    s_nr.sub_cnt = 0u;
    s_nr.ring_idx = 0u;
}

/* Bins that enter the passband were not tracked, so their own profile is
 * stale. Seed them from the mean of the valid bins at the old edge. */
static void nr_seed_bins(uint32_t from, uint32_t to,
                         uint32_t donor_from, uint32_t donor_to) {
    float inv = 1.0f / (float)(donor_to - donor_from);
    float smooth = 0.0f;
    float mn = 0.0f;
    float mnl = 0.0f;
    float sub = 0.0f;
    float psd = 0.0f;

    for (uint32_t j = donor_from; j < donor_to; j++) {
        smooth += s_nr.noise_smooth[j];
        mn     += s_nr.noise_min[j];
        mnl    += s_nr.noise_min_long[j];
        sub    += s_nr.sub_min[j];
        psd    += s_nr.noise_psd[j];
    }
    smooth *= inv;
    mn     *= inv;
    mnl    *= inv;
    sub    *= inv;
    psd    *= inv;

    for (uint32_t i = from; i < to; i++) {
        s_nr.noise_smooth[i]   = smooth;
        s_nr.noise_min[i]      = mn;
        s_nr.noise_min_long[i] = mnl;
        s_nr.sub_min[i]        = sub;
        s_nr.noise_psd[i]      = psd;
        s_nr.speech_prob[i]    = 0.0f;
        for (uint32_t k = 0; k < NR_SUBWIN_COUNT; k++) {
            s_ring_local[k][i] = sub;
        }
    }
}

static void nr_reset(void) {
    nr_ring_reset(&s_in);
    nr_ring_reset(&s_out);
    nr_ring_reset(&s_ola);
    arm_fill_f32(0.0f, s_ola_data, NR_OLA_CAP);

    arm_fill_f32(1.0f, s_nr.gain_smooth, NR_MASK_SIZE);
    arm_fill_f32(0.0f, s_nr.prev_clean_mag2, NR_MASK_SIZE);
    arm_fill_f32(0.0f, s_nr.noise_psd, NR_MASK_SIZE);
    arm_fill_f32(0.0f, s_nr.noise_smooth, NR_MASK_SIZE);
    arm_fill_f32(0.0f, s_nr.noise_min, NR_MASK_SIZE);
    arm_fill_f32(0.0f, s_nr.noise_min_long, NR_MASK_SIZE);
    nr_warmup_restart();
}

void Sdr_NR_SetThreshold(float nrthr) {
    if (isnan(nrthr)) {
        nrthr = 0.0f;
    } else {
        nrthr = NR_CLIP(nrthr, 0.0f, 100.0f);
    }
    if (s_nr.thr != nrthr) {
        s_nr.thr = nrthr;
        /* depth range 0 ... 2 */
        float depth = s_nr.thr / 50.0f;
        /* DD smoothing: more weight on the previous clean estimate as depth rises */
        s_nr.alpha_dd = 0.95f + depth * 0.023f;
        /* noise floor left when no speech is present; the depth term tracks
         * the threshold so higher settings suppress deeper, and the lower
         * bound keeps the floor from collapsing into pumping */
        s_nr.gain_floor = MAX(powf(10.0f, (-3.0f - 17.0f * depth) / 20.0f),
                              NR_GAIN_FLOOR_MIN);
        s_nr.knee = NR_GAIN_KNEE + NR_GAIN_KNEE_DEPTH * depth;
    }
}

/* Raised-cosine taper for both band edges, stored once per band change. The
 * mask rolls each edge over NR_EDGE_TAPER_BINS bins outside the passband, holds
 * one across the passband and is zero outside the extended range. */
static void nr_edge_mask_update(void) {
    uint32_t low = s_nr.filter_low_bin;
    uint32_t high = s_nr.filter_high_bin;
    uint32_t e = NR_EDGE_TAPER_BINS;
    uint32_t lo0 = low - 1u;
    uint32_t lo_ext = (lo0 > e) ? lo0 - e : 0u;
    uint32_t hi_ext = MIN(high + e, NR_MASK_SIZE - 1u);

    arm_fill_f32(0.0f, s_edge_mask, NR_MASK_SIZE);

    for (uint32_t k = 0; k <= e; k++) {
        float t = (float)k / (float)e;
        float rise = 0.5f * (1.0f - cosf((float)M_PI * t));
        float fall = 0.5f * (1.0f + cosf((float)M_PI * t));
        if (lo_ext + k < NR_MASK_SIZE) {
            s_edge_mask[lo_ext + k] = rise;
        }
        if (k <= hi_ext) {
            s_edge_mask[hi_ext - k] = fall;
        }
    }
    for (uint32_t i = lo0; i <= high; i++) {
        s_edge_mask[i] = 1.0f;
    }
}

static void nr_setup_filters(void) {
    int32_t low = s_nr.filt_low_hz;
    int32_t high = s_nr.filt_high_hz;
    int32_t tmp;
    switch (s_nr.rx_mode) {
    case SDR_MOD_CW:
        low += s_nr.key_tone_hz;
        high += s_nr.key_tone_hz;
        break;
    case SDR_MOD_CWR:
        tmp = low;
        low = s_nr.key_tone_hz - high;
        high = s_nr.key_tone_hz - tmp;
        break;
    case SDR_MOD_AM:
    case SDR_MOD_SAM:
        high = MIN(high, -low);
        low = 0;
        break;
    default:
        break;
    }
    low = NR_CLIP(low, 50, 1500);
    high = NR_CLIP(high, low + 100, (int32_t)NR_RATE_HZ / 2 - 100);

    int32_t low_bin = low * (int32_t)NR_NFFT / (int32_t)NR_RATE_HZ;
    int32_t high_bin = high * (int32_t)NR_NFFT / (int32_t)NR_RATE_HZ + 1;

    low_bin = MAX(low_bin, 1);
    high_bin = MIN(high_bin, (int32_t)NR_MASK_SIZE - 1);
    high_bin = MAX(high_bin, low_bin + 2);

    uint32_t old_low = s_nr.filter_low_bin;
    uint32_t old_high = s_nr.filter_high_bin;
    uint32_t new_low = (uint32_t)low_bin;
    uint32_t new_high = (uint32_t)high_bin;

    /* Bins newly entering the processed range have no clean-history yet. */
    if (new_low < old_low) {
        arm_fill_f32(0.0f, s_nr.prev_clean_mag2 + new_low, old_low - new_low);
    }
    if (new_high > old_high) {
        arm_fill_f32(0.0f, s_nr.prev_clean_mag2 + old_high, new_high - old_high);
    }

    if (new_low != old_low || new_high != old_high) {
        if (s_nr.warmup) {
            nr_warmup_restart();
        } else if (old_high > old_low) {
            /* Entering bins inherit a profile from up to NR_SEED_DONORS valid
             * bins at the old edge; their own values are stale because they
             * were not tracked while outside the passband. */
            if (new_low < old_low) {
                uint32_t donor_to = MIN(old_low + NR_SEED_DONORS, old_high);
                nr_seed_bins(new_low, old_low, old_low, donor_to);
            }
            if (new_high > old_high) {
                uint32_t donor_from = (old_high > old_low + NR_SEED_DONORS)
                                          ? old_high - NR_SEED_DONORS : old_low;
                nr_seed_bins(old_high, new_high, donor_from, old_high);
            }
        }
    }

    s_nr.filter_low_bin = new_low;
    s_nr.filter_high_bin = new_high;
    nr_edge_mask_update();
}

/* Configuration subscriptions: every input update just refreshes the snapshot
 * and re-maps the bin range; a transient mix during one dispatch batch is not
 * observed because the NR task runs only after App_Event_Process returns. */
static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;
    s_nr.rx_mode      = cfg->rx_mode;
    s_nr.filt_low_hz  = cfg->filter_low_hz;
    s_nr.filt_high_hz = cfg->filter_high_hz;
    nr_setup_filters();
}

static void on_key_tone_change(const SdrKeyToneCfg *cfg, void *user) {
    (void)user;
    s_nr.key_tone_hz = (int32_t)cfg->key_tone_hz;
    nr_setup_filters();
}

static inline void update_mag2(const float *z, uint32_t i) {
    float re = z[0];
    float im = z[1];

    s_mag2[i] = re * re + im * im;
}

static void nr_noise_update(uint32_t i, float power) {
    float smooth = s_nr.noise_smooth[i] +
                   (power - s_nr.noise_smooth[i]) * NR_MCRA_SMOOTH;
    s_nr.noise_smooth[i] = smooth;

    if (smooth < s_nr.sub_min[i]) {
        s_nr.sub_min[i] = smooth;
    }

    /* Local speech presence from the distance above the long-term minimum,
     * scaled to the mean noise level; the soft transition avoids a per-frame
     * binary gate. The gate opens fast and closes slowly, so a gap between
     * speech frames does not release the noise estimate. */
    float ref = s_nr.noise_min_long[i] * NR_MCRA_MIN_BIAS;
    float ratio = smooth / (ref + 1e-15f);
    float p = NR_CLIP((ratio - NR_MCRA_SPP_LOW_RATIO) /
                      (NR_MCRA_SPP_HIGH_RATIO - NR_MCRA_SPP_LOW_RATIO),
                      0.0f, 1.0f);
    float alpha_spp = (p > s_nr.speech_prob[i]) ? NR_MCRA_SPP_ATTACK_ALPHA
                                                : NR_MCRA_SPP_RELEASE_ALPHA;
    float spp = s_nr.speech_prob[i] +
                (p - s_nr.speech_prob[i]) * alpha_spp;
    s_nr.speech_prob[i] = spp;

    /* Hold the estimate while speech is likely; update it toward the current
     * power in pauses. */
    float alpha_d = NR_MCRA_ALPHA_D0 +
                    (1.0f - NR_MCRA_ALPHA_D0) * spp;
    /* Let the estimate fall faster than it rises, so a pause does not leave the
     * noise floor stuck high after the signal drops. */
    if (power < s_nr.noise_psd[i]) {
        alpha_d = MIN(alpha_d, NR_MCRA_ALPHA_DOWN);
    }
    s_nr.noise_psd[i] += (power - s_nr.noise_psd[i]) * (1.0f - alpha_d);

    /* Pull the estimate toward the floor smoothly instead of jumping it up on
     * every frame. */
    if (s_nr.noise_psd[i] < ref) {
        s_nr.noise_psd[i] += (ref - s_nr.noise_psd[i]) * NR_FLOOR_SMOOTH;
    }
}

static void nr_subwindow_step(void) {
    uint32_t low = s_nr.filter_low_bin;
    uint32_t high = s_nr.filter_high_bin;

    s_nr.sub_cnt++;
    if (s_nr.sub_cnt >= NR_SUBWIN_FRAMES) {
        for (uint32_t i = low; i < high; i++) {
            s_ring_local[s_nr.ring_idx][i] = s_nr.sub_min[i];
            s_nr.sub_min[i] = INFINITY;
        }
        s_nr.ring_idx = (s_nr.ring_idx + 1u) % NR_SUBWIN_COUNT;
        s_nr.sub_cnt = 0u;
    }

    for (uint32_t i = low; i < high; i++) {
        float mn = s_nr.sub_min[i];
        for (uint32_t k = 0; k < NR_SUBWIN_COUNT; k++) {
            mn = MIN(mn, s_ring_local[k][i]);
        }
        s_nr.noise_min[i] = mn;

        /* The long minimum follows any new low at once and otherwise rises
         * only by the leak, so an old outlier is forgotten instead of pinning
         * the floor. The relative clamp keeps it from staying too far below
         * the local minimum. */
        float lm = s_nr.noise_min_long[i] * (1.0f + NR_MCRA_LONG_LEAK);
        lm = MIN(lm, s_nr.noise_smooth[i]);
        lm = MAX(lm, mn * NR_MCRA_LONG_RELDROP);
        s_nr.noise_min_long[i] = lm;
    }
}

/* Clamp the gain to [floor, 1] with a smoothstep knee of width knee just above
 * the floor; above the knee the gain passes linearly. The ceiling is implicit
 * because the LSA gain never exceeds one. */
static inline float nr_soft_clip(float g, float floor, float knee) {
    if (g <= floor) {
        return floor;
    }
    if (g >= floor + knee) {
        return g;
    }
    float x = (g - floor) / knee;
    return floor + knee * (x * x * (3.0f - 2.0f * x));
}

static inline float nr_lsa_shape(float v) {
    if (v < NR_LSA_V_MIN) {
        float vv = v > 1e-20f ? v : 1e-20f;
        return NR_LSA_SILENCE_COEF / sqrtf(vv);
    }
    if (v > NR_LSA_V_MAX) {
        return 1.0f;
    }

    float t = (logf(v) - NR_LSA_LOG_MIN) * NR_LSA_LOG_SCALE;
    if (t < 0.0f) {
        t = 0.0f;
    }
    uint32_t idx = (uint32_t)t;
    if (idx >= NR_LSA_TABLE_SIZE - 1u) {
        return s_lsa_shape[NR_LSA_TABLE_SIZE - 1u];
    }
    float frac = t - (float)idx;
    return s_lsa_shape[idx] + (s_lsa_shape[idx + 1u] - s_lsa_shape[idx]) * frac;
}

static void nr_step1(void) {
    /* Apply window, forward FFT, update the noise PSD estimate. */
    for (uint32_t i = 0; i < s_nr.N; i++) {
        s_signal[i] = s_window[i] * nr_ring_get(&s_in);
    }
    nr_ring_lsr(&s_in, s_nr.overlap);

    arm_rfft_fast_f32(&s_nr.rfft, s_signal, s_z, 0u);

    /* Drop DC and Nyquist. */
    s_z[0] = 0.0f;
    s_z[1] = 0.0f;

    uint32_t low = s_nr.filter_low_bin;
    uint32_t high = s_nr.filter_high_bin;

    for (uint32_t i = low; i < high; i++) {
        update_mag2(&s_z[i << 1u], i);
    }

    if (s_nr.warmup) {
        /* Collect a startup mean per bin; the estimate follows it while the
         * sliding windows have no history yet. */
        for (uint32_t i = low; i < high; i++) {
            s_nr.warmup_sum[i] += s_mag2[i];
        }
        s_nr.warmup_cnt++;
        if (s_nr.warmup_cnt >= NR_WARMUP_FRAMES) {
            for (uint32_t i = low; i < high; i++) {
                float m = s_nr.warmup_sum[i] * (1.0f / (float)NR_WARMUP_FRAMES);
                s_nr.noise_smooth[i]   = m;
                s_nr.noise_min[i]      = m;
                s_nr.noise_min_long[i] = m;
                s_nr.sub_min[i]        = m;
                s_nr.noise_psd[i]      = m;
                s_nr.speech_prob[i]    = 0.0f;
                for (uint32_t k = 0; k < NR_SUBWIN_COUNT; k++) {
                    s_ring_local[k][i] = m;
                }
            }
            s_nr.sub_cnt = 0u;
            s_nr.ring_idx = 0u;
            s_nr.warmup = false;
        } else {
            float inv = 1.0f / (float)s_nr.warmup_cnt;
            for (uint32_t i = low; i < high; i++) {
                s_nr.noise_psd[i] = s_nr.warmup_sum[i] * inv;
            }
        }
        return;
    }

    for (uint32_t i = low; i < high; i++) {
        nr_noise_update(i, s_mag2[i]);
    }
    nr_subwindow_step();
}

static void nr_step2(void) {
    /* Per-bin MMSE-LSA gain from the decision-directed a-priori SNR, then the
     * noise floor, frequency spreading, temporal smoothing and the passband
     * edge taper. */
    uint32_t low = s_nr.filter_low_bin;
    uint32_t high = s_nr.filter_high_bin;

    uint32_t e = NR_EDGE_TAPER_BINS;
    uint32_t lo0 = low - 1u;
    uint32_t lo_ext = (lo0 > e) ? lo0 - e : 0u;
    uint32_t hi_ext = MIN(high + e, NR_MASK_SIZE - 1u);

    for (size_t i = 0; i < lo_ext; i++) {
        s_z[i * 2u] = 0.0f;
        s_z[i * 2u + 1u] = 0.0f;
    }

    if (s_nr.warmup) {
        /* Unity gain while the profile warms up: the signal stays audible and
         * the decision-directed memory follows the raw power, so suppression
         * does not start from a stale estimate. */
        arm_fill_f32(1.0f, s_nr.gain_smooth + lo0, (high - low) + 2u);
        for (uint32_t i = low; i < high; i++) {
            s_nr.prev_clean_mag2[i] = s_mag2[i];
        }
    } else {
        arm_fill_f32(0.0f, s_raw_gains, high + 1u);

        for (uint32_t i = low; i < high; i++) {
            float noise = s_nr.noise_psd[i] + 1e-15f;
            float gamma = MIN(s_mag2[i] / noise, NR_GAMMA_MAX);

            /* Decision-directed a-priori SNR; the instantaneous term is the
             * excess above unity. */
            float gamma_os = gamma - 1.0f;
            if (gamma_os < 0.0f) {
                gamma_os = 0.0f;
            }

            float xi = s_nr.alpha_dd * (s_nr.prev_clean_mag2[i] / noise) +
                       (1.0f - s_nr.alpha_dd) * gamma_os;

            /* The LSA silence asymptote is 0.749*sqrt(xi); anchoring xi_min to
             * the gain floor makes the gain settle on the floor instead of
             * clipping. */
            float anchor = s_nr.gain_floor / NR_LSA_SILENCE_COEF;
            float xi_floor = anchor * anchor * NR_LSA_XI_MIN;
            if (xi < xi_floor) {
                xi = xi_floor;
            }

            float u = xi / (1.0f + xi);
            float v = u * gamma;
            float gain = u * nr_lsa_shape(v);
            gain = nr_soft_clip(gain, s_nr.gain_floor, s_nr.knee);

            /* Spread each bin's gain over its two neighbours
             * (0.5 / 0.25 / 0.25). */
            s_raw_gains[i] += gain * 0.5f;
            s_raw_gains[i - 1u] += gain * 0.25f;
            s_raw_gains[i + 1u] += gain * 0.25f;
        }

        for (uint32_t i = lo0; i < high + 1u; i++) {
            float target = s_raw_gains[i];
            float lambda = (target > s_nr.gain_smooth[i]) ? NR_GAIN_ATTACK
                                                          : NR_GAIN_RELEASE;
            s_nr.gain_smooth[i] += (target - s_nr.gain_smooth[i]) * lambda;
        }

        /* DD memory: previous clean power, taken from the applied gain. */
        for (uint32_t i = low; i < high; i++) {
            s_nr.prev_clean_mag2[i] =
                s_nr.gain_smooth[i] * s_nr.gain_smooth[i] * s_mag2[i];
        }
    }

    /* The smoothed gain is applied over the extended band: bins outside the
     * passband hold the adjacent edge gain, and the raised-cosine mask rolls
     * the band edges off. */
    for (uint32_t i = lo_ext; i <= hi_ext; i++) {
        float g;
        if (i < lo0) {
            g = s_nr.gain_smooth[lo0];
        } else if (i > high) {
            g = s_nr.gain_smooth[high];
        } else {
            g = s_nr.gain_smooth[i];
        }
        g *= s_edge_mask[i];
        s_z[i * 2u] *= g;
        s_z[i * 2u + 1u] *= g;
    }

    for (size_t i = hi_ext + 1u; i < NR_MASK_SIZE; i++) {
        s_z[i * 2u] = 0.0f;
        s_z[i * 2u + 1u] = 0.0f;
    }

    arm_rfft_fast_f32(&s_nr.rfft, s_z, s_signal, 1u);

    for (uint32_t i = 0; i < s_nr.N; i++) {
        nr_ring_add(&s_ola, s_signal[i] * s_window[i]);
    }
    nr_ring_lsw(&s_ola, s_nr.overlap);

    for (uint32_t i = 0; i < s_nr.hop; i++) {
        float val = nr_ring_get_set(&s_ola, 0.0f);
        nr_ring_put(&s_out, val * NR_NORM_VAL);
    }
}

/* Main-loop task: drain every full FFT window that accumulated in the input
 * ring. Runs one or more frames per invocation so a coalesced event does not
 * leave a backlog behind. */
static void nr_task(void) {
    BSP_PROF_SECTION_BEGIN(BSP_PROF_SECTION_NR, prof_start);

    for (;;) {
        uint32_t key;
        uint32_t ready;

        key = Bsp_Irq_SaveLow();
        ready = nr_ring_ready(&s_in);
        Bsp_Irq_RestoreLow(key);

        if (ready < NR_NFFT) {
            break;
        }

        nr_step1();
        nr_step2();
    }

    BSP_PROF_SECTION_END(BSP_PROF_SECTION_NR, prof_start);
}

void Sdr_NR_Init(void) {
    s_in.data = s_in_data;
    s_in.cap = NR_IN_CAP;
    s_out.data = s_out_data;
    s_out.cap = NR_OUT_CAP;
    s_ola.data = s_ola_data;
    s_ola.cap = NR_OLA_CAP;

    arm_rfft_fast_init_1024_f32(&s_nr.rfft);
    s_nr.N = NR_NFFT;
    s_nr.hop = NR_HOP;
    s_nr.overlap = NR_OVERLAP;

    /* Periodic Hann (divide by N): matches the precomputed reference tables and
     * keeps the hop = N/4 overlap-add normalization NR_NORM_VAL valid. */
    for (uint32_t i = 0; i < NR_NFFT; i++) {
        s_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i /
                                          (float)NR_NFFT));
    }

    /* Fixed defaults; s_nr.thr is forced invalid so the first call recomputes. */
    s_nr.thr = -1.0f;
    Sdr_NR_SetThreshold(0.0f);

    s_nr.rx_mode      = SDR_MOD_LSB;
    s_nr.filt_low_hz  = 300;
    s_nr.filt_high_hz = 2700;
    s_nr.key_tone_hz  = 600;
    nr_setup_filters();

    nr_reset();
    s_nr.enabled = false;

    if (App_Scheduler_AddEvent(nr_task, APP_SCHED_EV_NR_CALC) != 0) {
        LOG_WARN("nr task not registered");
    }
    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
    if (App_Event_SubscribeKeyTone(on_key_tone_change, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeKeyTone failed");
    }
}

void Sdr_NR_SetEnabled(bool enabled) {
    uint32_t key = Bsp_Irq_Save();
    bool restarted = false;

    if (enabled != s_nr.enabled) {
        s_nr.enabled = enabled;
        if (!enabled) {
            /* Drop only the audio rings: the noise profile is kept so a later
             * enable resumes without a warm-up. */
            nr_ring_reset(&s_in);
            nr_ring_reset(&s_out);
            nr_ring_reset(&s_ola);
            arm_fill_f32(0.0f, s_ola_data, NR_OLA_CAP);
        } else {
            restarted = true;
        }
    }

    Bsp_Irq_Restore(key);

    /* A profile that had not finished warming up is not valid after a pause. */
    if (restarted && s_nr.warmup) {
        nr_warmup_restart();
    }
}

void Sdr_NR_Process(float *samples, uint32_t n) {
    if (samples == 0 || !s_nr.enabled) {
        return;
    }

    for (uint32_t i = 0; i < n; i++) {
        float in = samples[i];

        nr_ring_put(&s_in, in);

        if (s_out.r != s_out.w) {
            float out = s_out.data[s_out.r & (s_out.cap - 1u)];
            s_out.r++;
            samples[i] = out;
        } else {
            samples[i] = 0.0f;
        }
    }

    /* The ISR is the only writer of s_in.w and cannot be preempted by the
     * task, so the ready count is read without masking. */
    if (nr_ring_ready(&s_in) >= NR_NFFT) {
        App_Scheduler_Notify(APP_SCHED_EV_NR_CALC);
    }
}
