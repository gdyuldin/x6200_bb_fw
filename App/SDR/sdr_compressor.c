#include "sdr_compressor.h"

#include <math.h>

#include "board_config.h"
#include "bsp_log.h"
#include "bsp_irq.h"
#include "sdr_common.h"
#include "app_event.h"

#define LOG_TAG "app.sdr.compressor"

#define SDR_COMPRESSOR_FS BOARD_AUDIO_SAMPLE_RATE_HZ

#define SDR_COMPRESSOR_UNITY_LEVEL (0.0f)

#define SDR_COMPRESSOR_ATT_TAU_MS 2u
#define SDR_COMPRESSOR_RELEASE_TAU_MS 250u

/* Lookahead in samples: the detector must settle before the delayed sample
 * reaches the output, so the gain is already correct when it is applied. */
#define SDR_COMPRESSOR_COMP_DELAY \
    (2u * SDR_COMPRESSOR_ATT_TAU_MS * SDR_COMPRESSOR_FS / 1000)

/* Downward expansion ratio of the gate below threshold. */
#define SDR_COMPRESSOR_RATIO_GATE 0.25f
#define SDR_COMPRESSOR_INV_RATIO_GATE (1.0f / SDR_COMPRESSOR_RATIO_GATE)
/* Cap on gate suppression so quiet passages are not pushed into silence. */
#define SDR_COMPRESSOR_MAX_GATE_SUPPRESSION 40.0f

#define SDR_COMPRESSOR_TH (-32.0f)
#define SDR_COMPRESSOR_DEFAULT_RATIO 4.0f

/* Soft-knee width around the threshold, in dB. */
#define SDR_COMPRESSOR_KNEE_WIDTH_DB 4.0f

/* One lookahead slot carries both the sample and its power, so the sliding
 * RMS window and the delayed sample share a single ring. */
typedef struct {
    float sample;
    float power;
} SdrCompressorDelayEntry;

typedef struct {
    SdrCompressorDelayEntry data[SDR_COMPRESSOR_COMP_DELAY];
    uint32_t                w;
    uint32_t                r;
    uint32_t                size;
} SdrCompressorRing;

typedef struct {
    SdrCompressorRing delay_ring;
    float             power_sum;
    float             attack_alpha;
    float             release_alpha;
    float             level_comp;
    float             level_gate;
    float             gain_db;
    float             threshold;
    float             ratio;
    float             makeup;
    float             th_low;
    float             th_high;
    float             x1;
    float             knee_a;
    float             rms_in_db;
    float             rms_out_db;
    float             applied_db;
    bool              enabled;
} SdrCompressor;

static SdrCompressor s_comp;

static AppRadioMode s_radio_mode = APP_RADIO_MODE_OFF;


static void ring_put(SdrCompressorRing *ring, float sample, float power)
{
    ring->data[ring->w].sample = sample;
    ring->data[ring->w].power  = power;
    ring->w = (ring->w + 1u == ring->size) ? 0u : ring->w + 1u;
}

static SdrCompressorDelayEntry ring_get(SdrCompressorRing *ring)
{
    SdrCompressorDelayEntry entry = ring->data[ring->r];

    ring->r = (ring->r + 1u == ring->size) ? 0u : ring->r + 1u;
    return entry;
}

/* The ring reports full once w wraps onto r; it is only queried after a
 * put, so the initial w == r state is never read as full. */
static bool ring_full(const SdrCompressorRing *ring)
{
    return ring->w == ring->r;
}

static void ring_reset(SdrCompressorRing *ring)
{
    ring->w = 0u;
    ring->r = 0u;
}

/* Recompute makeup and the soft-knee points after a threshold or ratio change.
 * Makeup is the static gain that lifts a signal at SDR_COMPRESSOR_UNITY_LEVEL
 * back to unity after compression, minus the configured headroom. */
static void update_curve(void)
{
    SdrCompressor *c = &s_comp;
    float inv_ratio = 1.0f / c->ratio;
    float inv_gate  = SDR_COMPRESSOR_INV_RATIO_GATE;
    float u;

    c->makeup  = (SDR_COMPRESSOR_UNITY_LEVEL - c->threshold) * (1.0f - inv_ratio);
    c->th_low  = c->threshold - SDR_COMPRESSOR_KNEE_WIDTH_DB * 0.5f;
    c->th_high = c->threshold + SDR_COMPRESSOR_KNEE_WIDTH_DB * 0.5f;

    u         = (1.0f - inv_gate) / (inv_ratio - inv_gate);
    c->x1     = c->th_low + u * SDR_COMPRESSOR_KNEE_WIDTH_DB;
    c->knee_a = (inv_ratio - inv_gate) / (2.0f * SDR_COMPRESSOR_KNEE_WIDTH_DB);
}

static void reset(void)
{
    SdrCompressor *c = &s_comp;

    ring_reset(&c->delay_ring);
    c->power_sum  = 0.0f;
    c->level_comp = 0.0f;
    c->level_gate = 0.0f;
    c->gain_db    = 0.0f;
    c->rms_in_db  = -120.0f;
    c->rms_out_db = -120.0f;
    c->applied_db = 0.0f;
}

/* Events subscriptions */

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (evt->mode != APP_RADIO_MODE_TX && s_radio_mode != evt->mode) {
        // Reset after TX end
        uint32_t key = Bsp_Irq_SaveLow();
        reset();
        Bsp_Irq_RestoreLow(key);
    }
    s_radio_mode = evt->mode;
}


void Sdr_Compressor_Init(void)
{
    SdrCompressor *c = &s_comp;

    c->delay_ring.size    = SDR_COMPRESSOR_COMP_DELAY;
    c->attack_alpha  = 1.0f - expf(-1000.0f /
                                   (SDR_COMPRESSOR_ATT_TAU_MS * SDR_COMPRESSOR_FS));
    c->release_alpha = 1.0f - expf(-1000.0f /
                                   (SDR_COMPRESSOR_RELEASE_TAU_MS * SDR_COMPRESSOR_FS));
    c->ratio         = SDR_COMPRESSOR_DEFAULT_RATIO;
    c->threshold     = SDR_COMPRESSOR_TH;
    c->enabled       = true;

    reset();
    update_curve();

    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
}

void Sdr_Compressor_SetRatio(uint8_t ratio)
{
    uint32_t key;

    key = Bsp_Irq_SaveLow();
    s_comp.ratio = MAX((float)ratio, 1.0f);
    update_curve();
    Bsp_Irq_RestoreLow(key);
}

void Sdr_Compressor_SetThreshold(float db)
{
    uint32_t key;

    db = MIN(db, SDR_COMPRESSOR_UNITY_LEVEL - 3.0f);
    key = Bsp_Irq_SaveLow();
    s_comp.threshold = db;
    update_curve();
    Bsp_Irq_RestoreLow(key);
}

void Sdr_Compressor_ResetThreshold(void)
{
    Sdr_Compressor_SetThreshold(SDR_COMPRESSOR_TH);
}

void Sdr_Compressor_Process(float *audio, uint32_t size)
{
    SdrCompressor *c = &s_comp;

    if (!c->enabled || audio == 0 || size == 0u) {
        return;
    }

    for (uint32_t n = 0u; n < size; n++) {
        float val   = audio[n];
        float power = val * val;

        c->power_sum += power;
        ring_put(&c->delay_ring, val, power);

        if (!ring_full(&c->delay_ring)) {
            /* Lookahead still filling; emit silence until the window is valid. */
            audio[n] = 0.0f;
            continue;
        }

        SdrCompressorDelayEntry delayed = ring_get(&c->delay_ring);
        c->power_sum -= delayed.power;
        float power_mean = c->power_sum / (float)c->delay_ring.size;

        float rms_db;
        if (power_mean >= 0.0f) {
            rms_db = Sdr_Pow2Db(power_mean);
        } else {
            /* Running sum of squares can drift slightly negative. */
            rms_db = -120.0f;
        }

        float gain_comp = rms_db;
        float gain_gate = rms_db;

        if (rms_db < c->th_low) {
            gain_gate = c->threshold +
                        (rms_db - c->threshold) / SDR_COMPRESSOR_RATIO_GATE;
            if (rms_db - gain_gate > SDR_COMPRESSOR_MAX_GATE_SUPPRESSION) {
                gain_gate = rms_db - SDR_COMPRESSOR_MAX_GATE_SUPPRESSION;
            }
        } else if (rms_db < c->th_high) {
            float d    = rms_db - c->th_low;
            float knee = c->threshold +
                         (rms_db - c->threshold) * SDR_COMPRESSOR_INV_RATIO_GATE +
                         c->knee_a * d * d;

            /* Below x1 the knee still expands (slope > 1): gate detector.
             * Above x1 it compresses (slope < 1): compressor detector. */
            if (rms_db < c->x1) {
                gain_gate = knee;
            } else {
                gain_comp = knee;
            }
        } else {
            gain_comp = c->threshold + (rms_db - c->threshold) / c->ratio;
            /* Limiter branch */
            gain_comp = MIN(gain_comp, -c->makeup);
        }

        /* Level detector: instant attack (MAX/MIN), one-pole release. */
        float comp_change = rms_db - gain_comp;
        c->level_comp = MAX(comp_change,
                            (1.0f - c->release_alpha) * c->level_comp +
                                c->release_alpha * comp_change);
        float gate_change = rms_db - gain_gate;
        c->level_gate = MIN(gate_change,
                            (1.0f - c->release_alpha) * c->level_gate +
                                c->release_alpha * gate_change);

        c->gain_db = (1.0f - c->attack_alpha) * c->gain_db +
                     c->attack_alpha * (c->level_gate + c->level_comp);

        float corr_db = -c->gain_db + c->makeup;

        c->rms_in_db  = rms_db;
        c->applied_db = corr_db;
        c->rms_out_db = rms_db + corr_db;

        audio[n] = delayed.sample * Sdr_Db2Lin(corr_db);
    }
}

void Sdr_Compressor_Enable(bool on)
{
    s_comp.enabled = on;
}

void Sdr_Compressor_GetStats(SdrCompressorStats *out)
{
    uint32_t irq;

    if (out == 0) {
        return;
    }

    irq = Bsp_Irq_SaveLow();
    out->rms_in_db  = s_comp.rms_in_db;
    out->rms_out_db = s_comp.rms_out_db;
    out->applied_db = s_comp.applied_db;
    out->threshold  = s_comp.threshold;
    out->enabled    = s_comp.enabled;
    Bsp_Irq_RestoreLow(irq);
}
