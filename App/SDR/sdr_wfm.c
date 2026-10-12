#include "sdr_wfm.h"

#include <stdint.h>

#include <arm_math.h>

#include "app_event.h"
#include "board_config.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "drv_codec.h"
#include "sdr_fm_disc.h"
#include "sdr_halfband_decim.h"
#include "sdr_resampling_filters.h"
#include "sdr_wfm_filters.h"

#define LOG_TAG "app.sdr.wfm"

/* IQ rate and block.  The FPGA streams 4 samples per 48 kHz tick at fftdec=0
 * (after the 384 -> 192 kHz channel decimation done by the RX frontend). */
#define SDR_WFM_RATE (BOARD_AUDIO_SAMPLE_RATE_HZ * 4u)
#define SDR_WFM_BLOCK_SIZE (DRV_CODEC_BLOCK_FRAMES * 4u)
#define SDR_WFM_DEC1_SIZE (SDR_WFM_BLOCK_SIZE / 2u)
#define SDR_WFM_AUDIO_SIZE (DRV_CODEC_BLOCK_FRAMES)

#define SDR_WFM_DEVIATION 75000u

/* De-emphasis is applied after decimation, at the audio rate. */
#define SDR_WFM_DC_BLOCK_FREQ (20.0f)

/* Bound the phase unwrap carry so it does not lose float precision.  Smaller
 * than the NBFM value: the WFM deviation makes the phase advance ~7x faster
 * per sample at 192 kHz than NBFM does at 48 kHz. */
#define SDR_WFM_PHASE_TURNS_THRESHOLD 100u

#define SDR_WFM_FIR_DIFF_STATE_SIZE \
    (SDR_WFM_BLOCK_SIZE + SDR_WFM_FIR_NUM_TAPS - 1u)
#define SDR_WFM_DEC1_STATE_SIZE \
    (SDR_DECIM_192_15_NUM_TAPS + SDR_WFM_BLOCK_SIZE - 1u)
#define SDR_WFM_DEC2_STATE_SIZE \
    (SDR_DECIM_96_15_NUM_TAPS + SDR_WFM_DEC1_SIZE - 1u)

_Static_assert(SDR_WFM_BLOCK_SIZE / 4u == DRV_CODEC_BLOCK_FRAMES,
               "WFM audio block must equal the codec block");

typedef struct {
    float b0, b1;
    float a1;
    float x1, y1;
} FirstOrderIIR;

static Sdr_FmDisc s_disc;
static float s_fir_diff_state[SDR_WFM_FIR_DIFF_STATE_SIZE]
    __attribute__((aligned(32)));

static SdrHalfbandDecim s_dec1;
static float s_dec1_state[SDR_WFM_DEC1_STATE_SIZE] __attribute__((aligned(32)));
static SdrHalfbandDecim s_dec2;
static float s_dec2_state[SDR_WFM_DEC2_STATE_SIZE] __attribute__((aligned(32)));

static float s_disc_buf[SDR_WFM_BLOCK_SIZE] __attribute__((aligned(32)));
static float s_dec1_buf[SDR_WFM_DEC1_SIZE] __attribute__((aligned(32)));

static FirstOrderIIR de = SDR_WFM_DE_IIR_INIT;
static Sdr_DC_Filter s_dc;

static struct {
    AppRadioMode radio_mode;
    SdrModulation rx_mode;
} s_state;

static struct {
    bool reset_rx;
} s_pending;

static float first_order_iir(FirstOrderIIR *f, float x) {
    float y = f->b0 * x + f->b1 * f->x1 - f->a1 * f->y1;
    f->x1 = x;
    f->y1 = y;
    return y;
}

static void on_rx_tx(const AppEventRxTxEvent *evt, void *user) {
    (void)user;
    if (s_state.radio_mode != evt->mode) {
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.reset_rx = true;
        Bsp_Irq_RestoreLow(key);
    }
    s_state.radio_mode = evt->mode;
}

static void on_rx_cfg(const SdrRxCfg *cfg, void *user) {
    (void)user;
    if (cfg->rx_mode != s_state.rx_mode) {
        uint32_t key = Bsp_Irq_SaveLow();
        s_pending.reset_rx = true;
        Bsp_Irq_RestoreLow(key);
        s_state.rx_mode = cfg->rx_mode;
    }
}

static void apply_pending(void) {
    if (s_pending.reset_rx) {
        s_pending.reset_rx = false;
        de.x1 = 0.0f;
        de.y1 = 0.0f;
        Sdr_DC_Filter_Reset(&s_dc);
        Sdr_FmDisc_Reset(&s_disc);
        Sdr_HalfbandDecim_Reset(&s_dec1);
        Sdr_HalfbandDecim_Reset(&s_dec2);
    }
}

void Sdr_WFM_Init(void) {
    Sdr_FmDisc_Init(&s_disc, sdr_wfm_fir_coeffs, SDR_WFM_FIR_NUM_TAPS,
                    s_fir_diff_state, SDR_WFM_BLOCK_SIZE,
                    (float)SDR_WFM_PHASE_TURNS_THRESHOLD * (float)M_TWOPI);

    Sdr_HalfbandDecim_Init(&s_dec1, sdr_decim_192_15_coeffs,
                           SDR_DECIM_192_15_NUM_TAPS, s_dec1_state,
                           SDR_WFM_BLOCK_SIZE);
    Sdr_HalfbandDecim_Init(&s_dec2, sdr_decim_96_15_coeffs,
                           SDR_DECIM_96_15_NUM_TAPS, s_dec2_state,
                           SDR_WFM_DEC1_SIZE);

    s_dc.alpha = (float)M_TWOPI * SDR_WFM_DC_BLOCK_FREQ /
                 (float)BOARD_AUDIO_SAMPLE_RATE_HZ;
    s_pending.reset_rx = true;

    if (App_Event_SubscribeRxTx(on_rx_tx, NULL)) {
        LOG_ERROR("App_Event_SubscribeRxTx failed");
    }
    if (App_Event_SubscribeRxCfg(on_rx_cfg, NULL) != 0) {
        LOG_ERROR("App_Event_SubscribeRxCfg failed");
    }
}

void Sdr_WFM_Demod(const Sdr_IQ_Data *data, float *audio) {
    if (!data || !data->size || !audio || data->size != SDR_WFM_BLOCK_SIZE) {
        return;
    }
    apply_pending();

    Sdr_FmDisc_Process(&s_disc, data->i, data->q, s_disc_buf, data->size);

    /* 192 -> 96 -> 48 kHz; the decimation anti-alias filters also band-limit
     * the audio, so no separate audio LPF is needed. */
    Sdr_HalfbandDecim_Process(&s_dec1, s_disc_buf, s_dec1_buf,
                              SDR_WFM_BLOCK_SIZE);
    Sdr_HalfbandDecim_Process(&s_dec2, s_dec1_buf, audio, SDR_WFM_DEC1_SIZE);

    float k = (float)SDR_WFM_RATE / SDR_WFM_DEVIATION;
    for (size_t i = 0; i < SDR_WFM_AUDIO_SIZE; i++) {
        float v = first_order_iir(&de, audio[i]);
        v = Sdr_DC_Filter_Apply(&s_dc, v);
        audio[i] = v * k;
    }
}
