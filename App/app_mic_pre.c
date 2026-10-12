#include "app_mic_pre.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <arm_math.h>

#include "board_config.h"
#include "bsp_irq.h"

#define APP_MIC_PRE_CUT_COUNT       8u
#define APP_MIC_PRE_CUT_Q           100.0f
#define APP_MIC_PRE_CUT_DEPTH_DB    10.0f

static const float s_cut_centers_hz[APP_MIC_PRE_CUT_COUNT] = {
    750.0f, 900.0f, 1500.0f, 1800.0f,
    2250.0f, 2700.0f, 3000.0f, 3600.0f,
};

static arm_biquad_casd_df1_inst_f32 s_cut_inst;
static float s_cut_coeffs[APP_MIC_PRE_CUT_COUNT * 5];
static float s_cut_state[APP_MIC_PRE_CUT_COUNT * 4];

static volatile bool s_enabled;


static void cut_setup(void)
{
    /* Peaking RBJ with negative gain: A = 10^(-dB/40) puts the center gain
     * at exactly -dB when normalized by a0. */
    float a = powf(10.0f, -APP_MIC_PRE_CUT_DEPTH_DB / 40.0f);

    for (uint32_t i = 0; i < APP_MIC_PRE_CUT_COUNT; i++) {
        float w0 = 2.0f * (float)M_PI * s_cut_centers_hz[i] /
                   (float)BOARD_AUDIO_SAMPLE_RATE_HZ;
        float cw = cosf(w0);
        float alpha = sinf(w0) / (2.0f * APP_MIC_PRE_CUT_Q);
        float a0 = 1.0f + alpha / a;
        float *c = &s_cut_coeffs[i * 5];

        c[0] = (1.0f + alpha * a) / a0;
        c[1] = -2.0f * cw / a0;
        c[2] = (1.0f - alpha * a) / a0;
        c[3] = 2.0f * cw / a0;
        c[4] = -(1.0f - alpha / a) / a0;
    }

    arm_biquad_cascade_df1_init_f32(&s_cut_inst, APP_MIC_PRE_CUT_COUNT,
                                    s_cut_coeffs, s_cut_state);
}

static void reset_state(void)
{
    arm_fill_f32(0.0f, s_cut_state, APP_MIC_PRE_CUT_COUNT * 4);
}

void App_MicPre_Init(void)
{
    cut_setup();
    s_enabled = false;
    reset_state();
}

void App_MicPre_Reset(void)
{
    uint32_t key = Bsp_Irq_SaveLow();

    reset_state();

    Bsp_Irq_RestoreLow(key);
}

void App_MicPre_SetEnabled(bool enabled)
{
    uint32_t key = Bsp_Irq_SaveLow();

    if (enabled != s_enabled) {
        s_enabled = enabled;
        reset_state();
    }

    Bsp_Irq_RestoreLow(key);
}

void App_MicPre_Process(float *samples, uint32_t n)
{
    if (!s_enabled) {
        return;
    }
    arm_biquad_cascade_df1_f32(&s_cut_inst, samples, samples, n);
}
