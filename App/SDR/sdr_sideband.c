#include "sdr_sideband.h"

#include <string.h>

#include <arm_math.h>

#include "drv_codec.h"
#include "sdr_channel_filters.h"

/* Alternating complex FIR: even taps multiply the real input, odd taps the
 * imaginary input. The same history is reused for TX (audio in the i buffer);
 * RX and TX never run together and the history is cleared on a mode change. */
static struct {
    float32_t i[SDR_SSB_NUM_TAPS + DRV_CODEC_BLOCK_FRAMES - 1u];
    float32_t q[SDR_SSB_NUM_TAPS + DRV_CODEC_BLOCK_FRAMES - 1u];
} s_fir_hist __attribute__((aligned(32)));

/* Keep the most recent NUM_TAPS-1 samples at the front, then append. */
static void sideband_hist_shift(float32_t *hist, const float *x, uint32_t size)
{
    memmove(hist, hist + size, (SDR_SSB_NUM_TAPS - 1u) * sizeof(float32_t));
    arm_copy_f32(x, hist + (SDR_SSB_NUM_TAPS - 1u), size);
}

void Sdr_Sideband_Init(void)
{
    Sdr_Sideband_Reset();
}

void Sdr_Sideband_Reset(void)
{
    arm_fill_f32(0.0f, s_fir_hist.i, SDR_SSB_NUM_TAPS + DRV_CODEC_BLOCK_FRAMES - 1u);
    arm_fill_f32(0.0f, s_fir_hist.q, SDR_SSB_NUM_TAPS + DRV_CODEC_BLOCK_FRAMES - 1u);
}

/* Real part of the alternating complex FIR; only one multiply per tap. */
void Sdr_Sideband_Demod(const Sdr_IQ_Data *in, float *audio, bool lower)
{
    const uint32_t base = SDR_SSB_NUM_TAPS - 1u;
    const uint32_t size = in->size;

    sideband_hist_shift(s_fir_hist.i, in->i, size);
    sideband_hist_shift(s_fir_hist.q, in->q, size);

    for (uint32_t n = 0; n < size; ++n) {
        float acc = 0.0f;
        float qacc = 0.0f;

        for (uint32_t m = 0; m < SDR_SSB_EVEN_TAPS; ++m) {
            acc += sdr_ssb_coeffs_re[m] * s_fir_hist.i[base + n - 2u * m];
        }
        for (uint32_t m = 0; m < SDR_SSB_ODD_TAPS; ++m) {
            qacc += sdr_ssb_coeffs_im[m]
                    * s_fir_hist.q[base + n - 2u * m - 1u];
        }
        audio[n] = lower ? (acc + qacc) : (acc - qacc);
    }
}

/* Analytic TX signal from real audio; only one multiply per tap. */
void Sdr_Sideband_Mod(const float *audio, Sdr_IQ_Data *out, bool lower)
{
    const uint32_t base = SDR_SSB_NUM_TAPS - 1u;
    const uint32_t size = out->size;

    sideband_hist_shift(s_fir_hist.i, audio, size);

    for (uint32_t n = 0; n < size; ++n) {
        float i_acc = 0.0f;
        float q_acc = 0.0f;

        for (uint32_t m = 0; m < SDR_SSB_EVEN_TAPS; ++m) {
            i_acc += sdr_ssb_coeffs_re[m] * s_fir_hist.i[base + n - 2u * m];
        }
        for (uint32_t m = 0; m < SDR_SSB_ODD_TAPS; ++m) {
            q_acc += sdr_ssb_coeffs_im[m]
                     * s_fir_hist.i[base + n - 2u * m - 1u];
        }
        out->i[n] = i_acc;
        out->q[n] = lower ? -q_acc : q_acc;
    }
}
