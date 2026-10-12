#pragma once

#include <stdbool.h>

#include "sdr_common.h"

/* Spectrum path configuration. The FFT size is also the telemetry bin count. */
#define SDR_SPECTRUM_FFT_SIZE   512u
#define SDR_SPECTRUM_PERIOD_MS  40u

/* Maximum number of decimation-by-2 stages the spectrum zoom cascade can run;
 * the zoom exponent (host fftzoomcw) is clamped to this. */
#define SDR_SPECTRUM_ZOOM_MAX_STAGES 4u

/* Noise-floor estimator (an AGC reference, independent of the demodulator).
 * The minimum 32-bin power window inside the central 5%..95% span is taken as
 * the noise reference and scaled to the demodulation passband. */
#define SDR_SPECTRUM_NOISE_WINDOW_BINS 32u
#define SDR_SPECTRUM_NOISE_EDGE_BINS   26u /* ~5% of SDR_SPECTRUM_FFT_SIZE */
#define SDR_SPECTRUM_NOISE_AVG_TAU_S   1.0f
#define SDR_SPECTRUM_NOISE_SKIP_FRAMES 1u
/* Frames to skip after a retune: the front end and passband change, so the
 * first spectrum frames are transitional. ~8 frames * 40 ms = 320 ms. */
#define SDR_SPECTRUM_NOISE_RETUNE_FRAMES 8u

void Sdr_Spectrum_Init(void);

/* Runtime configuration pushed by App_RadioState. Each setter is independent
 * so the owner can push the current configuration without a combined
 * transaction. The filter pair is the RX passband in Hz (low = HPF edge,
 * high = LPF edge). */
void Sdr_Spectrum_SetModulation(SdrModulation modulation);
void Sdr_Spectrum_SetFftdec(uint8_t fftdec);
/* Spectrum zoom exponent (host fftzoomcw). The display runs on data decimated by
 * 2^n; n is clamped to SDR_SPECTRUM_ZOOM_MAX_STAGES. Applied only in RX for the
 * CW/CWR modulations; ignored in TX and other modulations. */
void Sdr_Spectrum_SetFftzoomcw(uint8_t n);
void Sdr_Spectrum_SetFilter(int32_t low_hz, int32_t high_hz);

/* Freeze the noise estimate for a short window after a retune, while the front
 * end and the passband settle. Call from the state-apply path (main loop). */
void Sdr_Spectrum_NotifyRetune(void);

/* Called from the I2S DMA callback after the TX block is filled. Copies new
 * raw I/Q from the FPGA ring into the spectrum accumulation buffer. In TX mode
 * it returns immediately; the TX signal is fed by Sdr_Spectrum_AppendTx. */
void Sdr_Spectrum_Process(void);

/* Select TX spectrum mode. In TX the display shows the generated TX signal and
 * uses the TX dB offset; the raw ring is not read. In RX the next Process call
 * resyncs to the current raw write position. */
void Sdr_Spectrum_SetTxMode(bool tx);

/* Append count float TX I/Q samples (interleaved internally) to the spectrum
 * accumulation buffer. Call from the I2S DMA ISR only while TX mode is active;
 * honors the capture pause and limits count to the free buffer space. */
void Sdr_Spectrum_AppendTx(const float *i, const float *q, uint32_t count);

const float * Sdr_Spectrum_Get_Db(void);

const float Sdr_Spectrum_Get_S_meter(void);

/* Latest noise-floor estimate in the demodulation passband (10*log10 of power,
 * no display offset) and its validity. Valid only in RX while the estimator
 * runs; false in TX and right after a generation change (first frame skipped).
 * Written in the spectrum task context, read from the AGC in the I2S ISR. */
float Sdr_Spectrum_Get_Noise_Db(void);
float Sdr_Spectrum_Get_Noise_Lin(void);
bool Sdr_Spectrum_Is_Noise_Valid(void);

/* Noise estimate mapped into the demodulated-audio (peak, 20log) domain for the
 * active modulation: Get_Noise_Db() plus the per-group calibration offset. The
 * AGC and the squelch demods consume this single source; the spectrum maps its
 * own modulation onto a group, so no modulation is passed in. Read from the I2S
 * ISR with the same race tolerance as the raw estimate above. */
float Sdr_Spectrum_Get_Demod_Noise_Est_Db(void);
