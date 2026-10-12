#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdr_common.h"
#include "sdr_config.h"

/* RX DSP orchestration.  Mirrors sdr_tx: the module owns the raw-I/Q ring
 * reader, the RX audio plans and the demodulation chain.  The radio layer only
 * pushes the committed configuration and scales the resulting mono block into
 * the stereo DAC output.
 *
 * The module is the single publisher of SdrRxCfg: it decides whether a new
 * snapshot can be applied immediately or has to wait for the FPGA raw
 * generation to change, and it emits the block in step with the demodulator.
 */

#define SDR_RX_RATE_12K  (12000u)
#define SDR_RX_RATE_24K  (24000u)
#define SDR_RX_RATE_48K  (48000u)
#define SDR_RX_RATE_192K (192000u)

/* Initialise the module and apply the default resample configuration for the
 * requested FPGA fftdec.  Call after Sdr_Resample_Init() and the demodulator
 * initialisers. */
void Sdr_Rx_Init(void);

/*
 * Push the committed RX configuration.  When the fftdec matches the running
 * generation the configuration is applied at once and SdrRxCfg is published;
 * otherwise it is staged and published when the FPGA raw generation commits.
 * Cheap when nothing changed.  Call from task context.
 */
void Sdr_Rx_SetConfig(const SdrRxCfg *cfg);

/* Lower fftdec to the largest value the mode still supports: the FPGA source
 * rate must not fall below the mode's rate plan. Never raises it. Returns false
 * when the mode has no plan or no fftdec is usable. */
bool Sdr_Rx_ClampFftdec(SdrModulation mode, uint8_t *fftdec);

/* Process one RX block.  audio48 must hold DRV_CODEC_BLOCK_FRAMES mono samples
 * at 48 kHz.  The buffer is always filled: zeros when there is no complete raw
 * I/Q block or the mode has no demodulator.  Returns true when real audio was
 * produced.  Safe from the I2S DMA ISR: no logs, divisions or blocking. */
bool Sdr_Rx_Process(float *audio48, uint32_t n);

/* RX signal-strength level for telemetry, in dB (more negative = weaker).
 * SSB/CW read the demodulator's in-band power; AM/FM use the spectrum S-meter.
 * Holds the last value while transmitting.  Call from task context. */
float Sdr_Rx_Get_Level_Db(void);
