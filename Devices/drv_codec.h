#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * TLV320AIC3204 audio codec on I2C1 (7-bit 0x18), full-duplex SPI3/I2S3 link.
 * The MCU is the I2S master (32-bit Philips, 48 kHz, BCLK/WCLK outputs), the
 * codec is a slave with PLL bypassed on the 12.288 MHz MCLK; the register
 * contract is the ordered A-G init sequence and is not reconfigured here.
 *
 * The link is always full duplex. Each direction runs a circular DMA with two
 * halves in .dma_d2; one half is DRV_CODEC_DMA_HALF_WORDS int32 words and is
 * the unit of block processing: DRV_CODEC_BLOCK_FRAMES stereo frames of
 * interleaved L,R samples (512 bytes, 1.33 ms at 48 kHz).
 */

#define DRV_CODEC_ADDR7            0x18u
#define DRV_CODEC_XFER_TIMEOUT_MS  100u
#define DRV_CODEC_DMA_HALF_WORDS   128u
/* Stereo L,R int32 samples: two words per frame. */
#define DRV_CODEC_BLOCK_FRAMES     (DRV_CODEC_DMA_HALF_WORDS / 2u)
#define DRV_CODEC_MICPGA_GAIN_MAX  0x5Fu

/*
 * Output driver gain (P1_R16..R19): D5:0 two's complement, -6..+29 dB, 1 dB
 * step. `on == false` sets the D6 mute bit; a muted driver cannot also be at
 * -6 dB (SLAA557), so the driver uses -5 dB in that case.
 */
#define DRV_CODEC_OUT_GAIN_MIN     (-6)
#define DRV_CODEC_OUT_GAIN_MAX     29

/*
 * Block processing hook. Runs in I2S3 DMA half/full interrupt context: keep
 * it short, no blocking calls. rx points at the received half of the DMA
 * ring, tx at the half the callback must fill before returning (write zeros
 * for silence; the driver never touches the samples). frames is always
 * DRV_CODEC_BLOCK_FRAMES. May be registered or swapped at runtime.
 */
typedef void (*DrvCodecBlockCb)(const int32_t *rx, int32_t *tx,
                                unsigned frames, void *ctx);

typedef enum {
    DRV_CODEC_IN_L_NONE,
    DRV_CODEC_IN_L_HMIC, /* hand mic:     IN1_L, single-ended */
    DRV_CODEC_IN_L_IMIC, /* internal mic: IN2_L/IN2_R, differential */
    DRV_CODEC_IN_L_USB, /* line-in L:(USB)    IN3_L, single-ended */
} DrvCodecInputL;

typedef enum {
    DRV_CODEC_IN_R_NONE,
    DRV_CODEC_IN_R_HOST, /* line-in R:(host)    IN3_R, single-ended */
} DrvCodecInputR;

/* HPR output source: the codec can take it from either DAC. */
typedef enum {
    DRV_CODEC_HPR_SRC_DACL_N = 0, /* left DAC, inverted */
    DRV_CODEC_HPR_SRC_DACR_P,     /* right DAC, non-inverted */
} DrvCodecHprSource;


/* Reset PB10, program the register sequence over I2C1. Negative on error. */
int Drv_Codec_Init(void);
/* cb == 0 clears the hook. */
int Drv_Codec_SetBlockCallback(DrvCodecBlockCb cb, void *ctx);
/* Full-duplex DMA on (TX silent until a callback fills it). */
int Drv_Codec_Start(void);
int Drv_Codec_Stop(void);

/* Remaining TX DMA I2S3 frames until the next reload, in stereo frames.
 * 0 when the codec is not running. */
uint32_t Drv_Codec_TxDmaRemaining(void);

int Drv_Codec_SetInputL(DrvCodecInputL input);
int Drv_Codec_SetInputR(DrvCodecInputR input);

int Drv_Codec_SetInGainL(uint8_t val);
int Drv_Codec_SetInGainR(uint8_t val);


// HOST out
int Drv_Codec_SetHPLGain(bool on, int8_t level);

// SPEAKER/phones
int Drv_Codec_SetHPRGain(bool on, int8_t level);
int Drv_Codec_SetHprSource(DrvCodecHprSource source);

// Unused
int Drv_Codec_SetLOLGain(bool on, int8_t level);

// USB out
int Drv_Codec_SetLORGain(bool on, int8_t level);
