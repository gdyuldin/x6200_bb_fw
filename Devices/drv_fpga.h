#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board_config.h"
#include "drv_codec.h"

/*
 * FPGA host bus on GPIOE (PE0..PE7 = data, PE8 = frame latch, PE9 = byte
 * clock, PE10 = IRQ). Bit-banged, MCU is the bus master; DMA is not possible
 * because the bus is bidirectional on the same pins.
 *
 * Tier A (Drv_Fpga_Service) runs from the PE10 EXTI callback and must stay
 * short: no float, no allocation, no blocking. Every tick services RX I/Q or
 * TX I/Q (mutually exclusive) and then at most one deferred command. Control
 * functions only compute words and set a flag; the frame leaves on the next
 * PE10 tick, so latency is at most one bus tick (~20.8 us).
 *
 *  Drv_Fpga_Init(calib)       — GPIOE walk self-test + version read; copies the
 *                               per-unit FPGA calibration bytes (may be 0).
 *  Drv_Fpga_SetRxFrequency()  — compute the RX branch tuning set. The next
 *                               control frame carries it when the RX branch is
 *                               selected.
 *  Drv_Fpga_SetTxFrequency()  — compute the TX branch tuning set. The next
 *                               control frame carries it when the TX branch is
 *                               selected. The application supplies the final
 *                               frequency with split, RIT and XIT already
 *                               folded in.
 *  Drv_Fpga_TxMute()           — silence the FPGA TX output for the requested
 *                               time in milliseconds; the countdown runs on
 *                               every PE10 TX sample, including while the
 *                               staging ring is empty.
 *  Drv_Fpga_SetSampleRate()    — request an RX fftdec (0..3 -> M = 8/4/2/1).
 *                               The active rate is committed on the PE10 tick
 *                               that sends the matching control frame; until
 *                               then the driver keeps using the previous M.
 *                               While TX is active the request is only stored
 *                               and applied when the driver returns to RX.
 *  Drv_Fpga_SetTxActive()      — request the TX branch (true) or RX branch
 *                               (false). The actual branch changes on the PE10
 *                               tick whose control frame carries the request:
 *                               that tick still emits a sample for the previous
 *                               branch, and the new branch starts on the next
 *                               tick. A false -> true request also drops any
 *                               unread TX staging data (read pointer = write
 *                               pointer), so data from an earlier transmission
 *                               is never emitted.
 *  Drv_Fpga_RequestStatus()    — arm a one-shot 0x02 read-status on a later tick.
 *  Drv_Fpga_GetSamplesPerTick()— M for the actual branch: TX = 1 (48 kHz),
 *                               RX = 8 >> RX fftdec. For raw ring sizing.
 *  Drv_Fpga_GetFftdec()        — effective fftdec of the actual branch: TX = 3,
 *                               RX = committed RX fftdec.
 *  Drv_Fpga_GetRequestedFftdec()— last requested RX fftdec; may differ from
 *                               Drv_Fpga_GetFftdec() until the next PE10 tick
 *                               commits it (or until the return to RX while
 *                               transmitting).
 *  Drv_Fpga_TxWrite()          — publish Q1.31 TX I/Q into the staging ring;
 *                               returns the number of samples accepted.
 *  Drv_Fpga_TickCount()        — PE10 ticks since the last read (read-and-clear).
 *  Drv_Fpga_SampleCount()      — RX samples since the last read (read-and-clear).
 */

/* Bus opcodes. */
#define DRV_FPGA_OP_CONTROL      0x01u /* control/retune frame (25 bytes) */
#define DRV_FPGA_OP_READ_STATUS  0x02u /* read 1 flags byte + 3x 16-bit words */
#define DRV_FPGA_OP_TX_IQ        0x03u /* write one I/Q sample */
#define DRV_FPGA_OP_RX_IQ        0x04u /* read M I/Q samples */
#define DRV_FPGA_OP_RX_RESET     0x05u /* RX module reset, asserted */
#define DRV_FPGA_OP_RX_RESET_OFF 0x06u /* RX module reset, released */
#define DRV_FPGA_OP_READ_VERSION 0x08u /* read 3 firmware version bytes */
#define DRV_FPGA_OP_TX_RESET     0x09u /* TX module reset, asserted */
#define DRV_FPGA_OP_TX_RESET_OFF 0x0Au /* TX module reset, released */

/* Mode values. The bus runs unless the mode is idle. */
#define DRV_FPGA_MODE_ACTIVE     0x00u
#define DRV_FPGA_MODE_IDLE       0x0Bu

/* Sampling decimation: M = DRV_FPGA_SAMPLES_PER_TICK_BASE >> fftdec. */
#define DRV_FPGA_FFTDEC_MAX                  3u
#define DRV_FPGA_SAMPLES_PER_TICK_BASE       8u
#define DRV_FPGA_FFTDEC_DEFAULT              0u

/* FPGA NCO reference clock = the board RF ADC/DAC sampling rate (clock
 * generator OUT1). The NCO words, the Nyquist zone/band selection and the FFT
 * sample-rate divisors all derive from this value. */
#define DRV_FPGA_NCO_CLK_HZ BOARD_RF_SAMPLING_RATE_HZ

/* Bus tick rate: PE10 fires at the board audio rate (shared clock). */

/* DTCM raw window (the DSP window): the consumer reads it in its own I2S DMA
 * ISR and only needs the size. I and Q are kept in separate buffers, so the
 * deinterleave happens once, in the PE10 ISR. The window holds two worst-case
 * I2S blocks (M = DRV_FPGA_SAMPLES_PER_TICK_BASE per tick); power of two for
 * cheap wrap. */
#define DRV_FPGA_RAW_WINDOW_SAMPLES \
    (2u * DRV_CODEC_BLOCK_FRAMES * DRV_FPGA_SAMPLES_PER_TICK_BASE)

/*
 * I/Q sample format: the FPGA sends/receives 24-bit samples, left-justified
 * into the top 24 bits of an int32 -> Q1.31 (bit31 = sign, full CMSIS q31
 * range). 0x7FFFFF00 = +1.0, 0x80000000 = -1.0. The consumer works in Q1.31;
 * do not expect raw 24-bit values in the low bits. TX staging is written in
 * the same format and the driver emits bits 8..31 LSB-first on the bus.
 */
typedef struct {
    const int32_t *i;     /* I samples, Q1.31 */
    const int32_t *q;     /* Q samples, Q1.31 */
    uint32_t       count; /* complex samples in the window */
} DrvFpgaRawWindow;

/*
 * Per-unit FPGA calibration bytes, copied from the EEPROM calib region at init.
 * The block covers the FPGA sub-region of the calib area; the bracket values are
 * intra-region offsets and are part of the EEPROM region contract, so do not
 * reorder or resize the fields.
 *
 *   ctrl3_mode_byte   [0xb5]      CTRL3 mode selector used by the TX branch
 *   band_vals[21]     [0xb6..0xca] per-band calibration values
 *   reserved[3]       [0xcb..0xcd] gap, not used
 *   fftdec3..fftdec0  [0xce..0xd1] control-frame payload 9, selected by fftdec
 *                                  (0 -> fftdec0, 3 -> fftdec3)
 *   tx_cic_gain       [0xd2]       control-frame payload 10 (TX CIC gain)
 *   dac_gain          [0xd3]       control-frame payload 11 (DAC gain)
 *
 * Values are per-unit and must come from this board's EEPROM, never hard-coded.
 */
typedef struct __attribute__((packed)) {
    uint8_t ctrl3_mode_byte;
    uint8_t band_vals[21];
    uint8_t reserved[3];
    uint8_t fftdec3;
    uint8_t fftdec2;
    uint8_t fftdec1;
    uint8_t fftdec0;
    uint8_t tx_cic_gain;
    uint8_t dac_gain;
} DrvFpgaCalib;

_Static_assert(sizeof(DrvFpgaCalib) == 31, "DrvFpgaCalib layout");

/*
 * FPGA status, fetched with a one-shot 0x02 request (Drv_Fpga_RequestStatus).
 * The FPGA answers with a 7-byte block: one flags byte, then three 16-bit
 * big-endian words. Only bits 0..1 of the flags byte are driven by the FPGA
 * (the upper bits are stale). adc_min/adc_max are the extremes of the raw ADC
 * sample seen since the previous read (the FPGA clears them after handing them
 * out); adc_raw is the instantaneous sample. The OTR bits are only
 * trustworthy once the FPGA has settled, so the driver gates them behind a
 * warm-up count; the three words update on every read.
 */
typedef struct {
    bool    adc_otr; /* ADC over-range (flags bit0) */
    bool    dac_otr; /* DAC over-range (flags bit1) */
    int16_t adc_min; /* minimum raw ADC sample since the last read */
    int16_t adc_max; /* maximum raw ADC sample since the last read */
    int16_t adc_raw; /* instantaneous raw ADC sample */
} DrvFpgaStatus;

int      Drv_Fpga_Init(const DrvFpgaCalib *calib);
void     Drv_Fpga_SetRxFrequency(uint32_t freq_hz);
void     Drv_Fpga_SetTxFrequency(uint32_t freq_hz);
void     Drv_Fpga_TxMute(uint32_t ms);
/* 3 version bytes read from the FPGA in Drv_Fpga_Init (0,0,0 if it did not
 * answer). Copied into ver[0..2]. */
void     Drv_Fpga_GetVersion(uint8_t ver[3]);
void     Drv_Fpga_SetSampleRate(uint8_t fftdec);
uint8_t  Drv_Fpga_GetFftdec(void);
uint8_t  Drv_Fpga_GetRequestedFftdec(void);
void     Drv_Fpga_SetTxActive(int active);
void     Drv_Fpga_RequestStatus(void);
/* Copy the last 0x02 status read into out (all zero until the first read). out
 * must not be null. */
void     Drv_Fpga_GetStatus(DrvFpgaStatus *out);
void     Drv_Fpga_TxReset(void);
void     Drv_Fpga_RxReset(void);
void     Drv_Fpga_Service(void);
uint32_t Drv_Fpga_GetSamplesPerTick(void);
uint32_t Drv_Fpga_TickCount(void);
uint32_t Drv_Fpga_SampleCount(void);

/*
 * Extension points for the consumers. The raw (DSP) window keeps I and Q in
 * separate buffers, so the consumer works on contiguous I and Q arrays without
 * an extra deinterleave pass. All buffers are owned by the driver; the consumer
 * only reads and keeps its own offset. Drv_Fpga_GetPos() returns the raw write
 * index so the consumer can follow the ring independently.
 */
/* Returns the driver-owned raw window view. The struct itself is constant
 * (buffer pointers and count never change); only the buffer contents are
 * updated by the PE10 ISR. Do not modify the returned struct. */
const DrvFpgaRawWindow *Drv_Fpga_GetRawWindow(void);
uint32_t Drv_Fpga_GetPos(void);
/* Incremented when the raw ring contents become invalid (e.g. after fftdec
 * changes and the new control frame has been sent). Consumers compare their
 * saved value and resync to Drv_Fpga_GetPos() on a change. */
uint32_t Drv_Fpga_GetRawGeneration(void);

/*
 * TX staging producer. Copies up to count Q1.31 I/Q samples into the
 * driver-owned TX ring and returns the number actually accepted (0..count).
 * A return smaller than count means the ring was full. Samples are emitted in
 * I->Q order, one complex sample per PE10 tick while the TX branch is active;
 * an empty ring emits zeros. Safe to call from the I2S DMA ISR; the copy runs
 * outside the interrupt mask and only the write index is published under it.
 * Do not call from the PE10 ISR.
 */
uint32_t Drv_Fpga_TxWrite(const int32_t *i, const int32_t *q, uint32_t count);

/*
 * TX output gain in Q15 (0..32768, 32768 = unity), applied by the PE10 ISR when
 * it emits a staged sample. Written by the ALC from the ADC ISR; a 32-bit
 * aligned store/load is atomic on this core.
 */
void Drv_Fpga_SetTxGainQ15(int32_t gain);
