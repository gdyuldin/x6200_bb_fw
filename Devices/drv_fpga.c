#include "drv_fpga.h"

#include <stddef.h>
#include <string.h>

#include "bsp_exti.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "main.h"

#define LOG_TAG "drv.fpga"

/*
 * FPGA host bus on GPIOE: PE0..PE7 carry the byte, PE8 is the frame latch and
 * PE9 the byte clock; PE10 raises the per-tick interrupt. Direction changes on
 * the data lines rewrite the MODER fields, so the masks below are built from the
 * CubeMX pin labels instead of hard-coded register literals.
 */

/* Bit mask of the eight data lines. */
#define DRV_FPGA_DATA_MASK                                                   \
    ((uint32_t)(FPGA_DATA_0_Pin | FPGA_DATA_1_Pin | FPGA_DATA_2_Pin |        \
                FPGA_DATA_3_Pin | FPGA_DATA_4_Pin | FPGA_DATA_5_Pin |        \
                FPGA_DATA_6_Pin | FPGA_DATA_7_Pin))
#define DRV_FPGA_DATA_BITS 8u

/* The bus primitives read the byte straight out of IDR and write BSRR without
 * masking, so the data lines must be the low eight pins of GPIOE. */
_Static_assert(DRV_FPGA_DATA_MASK == ((1u << DRV_FPGA_DATA_BITS) - 1u),
               "FPGA data pins must be PE0..PE7");

/* Two I2S blocks of headroom for the TX staging ring. Kept small on purpose:
 * the ALC feed-forward aligns the detector measurement with the drive, and the
 * staging latency adds directly to that delay. */
#define DRV_FPGA_TX_STAGING_SAMPLES (2u * DRV_CODEC_BLOCK_FRAMES)

/* Re-send the control frame so the FPGA NCO stays alive even when nothing else
 * changes. Counted in PE10 ticks. */
#define DRV_FPGA_KEEPALIVE_TICKS BOARD_AUDIO_SAMPLE_RATE_HZ

_Static_assert((DRV_FPGA_RAW_WINDOW_SAMPLES &
                (DRV_FPGA_RAW_WINDOW_SAMPLES - 1u)) == 0u,
               "raw window must stay a power of two for cheap wrap");

#define DRV_FPGA_CTRL_CTRL2_FFTDEC_SHIFT 6u

/*
 * CTRL1 (payload[0]) bits. bit0 tells the FPGA to stream RX I/Q; bit2 switches
 * it to accept TX I/Q from the MCU. bit1 (RX2) stays 0: this board has a single
 * antenna and duplex is not possible, so the RX bit is dropped while TX is
 * active. The upper bits are the ADC front-end controls: the driver and PGA are
 * always enabled, dither and randomization are optional (see fpga_ctrl1_build).
 */
#define DRV_FPGA_CTRL1_RX_ACTIVE  0x01u
#define DRV_FPGA_CTRL1_TX_ACTIVE  0x04u
#define DRV_FPGA_CTRL1_ADC_DITH   0x08u
#define DRV_FPGA_CTRL1_ADC_RAND   0x20u
#define DRV_FPGA_CTRL1_ADC_PGA    0x40u
#define DRV_FPGA_CTRL1_ADC_DRIVER 0x80u

/*
 * CTRL2 (payload idx 18) bit map. bit1 is the RX/idle flag (set when not
 * transmitting), bit2/bit3 the Nyquist-zone Gray code, bit4 a constant 1 and
 * bit5 the "tuned into a known band" flag; bits6-7 carry fftdec. bit0 stays 0.
 */
#define DRV_FPGA_CTRL2_RX_ACTIVE 0x02u /* bit1 */
#define DRV_FPGA_CTRL2_ZONE_A    0x04u /* bit2 */
#define DRV_FPGA_CTRL2_ZONE_B    0x08u /* bit3 */
#define DRV_FPGA_CTRL2_CONST     0x10u /* bit4, always 1 */
#define DRV_FPGA_CTRL2_BAND      0x20u /* bit5 */

/*
 * CTRL3 (payload idx 23): the DAC driver control bits. Both are set in RX/idle
 * (driver shut down); in TX the band/mode code selects the combination.
 */
#define DRV_FPGA_CTRL3_DAC_DRV_A0 0x01u
#define DRV_FPGA_CTRL3_DAC_DRV_A1 0x02u
#define DRV_FPGA_CTRL3_RX_IDLE \
    (DRV_FPGA_CTRL3_DAC_DRV_A0 | DRV_FPGA_CTRL3_DAC_DRV_A1)

/* Frequency folding/tuning boundaries derived from the RF sampling rate. */
#define DRV_FPGA_FS_HALF_HZ     (DRV_FPGA_NCO_CLK_HZ / 2u)
#define DRV_FPGA_FS_2_HZ        (DRV_FPGA_NCO_CLK_HZ * 2u)
#define DRV_FPGA_FS_6_HZ        (DRV_FPGA_NCO_CLK_HZ * 6u)
#define DRV_FPGA_ZONE_STEP_HIGH (DRV_FPGA_NCO_CLK_HZ * 9u / 2u)

/* read-status warm-up: flags are ignored until this many 0x02 calls. */
#define DRV_FPGA_STATUS_WARMUP 50u

/*
 * TX/stream warm-up: on each of the first N PE10 ticks after init the service
 * sends opcode 0x03 with one zero I/Q sample, before the normal TX/RX branch.
 * This brings the FPGA RX path up; RX is still serviced on the same ticks.
 */
#define DRV_FPGA_TX_WARMUP 100u

/* Control frame 0x01: 25 bytes (1 opcode + 24 payload), emitted verbatim on the
 * bus (byte 0 latched by the strobe, bytes 1..24 clocked). Positional layout:
 * control byte, RX1 NCO (big-endian), RX2 NCO, RX CIC gain, TX CIC gain, DAC
 * gain, ADC offset, TCXO divider, VCXO divider, control byte, TX NCO
 * (big-endian), control byte. The three gain bytes are per-unit values from the
 * EEPROM calib region (copied in by Drv_Fpga_Init); the RX CIC gain is selected
 * by fftdec (higher fftdec -> lower array index). The offset and divider fields
 * are fixed for this board. The control bytes carry the RX/TX-active bits, the
 * RX flag + Nyquist-zone + band + fftdec, and the RX/idle vs band class (see the
 * builders). The multi-byte words are big-endian and kept as byte arrays so the
 * struct matches the wire layout byte-for-byte; they are filled with
 * fpga_put_be32. */
typedef struct __attribute__((packed)) {
    uint8_t opcode;
    uint8_t ctrl1;
    uint8_t rx1_nco[4];
    uint8_t rx2_nco[4];
    uint8_t rx_cic_gain;
    uint8_t tx_cic_gain;
    uint8_t dac_gain;
    uint8_t adc_offset[2];
    uint8_t tcxo_div[2];
    uint8_t vcxo_div[2];
    uint8_t ctrl2;
    uint8_t tx_nco[4];
    uint8_t ctrl3;
} DrvFpgaCtrlFrame;

_Static_assert(sizeof(DrvFpgaCtrlFrame) == 25, "DrvFpgaCtrlFrame layout");
_Static_assert(offsetof(DrvFpgaCtrlFrame, rx1_nco) == 2, "ctrl frame offset");
_Static_assert(offsetof(DrvFpgaCtrlFrame, rx2_nco) == 6, "ctrl frame offset");
_Static_assert(offsetof(DrvFpgaCtrlFrame, rx_cic_gain) == 10,
               "ctrl frame offset");
_Static_assert(offsetof(DrvFpgaCtrlFrame, ctrl2) == 19, "ctrl frame offset");
_Static_assert(offsetof(DrvFpgaCtrlFrame, tx_nco) == 20, "ctrl frame offset");

/* One complete precomputed tuning set for a branch frequency. Two are kept:
 * RX and TX. The application supplies the final frequency (RIT/XIT and split
 * already applied), so switching the branch only selects a set; no 64-bit
 * tuning math runs in the PE10 ISR or in the SetTxActive setter. */
typedef struct {
    uint32_t freq_hz;   /* frequency this set was computed from */
    uint32_t nco;       /* NCO phase-increment word for the folded frequency */
    uint32_t aux;       /* prescaled AUX tune word */
    uint8_t  zone_gray; /* CTRL2 zone, Gray-encoded into bit2/bit3 */
    uint8_t  band_idx;  /* band-table index */
    bool     inv;       /* odd-Nyquist-zone spectral inversion */
} DrvFpgaTuning;

/* --- file data --------------------------------------------------------- */

static const uint16_t s_data_pins[DRV_FPGA_DATA_BITS] = {
    FPGA_DATA_0_Pin, FPGA_DATA_1_Pin, FPGA_DATA_2_Pin, FPGA_DATA_3_Pin,
    FPGA_DATA_4_Pin, FPGA_DATA_5_Pin, FPGA_DATA_6_Pin, FPGA_DATA_7_Pin,
};

/*
 * Band-table upper edges (Hz) for the 20-band classifier. Bands 0..18 span
 * 0..8 GHz; band 19 (> 8 GHz) is unreachable with a 32-bit frequency, so the
 * table needs only 18 edges.
 */
static const uint32_t s_band_edges_hz[18] = {
    1000000u,     /*  0: 1.0 MHz   */
    2500000u,     /*  1: 2.5 MHz   */
    4000000u,     /*  2: 4.0 MHz   */
    6000000u,     /*  3: 6.0 MHz   */
    8500000u,     /*  4: 8.5 MHz   */
    12000000u,    /*  5: 12.0 MHz  */
    16000000u,    /*  6: 16.0 MHz  */
    19500000u,    /*  7: 19.5 MHz  */
    22500000u,    /*  8: 22.5 MHz  */
    26500000u,    /*  9: 26.5 MHz  */
    28000000u,    /* 10: 28.0 MHz  */
    40000000u,    /* 11: 40.0 MHz  */
    60000000u,    /* 12: 60.0 MHz  */
    110000000u,   /* 13: 110.0 MHz */
    300000000u,   /* 14: 300.0 MHz */
    1000000000u,  /* 15: 1.0 GHz   */
    2000000000u,  /* 16: 2.0 GHz   */
    4000000000u,  /* 17: 4.0 GHz   */
};

/* MODER fields for the data lines: two bits per pin. Built at init from the
 * pin labels so no register literal is hard-coded. */
static struct {
    uint32_t moder_mask;
    uint32_t moder_out;
} s_bus;

/* Mode, committed/requested branch and committed/requested rate. The requested
 * values are written by the setters and committed in fpga_service_deferred()
 * after the matching control frame has left the bus. */
static struct {
    uint8_t  mode;
    /* tx_active is the branch the tick service actually uses; the application
     * only requests a branch through tx_active_requested. */
    volatile bool tx_active;
    bool     tx_active_requested;
    uint8_t  fftdec;
    uint32_t m_samples;
    /* Requested RX fftdec: committed to the active fftdec/m_samples only after
     * the matching control frame has been sent on a PE10 tick. Until then the
     * driver keeps reading the old rate. While TX is active the request stays
     * pending and is applied on the return to RX. */
    uint8_t  fftdec_requested;
} s_state;

/* Read-and-clear tick and RX-sample counters. */
static struct {
    volatile uint32_t ticks;
    volatile uint32_t samples;
} s_count;

/* Deferred command flags, consumed one per PE10 tick by fpga_service_deferred. */
static struct {
    bool update_pending;
    bool status_request;
    bool tx_reset;
    bool rx_reset;
    bool tx_reset_step;
    bool rx_reset_step;
} s_cmd;

/* Branch tuning sets and the control-frame keepalive counter. */
static struct {
    DrvFpgaTuning rx;
    DrvFpgaTuning tx;
    uint32_t      keepalive_ticks;
} s_tune;

/* TX mute countdown in PE10 ticks. While non-zero the TX service emits zeros;
 * the counter ticks on every TX sample, including when the staging ring is
 * empty. Written by Drv_Fpga_TxMute(), decremented in the PE10 ISR. */
static volatile uint32_t s_tx_mute_ticks;

/* FPGA identity read in Drv_Fpga_Init: firmware version bytes and the per-unit
 * calibration bytes copied from the EEPROM calib region. The calib is kept by
 * value so the driver owns its frame data. */
static struct {
    uint8_t      version[3];
    DrvFpgaCalib calib;
} s_ident;

/* Status-read warm-up and the last 0x02 status read. */
static struct {
    uint32_t      warmup;
    uint32_t      tx_warmup;
    DrvFpgaStatus status;
} s_stat;

/* DTCM raw window (DSP window): I and Q in separate buffers, Q1.31. Default
 * .bss lands in DTCMRAM per the linker script, so these stay in DTCM. The
 * deinterleave happens in the PE10 ISR, so the consumer reads contiguous I and
 * Q without an extra pass. Generation-only invalidation, no large memset under
 * IRQ lock: incremented after the new sample-rate control frame has actually
 * been sent. */
static struct {
    int32_t           i[DRV_FPGA_RAW_WINDOW_SAMPLES];
    int32_t           q[DRV_FPGA_RAW_WINDOW_SAMPLES];
    volatile uint32_t wr;
    volatile uint32_t generation;
    volatile bool     invalidate_pending;
} s_raw;

/* TX staging: I and Q in separate buffers (Q1.31), filled by the TX chain
 * outside Tier A; the ISR only reads and emits them in I->Q order. Read and
 * write indices are kept separate: rd advances in the PE10 ISR, wr in
 * Drv_Fpga_TxWrite(). The ring is never filled completely, so rd == wr means
 * "no data" and an empty ring emits zeros. */
static struct {
    int32_t           i[DRV_FPGA_TX_STAGING_SAMPLES];
    int32_t           q[DRV_FPGA_TX_STAGING_SAMPLES];
    volatile uint32_t rd;
    volatile uint32_t wr;
} s_tx;

/* TX output gain, Q15. Written by the ALC, read once per emitted sample. */
static volatile int32_t s_tx_gain_q15 = 32768;

/* --- low level bus primitives (no register literals) -------------------- */

static uint32_t fpga_pin_bit(uint16_t pin) {
    uint32_t bit = 0u;
    while ((pin & 1u) == 0u && bit < 16u) {
        pin >>= 1u;
        bit++;
    }
    return bit;
}

static void fpga_build_pin_masks(void) {
    s_bus.moder_mask = 0u;
    s_bus.moder_out  = 0u;
    for (uint32_t i = 0; i < DRV_FPGA_DATA_BITS; ++i) {
        uint32_t bit = fpga_pin_bit(s_data_pins[i]);
        s_bus.moder_mask |= 0x3u << (2u * bit);
        s_bus.moder_out |= 0x1u << (2u * bit);
    }
}

static void fpga_bus_moder_out(void) {
    GPIOE->MODER = (GPIOE->MODER & ~s_bus.moder_mask) | s_bus.moder_out;
}

static void fpga_bus_moder_in(void) {
    GPIOE->MODER &= ~s_bus.moder_mask;
}

static void fpga_bus_write_byte(uint8_t value) {
    /* BSRR: set the 1-bits of the byte and reset the 0-bits. Built from the
     * data mask, not a literal. */
    GPIOE->BSRR = (uint32_t)value |
                  (((uint32_t)~value & DRV_FPGA_DATA_MASK) << 16u);
}

static void fpga_bus_byte_clock(void) {
    GPIOE->BSRR = (uint32_t)FPGA_STROBE_1_Pin;
    GPIOE->BSRR = (uint32_t)FPGA_STROBE_1_Pin << 16u;
}

/* The byte is sampled inside the PE9-high window: raise PE9, sample IDR, drop
 * PE9. The FPGA drives the data lines only while PE9 is high; reading after the
 * trailing edge samples the released (tri-state) bus. A byte-wide read of IDR
 * returns the data lines directly (low pin byte, see the mask assert). */
static uint8_t fpga_bus_read_byte_clocked(void) {
    GPIOE->BSRR = (uint32_t)FPGA_STROBE_1_Pin;
    uint8_t val = *(volatile uint8_t *)&GPIOE->IDR;
    GPIOE->BSRR = (uint32_t)FPGA_STROBE_1_Pin << 16u;
    return val;
}

static void fpga_bus_strobe(void) {
    GPIOE->BSRR = (uint32_t)FPGA_STROBE_0_Pin;
    GPIOE->BSRR = (uint32_t)FPGA_STROBE_1_Pin;
    GPIOE->BSRR =
        ((uint32_t)FPGA_STROBE_0_Pin | (uint32_t)FPGA_STROBE_1_Pin) << 16u;
}

static void fpga_bus_send_opcode(uint8_t opcode) {
    fpga_bus_moder_out();
    fpga_bus_write_byte(opcode);
    fpga_bus_strobe();
}

/* --- I/Q transfer ------------------------------------------------------- */

/*
 * 24-bit bus sample, left-justified into the top 24 bits of an int32 -> Q1.31
 * (bit31 = sign). The bus carries the sample MSB-first: read shifts the first
 * byte up to bits 24..31 and each following byte down by 8, so the stored value
 * is B0<<24 | B1<<16 | B2<<8. Write is the exact inverse and emits the top
 * three bytes (bits 24..31, 16..23, 8..15) in that order. The value is
 * left-justified, so no sign extension is needed.
 */
static int32_t fpga_read_iq_word(void) {
    uint32_t raw = 0u;
    for (uint32_t b = 0; b < 3u; ++b) {
        uint8_t val = fpga_bus_read_byte_clocked();
        raw |= (uint32_t)val << (24u - 8u*b);
    }
    return (int32_t)raw;
}

static void fpga_bus_write_word(int32_t value) {
    uint32_t raw = (uint32_t)value;
    for (uint32_t b = 0; b < 3u; ++b) {
        fpga_bus_write_byte((uint8_t)(raw >> (24u - 8u*b)));
        fpga_bus_byte_clock();
    }
}

/* Store one sample into the raw ring at the caller's cursor. The cursor is
 * local to the tick so the volatile write position is published once. */
static void fpga_store_raw(uint32_t *wr, int32_t i, int32_t q) {
    s_raw.i[*wr] = i;
    s_raw.q[*wr] = q;
    if (++*wr >= DRV_FPGA_RAW_WINDOW_SAMPLES) {
        *wr = 0u;
    }
}

/* --- deferred commands -------------------------------------------------- */

static void fpga_put_be32(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value >> 24u);
    dst[1] = (uint8_t)(value >> 16u);
    dst[2] = (uint8_t)(value >> 8u);
    dst[3] = (uint8_t)value;
}

/* payload 9 calibration byte for the fftdec carried by the outgoing frame
 * (the value that is about to be committed, not the still-active one). */
static uint8_t fpga_calib_rx_cic_gain(uint8_t fftdec) {
    switch (fftdec) {
        case 0u: return s_ident.calib.fftdec0;
        case 1u: return s_ident.calib.fftdec1;
        case 2u: return s_ident.calib.fftdec2;
        case 3u: return s_ident.calib.fftdec3;
        default: return 0u;
    }
}

static uint8_t fpga_band_index(uint32_t freq_hz) {
    for (uint8_t i = 0; i < 18u; ++i) {
        if (freq_hz < s_band_edges_hz[i]) {
            return i;
        }
    }
    return 18u; /* >= 4.0 GHz: last reachable band */
}

/*
 * Triangular Nyquist fold of f into [0, fs/2]. Returns the folded frequency
 * and, when inverted != 0, the zone parity (1 = odd zone = spectral inversion).
 * Shared by the NCO words (raw frequency) and the AUX word (prescaled).
 */
static uint32_t fpga_fold(uint32_t f, uint8_t *inverted) {
    uint32_t zone = f / DRV_FPGA_FS_HALF_HZ;
    uint32_t rem  = f - zone * DRV_FPGA_FS_HALF_HZ;

    if (inverted != 0) {
        *inverted = (uint8_t)(zone & 1u);
    }
    if ((zone & 1u) != 0u) {
        return DRV_FPGA_FS_HALF_HZ - rem;
    }
    return rem;
}

/* Phase-increment word for a folded frequency: round(f * 2^32 / fs). */
static uint32_t fpga_nco_word(uint32_t folded_hz) {
    uint64_t scaled = ((uint64_t)folded_hz << 32u) + DRV_FPGA_FS_HALF_HZ;

    return (uint32_t)(scaled / DRV_FPGA_NCO_CLK_HZ);
}

/* Prescaler for the AUX tune word (1/3/6 selected by frequency). */
static uint32_t fpga_aux_divider(uint32_t freq_hz) {
    if (freq_hz <= DRV_FPGA_FS_2_HZ) {
        return 1u;
    }
    if (freq_hz <= DRV_FPGA_FS_6_HZ) {
        return 3u;
    }
    return 6u;
}

static uint32_t fpga_aux_word(uint32_t freq_hz) {
    uint32_t prescaled = freq_hz / fpga_aux_divider(freq_hz);

    return fpga_nco_word(fpga_fold(prescaled, 0));
}

/*
 * CTRL2 Nyquist-zone bits, Gray-coded into bit2/bit3. Below 2*fs the zone steps
 * by fs/2; above it by 4.5*fs (the prescaler's zone scale).
 */
static uint8_t fpga_ctrl2_zone(uint32_t freq_hz) {
    uint32_t zone;

    if (freq_hz <= DRV_FPGA_FS_2_HZ) {
        zone = freq_hz / DRV_FPGA_FS_HALF_HZ;
    } else {
        zone = freq_hz / DRV_FPGA_ZONE_STEP_HIGH;
    }
    zone &= 3u;
    switch (zone) {
        case 0u: return 0u;
        case 1u: return DRV_FPGA_CTRL2_ZONE_A;
        case 2u: return DRV_FPGA_CTRL2_ZONE_A | DRV_FPGA_CTRL2_ZONE_B;
        default: return DRV_FPGA_CTRL2_ZONE_B;
    }
}

/* CTRL3 TX branch: mode byte 0/1/2 pick a DAC driver combination, anything
 * else falls back to the band calibration value (0 -> the RX/idle code, else
 * the driver off). */
static uint8_t fpga_ctrl3_tx(uint8_t mode_byte, uint8_t band_calib) {
    switch (mode_byte) {
        case 0u: return DRV_FPGA_CTRL3_DAC_DRV_A1;
        case 1u: return DRV_FPGA_CTRL3_DAC_DRV_A0;
        case 2u: return 0x00u;
        default: return (band_calib == 0u) ? DRV_FPGA_CTRL3_RX_IDLE : 0x00u;
    }
}

/* Tuning set the next control frame must carry. The application already folded
 * split and RIT/XIT into the RX and TX frequencies, so the branch flag alone
 * selects the set. */
static const DrvFpgaTuning *fpga_branch_tuning(void) {
    return s_state.tx_active_requested ? &s_tune.tx : &s_tune.rx;
}

/* CTRL2 from the branch flags and the precomputed range bits. bit1 is the
 * RX/idle flag (clear in TX), bit4 the constant 1, bit5 marks a tuned band and
 * bits6-7 carry fftdec. */
static uint8_t fpga_ctrl2_build(bool tx_active, uint8_t fftdec,
                                const DrvFpgaTuning *tuning) {
    uint8_t ctrl2 = (uint8_t)(fftdec << DRV_FPGA_CTRL_CTRL2_FFTDEC_SHIFT);

    if (!tx_active) {
        ctrl2 |= DRV_FPGA_CTRL2_RX_ACTIVE;
    }
    ctrl2 |= DRV_FPGA_CTRL2_CONST;
    ctrl2 |= tuning->zone_gray;
    if (tuning->freq_hz != 0u) {
        ctrl2 |= DRV_FPGA_CTRL2_BAND;
    }
    return ctrl2;
}

/* CTRL3: 0x03 in RX/idle; in TX the band class from the mode byte and the
 * per-band calibration value. band_idx belongs to the branch tuning set. */
static uint8_t fpga_ctrl3_build(bool tx_active, uint8_t band_idx) {
    if (!tx_active) {
        return DRV_FPGA_CTRL3_RX_IDLE;
    }
    return fpga_ctrl3_tx(s_ident.calib.ctrl3_mode_byte,
                         s_ident.calib.band_vals[band_idx]);
}

/* CTRL1 from the requested branch. RX-active (bit0) when the next frame asks
 * for RX, TX-active (bit2) when it asks for TX; both clear in idle. The frame
 * deliberately carries tx_active_requested, not tx_active: on the switching
 * tick the sample branch still emits the previous mode's sample, and the new
 * branch takes over on the next tick. */
static uint8_t fpga_ctrl1_build(bool tx_active, uint8_t mode) {
    uint8_t ctrl1 = 0u;

    if (mode != DRV_FPGA_MODE_IDLE) {
        ctrl1 = tx_active ? DRV_FPGA_CTRL1_TX_ACTIVE : DRV_FPGA_CTRL1_RX_ACTIVE;
    }
    /* ADC driver and PGA stay enabled; dither and randomization are optional
     * modes and currently off. */
    ctrl1 |= DRV_FPGA_CTRL1_ADC_DRIVER;
    ctrl1 |= DRV_FPGA_CTRL1_ADC_PGA;
    /* ctrl1 |= DRV_FPGA_CTRL1_ADC_RAND; */
    /* ctrl1 |= DRV_FPGA_CTRL1_ADC_DITH; */
    return ctrl1;
}

static void fpga_send_control_frame(void) {
    DrvFpgaCtrlFrame       frame;
    bool                   tx_active = s_state.tx_active_requested;
    uint8_t                mode      = s_state.mode;
    /* Effective fftdec for the branch this frame selects: the FPGA runs at
     * 48 kHz in TX, so the TX branch always carries DRV_FPGA_FFTDEC_MAX. The RX
     * request is carried as-is and committed after the frame is sent. */
    uint8_t                fftdec    = tx_active ? DRV_FPGA_FFTDEC_MAX
                                                 : s_state.fftdec_requested;
    const DrvFpgaTuning   *branch    = fpga_branch_tuning();

    memset(&frame, 0, sizeof(frame));
    frame.opcode      = DRV_FPGA_OP_CONTROL;
    frame.ctrl1       = fpga_ctrl1_build(tx_active, mode);
    /* ADC offset stays 0 (memset); fixed crystal-corrector dividers. */
    frame.tcxo_div[1] = 2u;
    frame.vcxo_div[1] = 2u;
    frame.ctrl2       = fpga_ctrl2_build(tx_active, fftdec, branch);
    frame.ctrl3       = fpga_ctrl3_build(tx_active, branch->band_idx);
    frame.rx_cic_gain = fpga_calib_rx_cic_gain(fftdec);
    frame.tx_cic_gain = s_ident.calib.tx_cic_gain;
    frame.dac_gain    = s_ident.calib.dac_gain;
    /* The RX1/RX2 slots carry the branch NCO and the TX slot the branch AUX
     * word; the application has already folded the VFO, split and RIT/XIT
     * selection into the RX and TX frequencies. */
    fpga_put_be32(frame.rx1_nco, branch->nco);
    fpga_put_be32(frame.rx2_nco, branch->nco);
    fpga_put_be32(frame.tx_nco, branch->aux);

    fpga_bus_moder_out();
    fpga_bus_write_byte(frame.opcode);
    fpga_bus_strobe();
    for (uint32_t i = 1u; i < sizeof(frame); ++i) {
        fpga_bus_write_byte(((const uint8_t *)&frame)[i]);
        fpga_bus_byte_clock();
    }
}

/* One 16-bit status word, MSB first. The two bytes are read into separate
 * temporaries so the bus order does not depend on the compiler's operand
 * evaluation order. */
static int16_t fpga_read_status_word(void) {
    uint8_t hi = fpga_bus_read_byte_clocked();
    uint8_t lo = fpga_bus_read_byte_clocked();

    return (int16_t)(((uint16_t)hi << 8u) | lo);
}

static void fpga_read_status_words(void) {
    fpga_bus_send_opcode(DRV_FPGA_OP_READ_STATUS);
    fpga_bus_moder_in();

    uint8_t flags = fpga_bus_read_byte_clocked();

    s_stat.status.adc_min = fpga_read_status_word();
    s_stat.status.adc_max = fpga_read_status_word();
    s_stat.status.adc_raw = fpga_read_status_word();

    /* The OTR bits are meaningless until the FPGA has settled; the three words
     * are good on every read. */
    if (s_stat.warmup < DRV_FPGA_STATUS_WARMUP) {
        s_stat.warmup++;
    }
    if (s_stat.warmup >= DRV_FPGA_STATUS_WARMUP) {
        s_stat.status.adc_otr = (flags & 0x01u) != 0u;
        s_stat.status.dac_otr = (flags & 0x02u) != 0u;
    }
}

static void fpga_reset_pair(bool *armed, bool *step, uint8_t op_on,
                            uint8_t op_off) {
    fpga_bus_send_opcode(*step ? op_off : op_on);
    *step = !*step;
    if (!*step) {
        *armed = false;
    }
}

static int32_t fpga_apply_tx_gain(int32_t x, int32_t gain_q15) {
    return (int32_t)(((int64_t)x * gain_q15) >> 15);
}

/* --- per-tick service --------------------------------------------------- */

static void fpga_service_rx(void) {
    /* Keep the ring cursor and the tick's sample count local and publish them
     * once: reading the volatile position per sample only costs load/store
     * slots in the bus-bound loop. The consumer cannot observe the half-written
     * tick because this ISR is the highest-priority context. */
    uint32_t       count = s_state.m_samples;
    bool           inv   = s_tune.rx.inv;
    uint32_t       wr    = s_raw.wr;

    fpga_bus_send_opcode(DRV_FPGA_OP_RX_IQ);
    fpga_bus_moder_in();

    for (uint32_t k = 0; k < count; ++k) {
        int32_t q = fpga_read_iq_word();
        int32_t i = fpga_read_iq_word();
        if (inv) {
            int32_t t = i;
            i = q;
            q = t;
        }
        fpga_store_raw(&wr, i, q);
    }
    s_raw.wr = wr;
    s_count.samples += count;
}

static void fpga_service_tx(void) {
    uint32_t rd = s_tx.rd;
    uint32_t wr = s_tx.wr;
    int32_t  gain = s_tx_gain_q15;
    bool     muted = false;

    fpga_bus_send_opcode(DRV_FPGA_OP_TX_IQ);

    /*
     * TX emits I then Q. The channel order may have to be swapped once TX is
     * verified on hardware.
     *
     * The mute countdown runs before the empty-ring early exit so it always
     * expires, even while no TX data is published. While muted the FPGA gets
     * zeros; with data present the read pointer still advances as if the
     * sample had been consumed, so stale audio cannot leave after the mute.
     *
     * An empty ring (rd == wr) emits zeros and leaves the read pointer alone:
     * the FPGA sends zeros from the entry into TX until the first published
     * block, and again after every underrun.
     */
    if (s_tx_mute_ticks != 0u) {
        s_tx_mute_ticks--;
        muted = true;
    }

    if (rd == wr) {
        fpga_bus_write_word(0);
        fpga_bus_write_word(0);
        return;
    }

    if (muted || gain == 0) {
        /* Muted or fully limited: still consume the staged sample so stale
         * audio cannot leave once the output reopens. */
        fpga_bus_write_word(0);
        fpga_bus_write_word(0);
    } else if (gain == 32768) {
        fpga_bus_write_word(s_tx.q[rd]);
        fpga_bus_write_word(s_tx.i[rd]);
    } else {
        fpga_bus_write_word(fpga_apply_tx_gain(s_tx.q[rd], gain));
        fpga_bus_write_word(fpga_apply_tx_gain(s_tx.i[rd], gain));
    }

    rd++;
    if (rd >= DRV_FPGA_TX_STAGING_SAMPLES) {
        rd = 0u;
    }
    s_tx.rd = rd;
}

/* Warm-up tick: emit opcode 0x03 with one zero I/Q sample. Sent on each of the
 * first DRV_FPGA_TX_WARMUP ticks so the FPGA RX path comes up; the staging
 * buffers are not touched. */
static void fpga_service_tx_warmup(void) {
    fpga_bus_send_opcode(DRV_FPGA_OP_TX_IQ);
    fpga_bus_write_word(0);
    fpga_bus_write_word(0);
}

/* Sample-rate configuration only: fftdec selects M = 8/4/2/1, and the next
 * control frame carries the new value. FFT timing and buffering belong to the
 * application. */
static void fpga_sample_rate_config(uint8_t fftdec) {
    s_state.fftdec    = fftdec;
    s_state.m_samples = DRV_FPGA_SAMPLES_PER_TICK_BASE >> fftdec;
}

static void fpga_service_deferred(void) {
    if (s_cmd.update_pending) {
        bool requested_tx;

        s_cmd.update_pending = false;
        requested_tx         = s_state.tx_active_requested;

        /* Start a new raw generation on the TX->RX switch. */
        if (!requested_tx && s_state.tx_active) {
            s_raw.generation++;
        }
        fpga_send_control_frame();

        /* The frame above carries the requested branch. The sample branch and
         * the effective rate only switch now, so the tick that sent the frame
         * still emitted a sample for the previous branch; the new branch starts
         * on the next PE10 tick. Runs under the IRQ mask held by
         * Drv_Fpga_Service, so the consumer never sees a mismatched branch or
         * M/generation pair. */
        s_state.tx_active = requested_tx;

        if (!requested_tx) {
            /* Back in RX: commit the RX fftdec accumulated while transmitting.
             * The frame already carried the same value, so the next tick reads
             * the new M. Bump the raw generation only when the rate actually
             * changed, so RX consumers resync the raw ring. */
            s_raw.invalidate_pending = false;
            if (s_state.fftdec_requested != s_state.fftdec) {
                fpga_sample_rate_config(s_state.fftdec_requested);
                s_raw.generation++;
            }
        }
        /* While TX is active the RX fftdec request stays pending (and
         * invalidate_pending stays set) until the return to RX; the TX branch
         * always runs at 48 kHz regardless of host rate changes. */
        return;
    }
    if (s_cmd.status_request) {
        s_cmd.status_request = false;
        fpga_read_status_words();
        return;
    }
    if (s_cmd.rx_reset) {
        fpga_reset_pair(&s_cmd.rx_reset, &s_cmd.rx_reset_step,
                        DRV_FPGA_OP_RX_RESET, DRV_FPGA_OP_RX_RESET_OFF);
        return;
    }
    if (s_cmd.tx_reset) {
        fpga_reset_pair(&s_cmd.tx_reset, &s_cmd.tx_reset_step,
                        DRV_FPGA_OP_TX_RESET, DRV_FPGA_OP_TX_RESET_OFF);
        return;
    }
}

static void fpga_exti_thunk(void *ctx) {
    (void)ctx;
    Drv_Fpga_Service();
}

/* --- init self-test ----------------------------------------------------- */

/*
 * One walk exchange: drive the byte on PE0..PE7 as outputs (clear the lines
 * first, strobe, then set the byte and latch it with a PE9 pulse), release the
 * lines to inputs, and read the byte back while PE9 is high.
 */
static uint8_t fpga_walk_byte(uint8_t value) {
    fpga_bus_moder_out();
    GPIOE->BSRR = DRV_FPGA_DATA_MASK << 16u;
    fpga_bus_strobe();
    GPIOE->BSRR = (uint32_t)value | (DRV_FPGA_DATA_MASK << 16u);
    fpga_bus_byte_clock();
    fpga_bus_moder_in();
    return fpga_bus_read_byte_clocked();
}

static uint32_t fpga_walk_test(void) {
    uint32_t errors = 0u;

    for (uint32_t i = 0; i < 256u; ++i) {
        if (fpga_walk_byte((uint8_t)i) != (uint8_t)i) {
            errors++;
            Bsp_Time_DelayMs(300u);
        }
    }
    return errors;
}

static int fpga_read_version(uint8_t ver[3]) {
    fpga_bus_send_opcode(DRV_FPGA_OP_READ_VERSION);
    fpga_bus_moder_in();

    for (uint32_t i = 0; i < 3u; ++i) {
        ver[i] = fpga_bus_read_byte_clocked();
    }
    if (ver[0] == 0u && ver[1] == 0u && ver[2] == 0u) {
        return -1;
    }
    return 0;
}

int Drv_Fpga_Init(const DrvFpgaCalib *calib) {
    uint32_t errors;
    uint8_t  version[3];
    int      rc;

    fpga_build_pin_masks();

    if (calib != 0) {
        s_ident.calib = *calib;
    } else {
        s_ident.calib = (DrvFpgaCalib){ 0 };
    }

    s_state.mode                  = DRV_FPGA_MODE_ACTIVE;
    s_state.tx_active             = false;
    s_state.tx_active_requested   = false;
    s_raw.wr                      = 0u;
    s_raw.generation              = 0u;
    s_raw.invalidate_pending      = false;
    s_tx.rd                       = 0u;
    s_tx.wr                       = 0u;
    s_tx_gain_q15                 = 32768;
    s_stat.warmup                 = 0u;
    s_stat.tx_warmup              = 0u;
    s_stat.status                 = (DrvFpgaStatus){ 0 };
    s_cmd.update_pending          = false;
    s_cmd.status_request          = false;
    s_cmd.tx_reset                = false;
    s_cmd.rx_reset                = false;
    s_cmd.tx_reset_step           = false;
    s_cmd.rx_reset_step           = false;
    s_tune.rx                     = (DrvFpgaTuning){ 0 };
    s_tune.tx                     = (DrvFpgaTuning){ 0 };
    s_tune.keepalive_ticks        = 0u;
    s_tx_mute_ticks               = 0u;
    s_state.fftdec_requested      = DRV_FPGA_FFTDEC_DEFAULT;
    fpga_sample_rate_config(DRV_FPGA_FFTDEC_DEFAULT);

    fpga_bus_send_opcode(DRV_FPGA_OP_RX_RESET_OFF);
    fpga_bus_send_opcode(DRV_FPGA_OP_TX_RESET_OFF);

    errors = fpga_walk_test();
    if (errors != 0u) {
        LOG_WARN("walk self-test %u/%u lines mismatched", (unsigned)errors,
                 (unsigned)(256u - errors));
    } else {
        LOG_INFO("walk self-test ok");
    }

    rc = fpga_read_version(version);
    if (rc != 0) {
        LOG_WARN("fpga not found, continuing");
        s_ident.version[0] = 0u;
        s_ident.version[1] = 0u;
        s_ident.version[2] = 0u;
    } else {
        LOG_INFO("fpga fw version %u.%u.%u", (unsigned)version[0],
                 (unsigned)version[1], (unsigned)version[2]);
        memcpy(s_ident.version, version, sizeof(s_ident.version));
    }

    Drv_Fpga_SetRxFrequency(14100000u);
    Drv_Fpga_SetTxFrequency(14100000u);

    if (Bsp_Exti_Register(FPGA_IRQ_Pin, fpga_exti_thunk, 0) != 0) {
        LOG_WARN("exti register failed");
    }
    return rc;
}

/* --- public control API ------------------------------------------------- */

static void fpga_compute_tuning(DrvFpgaTuning *tuning, uint32_t freq_hz) {
    uint8_t inv = 0u;

    tuning->freq_hz   = freq_hz;
    tuning->nco       = fpga_nco_word(fpga_fold(freq_hz, &inv));
    tuning->aux       = fpga_aux_word(freq_hz);
    tuning->zone_gray = fpga_ctrl2_zone(freq_hz);
    tuning->band_idx  = fpga_band_index(freq_hz);
    tuning->inv       = (inv != 0u);
}

/* Publish a freshly computed branch tuning. All math (the 64-bit divide plus
 * the band/zone classification) runs before interrupts are masked, so a PE10
 * tick is never delayed by it. The control frame is armed only when the updated
 * branch is the one currently requested; otherwise the value waits for the next
 * SetTxActive switch that selects it. A frequency of 0 ("unset") stays unset so
 * the branch is never tuned. */
static void fpga_publish_tuning(DrvFpgaTuning *slot, bool tx_branch,
                                uint32_t freq_hz) {
    DrvFpgaTuning tuning;
    uint32_t      primask;

    fpga_compute_tuning(&tuning, freq_hz);

    primask = Bsp_Irq_Save();
    *slot = tuning;
    if (s_state.tx_active_requested == tx_branch) {
        s_cmd.update_pending = true;
    }
    Bsp_Irq_Restore(primask);
}

void Drv_Fpga_SetRxFrequency(uint32_t freq_hz) {
    fpga_publish_tuning(&s_tune.rx, false, freq_hz);
}

void Drv_Fpga_SetTxFrequency(uint32_t freq_hz) {
    fpga_publish_tuning(&s_tune.tx, true, freq_hz);
}

void Drv_Fpga_TxMute(uint32_t ms) {
    /* ms * 48 ticks/ms; keep the conversion in 64 bits so long values cannot
     * wrap. Overlapping requests extend the mute instead of shortening it. */
    uint64_t       ticks = ((uint64_t)ms * BOARD_AUDIO_SAMPLE_RATE_HZ) / 1000u;
    uint32_t       primask;

    if (ticks > UINT32_MAX) {
        ticks = UINT32_MAX;
    }
    primask = Bsp_Irq_Save();
    if (s_tx_mute_ticks < (uint32_t)ticks) {
        s_tx_mute_ticks = (uint32_t)ticks;
    }
    Bsp_Irq_Restore(primask);
}

void Drv_Fpga_GetVersion(uint8_t ver[3]) {
    if (ver == 0) {
        return;
    }
    memcpy(ver, s_ident.version, sizeof(s_ident.version));
}

void Drv_Fpga_SetSampleRate(uint8_t fftdec) {
    uint32_t primask;

    if (fftdec > DRV_FPGA_FFTDEC_MAX) {
        fftdec = DRV_FPGA_FFTDEC_MAX;
    }
    primask = Bsp_Irq_Save();
    /* Store the request only; the active rate changes in fpga_service_deferred()
     * once the matching control frame has left the bus. */
    if (fftdec != s_state.fftdec_requested) {
        s_state.fftdec_requested  = fftdec;
        s_raw.invalidate_pending  = (fftdec != s_state.fftdec);
        s_cmd.update_pending      = true;
    }
    Bsp_Irq_Restore(primask);
}

void Drv_Fpga_SetTxActive(int active) {
    bool     value   = (active != 0);
    uint32_t primask = Bsp_Irq_Save();

    /* Only request the branch; tx_active changes in fpga_service_deferred()
     * once the matching control frame has been sent, so the tick that sends the
     * frame still emits a sample for the previous branch. A false -> true
     * request also drops unread staging data by collapsing the ring
     * (rd = wr), so data from an earlier transmission is never emitted. No
     * memset is needed. */
    if (s_state.tx_active_requested != value) {
        s_state.tx_active_requested = value;
        if (value) {
            s_tx.rd = s_tx.wr;
        }
        s_cmd.update_pending = true;
    }
    Bsp_Irq_Restore(primask);
}

uint32_t Drv_Fpga_TxWrite(const int32_t *i, const int32_t *q, uint32_t count) {
    uint32_t primask;
    uint32_t rd;
    uint32_t wr;
    uint32_t free_samples;
    uint32_t accepted;
    uint32_t first;
    uint32_t second;

    if (i == 0 || q == 0 || count == 0u) {
        return 0u;
    }

    /* Snapshot both indices; the reader may advance concurrently. */
    primask = Bsp_Irq_Save();
    rd      = s_tx.rd;
    wr      = s_tx.wr;
    Bsp_Irq_Restore(primask);

    /* Keep one slot free so rd == wr always means "empty": the consumer never
     * sees a full ring and the producer never has to overwrite unread data. */
    free_samples = (rd > wr) ? (rd - wr - 1u)
                             : (DRV_FPGA_TX_STAGING_SAMPLES - wr + rd - 1u);
    accepted     = (count < free_samples) ? count : free_samples;
    if (accepted == 0u) {
        return 0u;
    }

    /* Copy outside the interrupt mask; the new data is invisible to the PE10
     * ISR until s_tx.wr is published below. Handle wrap with two memcpys. */
    first = DRV_FPGA_TX_STAGING_SAMPLES - wr;
    if (first > accepted) {
        first = accepted;
    }
    second = accepted - first;
    memcpy(&s_tx.i[wr], i, first * sizeof(int32_t));
    memcpy(&s_tx.q[wr], q, first * sizeof(int32_t));
    if (second != 0u) {
        memcpy(&s_tx.i[0], &i[first], second * sizeof(int32_t));
        memcpy(&s_tx.q[0], &q[first], second * sizeof(int32_t));
    }

    wr += accepted;
    if (wr >= DRV_FPGA_TX_STAGING_SAMPLES) {
        wr -= DRV_FPGA_TX_STAGING_SAMPLES;
    }
    primask = Bsp_Irq_Save();
    s_tx.wr = wr;
    Bsp_Irq_Restore(primask);

    return accepted;
}

void Drv_Fpga_SetTxGainQ15(int32_t gain) {
    uint32_t primask;

    if (gain < 0) {
        gain = 0;
    } else if (gain > 32768) {
        gain = 32768;
    }
    primask = Bsp_Irq_Save();
    s_tx_gain_q15 = gain;
    Bsp_Irq_Restore(primask);
}

void Drv_Fpga_RequestStatus(void) {
    s_cmd.status_request = true;
}

void Drv_Fpga_GetStatus(DrvFpgaStatus *out) {
    uint32_t primask;

    if (out == 0) {
        return;
    }
    primask = Bsp_Irq_Save();
    *out    = s_stat.status;
    Bsp_Irq_Restore(primask);
}

/* Two-step RX/TX module reset: the first call asserts the reset and the next
 * PE10 tick releases it. */
void Drv_Fpga_TxReset(void) {
    s_cmd.tx_reset      = true;
    s_cmd.tx_reset_step = false;
}

void Drv_Fpga_RxReset(void) {
    s_cmd.rx_reset      = true;
    s_cmd.rx_reset_step = false;
}

uint32_t Drv_Fpga_GetSamplesPerTick(void) {
    /* Actual branch: TX always runs at 48 kHz (M = 1); RX uses the committed
     * RX fftdec. tx_active flips only in the deferred control frame. */
    return s_state.tx_active ? 1u : s_state.m_samples;
}

uint32_t Drv_Fpga_TickCount(void) {
    uint32_t primask = Bsp_Irq_Save();
    uint32_t value   = s_count.ticks;

    s_count.ticks = 0u;
    Bsp_Irq_Restore(primask);
    return value;
}

uint32_t Drv_Fpga_SampleCount(void) {
    uint32_t primask = Bsp_Irq_Save();
    uint32_t value   = s_count.samples;

    s_count.samples = 0u;
    Bsp_Irq_Restore(primask);
    return value;
}

const DrvFpgaRawWindow *Drv_Fpga_GetRawWindow(void) {
    static const DrvFpgaRawWindow window = {
        .i     = s_raw.i,
        .q     = s_raw.q,
        .count = DRV_FPGA_RAW_WINDOW_SAMPLES,
    };

    return &window;
}

uint32_t Drv_Fpga_GetPos(void) {
    return s_raw.wr;
}

uint8_t Drv_Fpga_GetFftdec(void) {
    /* Effective fftdec of the actual branch: the FPGA always runs at 48 kHz in
     * TX. fftdec keeps the RX value and is not overwritten while transmitting. */
    return s_state.tx_active ? DRV_FPGA_FFTDEC_MAX : s_state.fftdec;
}

uint8_t Drv_Fpga_GetRequestedFftdec(void) {
    return s_state.fftdec_requested;
}

uint32_t Drv_Fpga_GetRawGeneration(void) {
    return s_raw.generation;
}

void Drv_Fpga_Service(void) {
    uint32_t primask = Bsp_Irq_Save();
    if (s_stat.tx_warmup < DRV_FPGA_TX_WARMUP) {
        /* Power-on warm-up: 0x03 with a zero sample on every tick, in addition
         * to the RX branch below. Takes priority over the normal TX branch. */
        s_stat.tx_warmup++;
        fpga_service_tx_warmup();
    } else if (s_state.tx_active) {
        fpga_service_tx();
    }

    if (!s_state.tx_active && s_state.mode != DRV_FPGA_MODE_IDLE) {
        fpga_service_rx();
    }

    if (++s_tune.keepalive_ticks >= DRV_FPGA_KEEPALIVE_TICKS) {
        s_tune.keepalive_ticks = 0u;
        s_cmd.update_pending   = true;
    }

    fpga_service_deferred();
    s_count.ticks++;
    Bsp_Irq_Restore(primask);
}
