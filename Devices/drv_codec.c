#include "drv_codec.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_gpio.h"
#include "bsp_i2c_bus.h"
#include "bsp_log.h"
#include "bsp_prof.h"
#include "bsp_time.h"
#include "i2s.h"
#include "main.h"

#define LOG_TAG "drv.codec"

/* PB10, active low. */
#define DRV_CODEC_RESET_HOLD_MS     100u
#define DRV_CODEC_RESET_RECOVER_MS  100u

#define AIC3204_REG_PAGE_SEL        0x00u
#define AIC3204_REG_SOFT_RESET      0x01u
#define AIC3204_PAGE_0              0x00u
#define AIC3204_PAGE_1              0x01u
#define AIC3204_SOFT_RESET          0x01u

/*
 * Registers the init table writes once but that are re-programmed at runtime
 * (input routing and levels); the setters reuse these names instead of raw
 * numbers. The Px prefix is the page the register lives on. MICBIAS is written
 * once by the init table; the MICPGA registers are not part of it and exist
 * only for runtime use.
 */
#define AIC3204_P0_REG_DAC_ROUTING  0x40u
#define AIC3204_P0_REG_DAC_VOL_L    0x41u
#define AIC3204_P0_REG_DAC_VOL_R    0x42u
#define AIC3204_P0_REG_ADC_CH_SETUP 0x51u
#define AIC3204_P0_REG_ADC_ROUTING  0x52u
#define AIC3204_P0_REG_ADC_VOL_L    0x53u
#define AIC3204_P0_REG_ADC_VOL_R    0x54u
#define AIC3204_P1_REG_LDO_CTRL     0x02u
#define AIC3204_P1_REG_HPL_ROUTING  0x0Cu
#define AIC3204_P1_REG_HPR_ROUTING  0x0Du
#define AIC3204_P1_REG_LOL_ROUTING  0x0Eu
#define AIC3204_P1_REG_LOR_ROUTING  0x0Fu
#define AIC3204_P1_REG_HP_GAIN_L    0x10u
#define AIC3204_P1_REG_HP_GAIN_R    0x11u
#define AIC3204_P1_REG_LO_GAIN_L    0x12u
#define AIC3204_P1_REG_LO_GAIN_R    0x13u
#define AIC3204_P1_REG_MICBIAS      0x33u
#define AIC3204_P1_REG_MICPGA_POS_L 0x34u
#define AIC3204_P1_REG_MICPGA_NEG_L 0x36u
#define AIC3204_P1_REG_MICPGA_POS_R 0x37u
#define AIC3204_P1_REG_MICPGA_NEG_R 0x39u
#define AIC3204_P1_REG_MICPGA_L     0x3Bu
#define AIC3204_P1_REG_MICPGA_R     0x3Cu

/*
 * Output routing (page 1 R12..R15). HPL and HPR are normally both fed from the
 * left DAC so the host volume can drive the per-output HPR gain without moving
 * HPL: HPR takes the left DAC with inverted sign (D4), HPL the non-inverted one
 * (D3). Voice recording switches HPR to the right DAC at runtime
 * (Drv_Codec_SetHprSource). LOL/LOR stay on their own DAC.
 */
#define AIC3204_ROUTE_HPL_DACL_P    0x08u
#define AIC3204_ROUTE_HPR_DACL_N    0x10u
#define AIC3204_ROUTE_HPR_DACR_P    0x08u
#define AIC3204_ROUTE_LOL_DACL      0x08u
#define AIC3204_ROUTE_LOR_DACR      0x08u

/*
 * ADC channel 1st-order IIR coefficients in buffer A. Q23 format:
 * H(z) = (N0 + N1*z^-1) / (2^23 - D1*z^-1), 24-bit two's complement, MSB
 * first in three consecutive registers. Butterworth 1st-order highpass,
 * fc = 50 Hz at BOARD_AUDIO_SAMPLE_RATE_HZ; regenerate with
 * scipy.signal.iirfilter(1, 50*2/48000, btype='highpass', ftype='butter').
 * The same coefficients go to both channels.
 */
#define AIC3204_PAGE_8              0x08u
#define AIC3204_PAGE_9              0x09u
#define AIC3204_P8_REG_ADC_IIR_N0_L 0x18u
#define AIC3204_P8_REG_ADC_IIR_N1_L 0x1Cu
#define AIC3204_P8_REG_ADC_IIR_D1_L 0x20u
#define AIC3204_P9_REG_ADC_IIR_N0_R 0x20u
#define AIC3204_P9_REG_ADC_IIR_N1_R 0x24u
#define AIC3204_P9_REG_ADC_IIR_D1_R 0x28u
#define AIC3204_ADC_IIR_N0          0x7F951Eu
#define AIC3204_ADC_IIR_N1          0x806AE2u
#define AIC3204_ADC_IIR_D1          0x7F2A3Cu

#define DRV_CODEC_DMA_BLOCKS        2u
#define DRV_CODEC_DMA_WORDS         (DRV_CODEC_DMA_BLOCKS * DRV_CODEC_DMA_HALF_WORDS)
#define DRV_CODEC_DMA_POOL_BYTES    8192u

typedef struct {
    uint8_t page;
    uint8_t reg;
    uint8_t value;
} DrvCodecRegWrite;

typedef enum {
    DRV_CODEC_STATE_IDLE = 0,
    DRV_CODEC_STATE_READY,
    DRV_CODEC_STATE_RUNNING,
    DRV_CODEC_STATE_ERROR
} DrvCodecState;

static int32_t s_rx_dma[DRV_CODEC_DMA_WORDS]
    __attribute__((section(".dma_d2"), aligned(32)));
static int32_t s_tx_dma[DRV_CODEC_DMA_WORDS]
    __attribute__((section(".dma_d2"), aligned(32)));

_Static_assert(sizeof(int32_t) == 4, "sample format is int32");
_Static_assert(DRV_CODEC_DMA_HALF_WORDS * sizeof(int32_t) == 512u,
               "dma half-buffer is 512 bytes");
_Static_assert(sizeof(s_rx_dma) ==
                   2u * DRV_CODEC_DMA_HALF_WORDS * sizeof(int32_t),
               "dma buffer is 2 half-blocks");
_Static_assert(sizeof(s_tx_dma) ==
                   2u * DRV_CODEC_DMA_HALF_WORDS * sizeof(int32_t),
               "dma buffer is 2 half-blocks");
_Static_assert(_Alignof(s_rx_dma) == 32u, "dma buffer cache-line aligned");
_Static_assert(_Alignof(s_tx_dma) == 32u, "dma buffer cache-line aligned");
_Static_assert(2u * sizeof(s_rx_dma) <= DRV_CODEC_DMA_POOL_BYTES,
               "codec DMA fits the .dma_d2 pool");

static const DrvCodecRegWrite s_init_seq_pre[] = {
    /* A - page 0: clocks and interface */
    { AIC3204_PAGE_0, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_0 },
    // PLL
    { AIC3204_PAGE_0, 0x04u, 0x03u }, /* CODEC_CLKIN = PLL_CLK, PLL_CLKIN = MCLK */
    { AIC3204_PAGE_0, 0x05u, 0x91u }, /* Turn on PLL, P=1, R=1 */
    { AIC3204_PAGE_0, 0x06u, 0x07u }, /* J = 7 */
    { AIC3204_PAGE_0, 0x07u, 0x00u }, /* D MSB */
    { AIC3204_PAGE_0, 0x08u, 0x00u }, /* D LSB (D=0) */
    // DAC
    { AIC3204_PAGE_0, 0x0Bu, 0x82u }, /* NDAC = 2 */
    { AIC3204_PAGE_0, 0x0Cu, 0x87u }, /* MDAC = 7 */
    { AIC3204_PAGE_0, 0x0Du, 0x00u }, /* DOSR MSB = 0 */
    { AIC3204_PAGE_0, 0x0Eu, 0x80u }, /* DOSR LSB = 128 */
    // ADC
    { AIC3204_PAGE_0, 0x12u, 0x87u }, /* NADC = 7 */
    { AIC3204_PAGE_0, 0x13u, 0x82u }, /* MADC = 2 */
    { AIC3204_PAGE_0, 0x14u, 0x80u }, /* AOSR = 128 */

    { AIC3204_PAGE_0, 0x1Bu, 0x30u }, /* Audio Interface Setting 1 */
    { AIC3204_PAGE_0, 0x3Cu, 0x19u }, /* DAC Signal Processing Block */
    { AIC3204_PAGE_0, 0x3Du, 0x01u }, /* ADC Signal Processing Block */
    /* B - page 1: power */
    { AIC3204_PAGE_1, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1 },
    { AIC3204_PAGE_1, 0x01u, 0x08u }, /* Power Configuration */
    { AIC3204_PAGE_1, AIC3204_P1_REG_LDO_CTRL, 0x01u }, /* LDO Control */
    { AIC3204_PAGE_1, 0x0Au, 0x03u }, /* Common Mode Control */
    { AIC3204_PAGE_1, 0x3Du, 0x00u }, /* ADC Power Tune */
    { AIC3204_PAGE_1, 0x03u, 0x00u }, /* Playback Config 1 */
    { AIC3204_PAGE_1, 0x04u, 0x00u }, /* Playback Config 2 */
    { AIC3204_PAGE_1, 0x47u, 0x32u }, /* Analog Input Quick Charging */
    { AIC3204_PAGE_1, 0x7Bu, 0x05u }, /* Reference Power-up */
    /*
     * MICBIAS: set once at init, never switched with TX/RX. Needed by the
     * electret hand mic and the internal mic; both are silent without it. Must
     * come after Reference Power-up.
     */
    { AIC3204_PAGE_1, AIC3204_P1_REG_MICBIAS, 0x60u },
};

static const DrvCodecRegWrite s_init_seq_post[] = {
    /* C - page 0: input routing and ADC levels */
    { AIC3204_PAGE_0, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_0 },
    { AIC3204_PAGE_0, AIC3204_P0_REG_ADC_CH_SETUP, 0xC0u }, /* ADC Channel Setup */
    { AIC3204_PAGE_0, AIC3204_P0_REG_ADC_ROUTING, 0x00u }, /* ADC Fine Gain */
    { AIC3204_PAGE_0, AIC3204_P0_REG_ADC_VOL_L, 0x00u }, /* Left ADC Volume */
    { AIC3204_PAGE_0, AIC3204_P0_REG_ADC_VOL_R, 0x00u }, /* Right ADC Volume */
    /* D - page 1: output routing and driver levels */
    { AIC3204_PAGE_1, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1 },
    { AIC3204_PAGE_1, 0x09u, 0x3Fu }, /* Output Driver Power Control */
    { AIC3204_PAGE_1, AIC3204_P1_REG_HPL_ROUTING, AIC3204_ROUTE_HPL_DACL_P },
    { AIC3204_PAGE_1, AIC3204_P1_REG_HPR_ROUTING, AIC3204_ROUTE_HPR_DACL_N },
    { AIC3204_PAGE_1, AIC3204_P1_REG_LOL_ROUTING, AIC3204_ROUTE_LOL_DACL },
    { AIC3204_PAGE_1, AIC3204_P1_REG_LOR_ROUTING, AIC3204_ROUTE_LOR_DACR },
    { AIC3204_PAGE_1, AIC3204_P1_REG_HP_GAIN_L, 0x40u }, /* HPL Driver Gain */
    { AIC3204_PAGE_1, AIC3204_P1_REG_HP_GAIN_R, 0x40u }, /* HPR Driver Gain */
    { AIC3204_PAGE_1, AIC3204_P1_REG_LO_GAIN_L, 0x40u }, /* LOL Driver Gain */
    { AIC3204_PAGE_1, AIC3204_P1_REG_LO_GAIN_R, 0x40u }, /* LOR Driver Gain */
    { AIC3204_PAGE_1, 0x14u, 0x25u }, /* Headphone Driver Startup */
    /* E - page 0: DAC routing and levels */
    { AIC3204_PAGE_0, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_0 },
    { AIC3204_PAGE_0, 0x3Fu, 0xD6u }, /* DAC Channel Setup 1 */
    { AIC3204_PAGE_0, AIC3204_P0_REG_DAC_ROUTING, 0x00u }, /* DAC Channel Setup 2 */
    { AIC3204_PAGE_0, AIC3204_P0_REG_DAC_VOL_L, 0x00u }, /* Left DAC Volume */
    { AIC3204_PAGE_0, AIC3204_P0_REG_DAC_VOL_R, 0x00u }, /* Right DAC Volume */
    /* F - page 0: ADC volume repeat */
    { AIC3204_PAGE_0, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_0 },
    { AIC3204_PAGE_0, AIC3204_P0_REG_ADC_VOL_L, 0x00u }, /* Left ADC Volume */
    { AIC3204_PAGE_0, AIC3204_P0_REG_ADC_VOL_R, 0x00u }, /* Right ADC Volume */
    /* G - page 0: DAC volume repeat */
    { AIC3204_PAGE_0, AIC3204_REG_PAGE_SEL, AIC3204_PAGE_0 },
    { AIC3204_PAGE_0, AIC3204_P0_REG_DAC_VOL_L, 0x00u }, /* Left DAC Volume */
    { AIC3204_PAGE_0, AIC3204_P0_REG_DAC_VOL_R, 0x00u }, /* Right DAC Volume */
};

static const BspPin        s_reset_pin = { CODEC_RESET_GPIO_Port,
                                           CODEC_RESET_Pin };
static DrvCodecBlockCb     s_cb;
static void               *s_ctx;
static volatile DrvCodecState s_state;

static int codec_write_reg(uint8_t reg, uint8_t value) {
    const uint8_t buf[2] = { reg, value };
    return Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, DRV_CODEC_ADDR7, buf,
                                sizeof(buf), DRV_CODEC_XFER_TIMEOUT_MS);
}

static int codec_write_coef24(uint8_t reg, uint32_t coef) {
    const uint8_t buf[4] = { reg, (uint8_t)(coef >> 16), (uint8_t)(coef >> 8),
                             (uint8_t)coef };
    return Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, DRV_CODEC_ADDR7, buf,
                                sizeof(buf), DRV_CODEC_XFER_TIMEOUT_MS);
}

static int codec_apply_table(const DrvCodecRegWrite *seq, size_t n) {
    uint8_t page = AIC3204_PAGE_0;

    for (unsigned i = 0; i < n; ++i) {
        const DrvCodecRegWrite *w = &seq[i];
        int                     rc;

        if (w->reg == AIC3204_REG_PAGE_SEL) {
            if (w->value != w->page) {
                return -EIO;
            }
            page = w->value;
        } else if (w->page != page) {
            return -EIO;
        }
        rc = codec_write_reg(w->reg, w->value);
        if (rc != 0) {
            LOG_DEBUG("write step %u p%u r0x%02X failed (%d)",
                      i, (unsigned)w->page, (unsigned)w->reg, rc);
            return rc;
        }
    }
    return 0;
}

/*
 * ADC 1st-order IIR coefficients, buffer A. The ADC must be powered down for
 * these writes to be accepted (SLAA557 5.4.4/5.5.3); this runs between the
 * init sequence's B and C blocks, before block C powers the ADC up. Buffer A
 * is the live buffer because adaptive mode is off (P8_R1 D2 = 0, default).
 */
static int codec_setup_adc_dc_blocker(void) {
    int rc;

    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_8);
    if (rc != 0) {
        LOG_DEBUG("dc blocker page 8 select failed (%d)", rc);
        return rc;
    }
    rc = codec_write_coef24(AIC3204_P8_REG_ADC_IIR_N0_L, AIC3204_ADC_IIR_N0);
    if (rc != 0) {
        LOG_DEBUG("dc blocker left N0 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_coef24(AIC3204_P8_REG_ADC_IIR_N1_L, AIC3204_ADC_IIR_N1);
    if (rc != 0) {
        LOG_DEBUG("dc blocker left N1 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_coef24(AIC3204_P8_REG_ADC_IIR_D1_L, AIC3204_ADC_IIR_D1);
    if (rc != 0) {
        LOG_DEBUG("dc blocker left D1 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_9);
    if (rc != 0) {
        LOG_DEBUG("dc blocker page 9 select failed (%d)", rc);
        return rc;
    }
    rc = codec_write_coef24(AIC3204_P9_REG_ADC_IIR_N0_R, AIC3204_ADC_IIR_N0);
    if (rc != 0) {
        LOG_DEBUG("dc blocker right N0 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_coef24(AIC3204_P9_REG_ADC_IIR_N1_R, AIC3204_ADC_IIR_N1);
    if (rc != 0) {
        LOG_DEBUG("dc blocker right N1 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_coef24(AIC3204_P9_REG_ADC_IIR_D1_R, AIC3204_ADC_IIR_D1);
    if (rc != 0) {
        LOG_DEBUG("dc blocker right D1 failed (%d)", rc);
    }
    return rc;
}

static void codec_block_irq(unsigned half) {
    DrvCodecBlockCb cb = s_cb;

    if (cb != 0) {
        cb(&s_rx_dma[half * DRV_CODEC_DMA_HALF_WORDS],
           &s_tx_dma[half * DRV_CODEC_DMA_HALF_WORDS],
           DRV_CODEC_BLOCK_FRAMES, s_ctx);
    }
}

int Drv_Codec_Init(void) {
    int rc;

    Bsp_Gpio_Write(s_reset_pin, GPIO_PIN_RESET);
    Bsp_Time_DelayMs(DRV_CODEC_RESET_HOLD_MS);
    Bsp_Gpio_Write(s_reset_pin, GPIO_PIN_SET);
    Bsp_Time_DelayMs(DRV_CODEC_RESET_RECOVER_MS);

    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_0);
    if (rc != 0) {
        LOG_DEBUG("page select failed (%d)", rc);
        return rc;
    }
    rc = codec_write_reg(AIC3204_REG_SOFT_RESET, AIC3204_SOFT_RESET);
    if (rc != 0) {
        LOG_DEBUG("soft reset failed (%d)", rc);
        return rc;
    }
    rc = codec_apply_table(s_init_seq_pre,
                           sizeof(s_init_seq_pre) / sizeof(s_init_seq_pre[0]));
    if (rc != 0) {
        return rc;
    }
    rc = codec_setup_adc_dc_blocker();
    if (rc != 0) {
        return rc;
    }
    rc = codec_apply_table(s_init_seq_post,
                           sizeof(s_init_seq_post) / sizeof(s_init_seq_post[0]));
    if (rc != 0) {
        return rc;
    }
    s_state = DRV_CODEC_STATE_READY;
    LOG_INFO("aic3204 init done");
    return 0;
}

int Drv_Codec_SetBlockCallback(DrvCodecBlockCb cb, void *ctx) {
    s_cb  = cb;
    s_ctx = ctx;
    return 0;
}


int Drv_Codec_SetInputL(DrvCodecInputL input) {
    int rc;

    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1);
    if (rc != 0) {
        LOG_DEBUG("input routing page select failed (%d)", rc);
        return rc;
    }
    uint8_t pos = 0u, neg = 0u;
    if (input == DRV_CODEC_IN_L_HMIC) {
        /* hand mic:     IN1_L, single-ended */
        pos = 0x40u;
        neg = 0x40u;
    } else if (input == DRV_CODEC_IN_L_IMIC) {
        /* internal mic: IN2_L/IN2_R, differential */
        pos = 0x10u;
        neg = 0x10u;
    } else if (input == DRV_CODEC_IN_L_USB) {
        /* line-in L:(USB)    IN3_L, single-ended */
        pos = 0x4u;
        neg = 0x40u;
    }
    rc = codec_write_reg(AIC3204_P1_REG_MICPGA_POS_L, pos);
    if (rc != 0) {
        LOG_DEBUG("input routing r52 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_reg(AIC3204_P1_REG_MICPGA_NEG_L, neg);
    if (rc != 0) {
        LOG_DEBUG("input routing r54 failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_SetInputR(DrvCodecInputR input) {
    int rc;

    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1);
    if (rc != 0) {
        LOG_DEBUG("input routing page select failed (%d)", rc);
        return rc;
    }
    uint8_t pos = 0u, neg = 0u;
    if (input == DRV_CODEC_IN_R_HOST) {
        // IN3_R, single-ended
        pos = 0x4u;
        // Neg to CM
        neg = 0x40u;
    }
    rc = codec_write_reg(AIC3204_P1_REG_MICPGA_POS_R, pos);
    if (rc != 0) {
        LOG_DEBUG("input routing r55 failed (%d)", rc);
        return rc;
    }
    rc = codec_write_reg(AIC3204_P1_REG_MICPGA_NEG_R, neg);
    if (rc != 0) {
        LOG_DEBUG("input routing r57 failed (%d)", rc);
    }
    return rc;
}

static inline int SetInGain(uint8_t reg, uint8_t val) {
    int rc;
    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1);
    if (rc != 0) {
        LOG_DEBUG("input gain page select failed (%d)", rc);
        return rc;
    }

    if (val > DRV_CODEC_MICPGA_GAIN_MAX) {
        val = DRV_CODEC_MICPGA_GAIN_MAX;
    }
    rc = codec_write_reg(reg, val);
    return rc;
}

int Drv_Codec_SetInGainL(uint8_t val) {
    int     rc;
    rc = SetInGain(AIC3204_P1_REG_MICPGA_L, val);
    if (rc != 0) {
        LOG_DEBUG("input gain left failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_SetInGainR(uint8_t val) {
    int     rc;
    rc = SetInGain(AIC3204_P1_REG_MICPGA_R, val);
    if (rc != 0) {
        LOG_DEBUG("input gain left failed (%d)", rc);
    }
    return rc;
}

static inline int SetOutGain(uint8_t reg, bool on, int8_t level) {
    int rc;

    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1);
    if (rc != 0) {
        LOG_DEBUG("page select failed (%d)", rc);
        return rc;
    }
    if (level > DRV_CODEC_OUT_GAIN_MAX) {
        level = DRV_CODEC_OUT_GAIN_MAX;
    }
    if (level < DRV_CODEC_OUT_GAIN_MIN) {
        level = DRV_CODEC_OUT_GAIN_MIN;
    }
    if (!on && level == DRV_CODEC_OUT_GAIN_MIN) {
        /* A muted driver cannot also be at -6 dB (SLAA557). */
        level = DRV_CODEC_OUT_GAIN_MIN + 1;
    }
    uint8_t val = ((uint8_t)level & 0x3Fu) | (on ? 0u : 0x40u);
    rc = codec_write_reg(reg, val);
    return rc;
}

int Drv_Codec_SetHPLGain(bool on, int8_t level) {
    int rc;
    rc = SetOutGain(AIC3204_P1_REG_HP_GAIN_L, on, level);
    if (rc != 0) {
        LOG_DEBUG("SetHPLGain gain failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_SetHPRGain(bool on, int8_t level) {
    int rc;
    rc = SetOutGain(AIC3204_P1_REG_HP_GAIN_R, on, level);
    if (rc != 0) {
        LOG_DEBUG("SetHPRGain gain failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_SetHprSource(DrvCodecHprSource source) {
    uint8_t value;
    int     rc;

    switch (source) {
    case DRV_CODEC_HPR_SRC_DACL_N:
        value = AIC3204_ROUTE_HPR_DACL_N;
        break;
    case DRV_CODEC_HPR_SRC_DACR_P:
        value = AIC3204_ROUTE_HPR_DACR_P;
        break;
    default:
        return -EINVAL;
    }

    rc = codec_write_reg(AIC3204_REG_PAGE_SEL, AIC3204_PAGE_1);
    if (rc != 0) {
        LOG_DEBUG("hpr routing page select failed (%d)", rc);
        return rc;
    }
    rc = codec_write_reg(AIC3204_P1_REG_HPR_ROUTING, value);
    if (rc != 0) {
        LOG_DEBUG("SetHprSource failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_SetLOLGain(bool on, int8_t level) {
    int rc;
    rc = SetOutGain(AIC3204_P1_REG_LO_GAIN_L, on, level);
    if (rc != 0) {
        LOG_DEBUG("SetLOLGain gain failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_SetLORGain(bool on, int8_t level) {
    int rc;
    rc = SetOutGain(AIC3204_P1_REG_LO_GAIN_R, on, level);
    if (rc != 0) {
        LOG_DEBUG("SetLORGain gain failed (%d)", rc);
    }
    return rc;
}

int Drv_Codec_Start(void) {
    unsigned i;

    if (s_state == DRV_CODEC_STATE_IDLE) {
        return -EPERM;
    }
    if (s_state == DRV_CODEC_STATE_RUNNING) {
        return 0;
    }
    for (i = 0; i < DRV_CODEC_DMA_WORDS; ++i) {
        s_tx_dma[i] = 0;
    }
    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s3, (const uint16_t *)s_tx_dma,
                                      (uint16_t *)s_rx_dma,
                                      (uint16_t)DRV_CODEC_DMA_WORDS) !=
        HAL_OK) {
        s_state = DRV_CODEC_STATE_READY;
        return -EIO;
    }
    s_state = DRV_CODEC_STATE_RUNNING;
    return 0;
}

int Drv_Codec_Stop(void) {
    if (s_state == DRV_CODEC_STATE_RUNNING) {
        if (HAL_I2S_DMAStop(&hi2s3) != HAL_OK) {
            return -EIO;
        }
    }
    s_state = DRV_CODEC_STATE_READY;
    return 0;
}

uint32_t Drv_Codec_TxDmaRemaining(void) {
    DMA_HandleTypeDef *hdma;

    if (s_state != DRV_CODEC_STATE_RUNNING) {
        return 0u;
    }
    hdma = hi2s3.hdmatx;
    if (hdma == 0 || hdma->Instance == 0) {
        return 0u;
    }
    /* Two 32-bit words per stereo frame; NDTR is a single atomic read. */
    return (uint32_t)((DMA_Stream_TypeDef *)hdma->Instance)->NDTR >> 1u;
}

void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s == &hi2s3) {
        BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_CODEC, prof_start);
        codec_block_irq(0u);
        BSP_PROF_ISR_END(BSP_PROF_ISR_CODEC, prof_start);
    }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s == &hi2s3) {
        BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_CODEC, prof_start);
        codec_block_irq(1u);
        BSP_PROF_ISR_END(BSP_PROF_ISR_CODEC, prof_start);
    }
}

/* No HAL_I2SEx_TxRxErrorCallback in this HAL: full-duplex DMA errors come
 * through I2S_DMAError -> HAL_I2S_ErrorCallback. */
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s) {
    if (hi2s != &hi2s3) {
        return;
    }
    s_state = DRV_CODEC_STATE_ERROR;
    (void)HAL_I2S_DMAStop(hi2s);
    LOG_ERROR("i2s3 dma error 0x%02X", (unsigned)hi2s->ErrorCode);
}
