#include "drv_clockgen.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include "bsp_i2c_bus.h"
#include "bsp_log.h"
#include "bsp_time.h"

#define LOG_TAG "drv.clockgen"

/*
 * VersaClock 6E RAM programming per the 5P49V6965 datasheet and the
 * VersaClock 6E Register Descriptions and Programming Guide. Field values
 * are computed from the guide formulas; the Trim RAM 0x01-0x0F, the test
 * RAM 0x6A-0x6F and factory-reserved fields are left untouched (writing
 * 0x11 test_mode_vco_band would kill VCO auto-calibration).
 */

/* RAM registers (guide Table 4). */
#define CLOCKGEN_REG_OTP_CTRL   0x00u
#define CLOCKGEN_REG_SRC_SD     0x10u
#define CLOCKGEN_REG_REF_DIV    0x15u
#define CLOCKGEN_REG_VCO_CTRL   0x16u
#define CLOCKGEN_REG_FB_INT_HI  0x17u
#define CLOCKGEN_REG_FB_INT_LO  0x18u
#define CLOCKGEN_REG_FB_FRAC0   0x19u
#define CLOCKGEN_REG_FB_FRAC1   0x1Au
#define CLOCKGEN_REG_FB_FRAC2   0x1Bu
#define CLOCKGEN_REG_VCO_CAL    0x1Cu
#define CLOCKGEN_REG_FOD1_CTRL  0x21u
#define CLOCKGEN_REG_OD1_OFF0   0x22u
#define CLOCKGEN_REG_OD1_OFF1   0x23u
#define CLOCKGEN_REG_OD1_OFF2   0x24u
#define CLOCKGEN_REG_OD1_OFF3   0x25u
#define CLOCKGEN_REG_OD1_INT_HI 0x2Du
#define CLOCKGEN_REG_OD1_INT_LO 0x2Eu
#define CLOCKGEN_REG_FOD2_CTRL  0x31u
#define CLOCKGEN_REG_FOD3_CTRL  0x41u
#define CLOCKGEN_REG_FOD4_CTRL  0x51u
#define CLOCKGEN_REG_OUT1_CFG   0x60u
#define CLOCKGEN_REG_OUT1_CTRL  0x61u
#define CLOCKGEN_REG_OUT2_CTRL  0x63u
#define CLOCKGEN_REG_OUT3_CTRL  0x65u
#define CLOCKGEN_REG_OUT4_CTRL  0x67u
#define CLOCKGEN_REG_CLK_OE     0x68u
#define CLOCKGEN_REG_CLK_OS     0x69u
#define CLOCKGEN_REG_GLOBAL_RST 0x76u

/* Table 25: source/shutdown, functional bits only (D5/D4 reserved). */
#define CLOCKGEN_SRC_MASK      0xCFu
#define CLOCKGEN_SRC_EN_XTAL   (1u << 7)

/* Tables 31/32: prediv = 2 via Sel_prediv2 with Bypass_prediv cleared. */
#define CLOCKGEN_REFDIV_SEL_2    (1u << 7)
#define CLOCKGEN_VCO_BYPASS_PREDIV (1u << 7)

/* Tables 47-50: FOD control, functional bits D7 and D3:0 (mode nibble). */
#define CLOCKGEN_FOD_MASK      0x8Fu
#define CLOCKGEN_FOD_RESETB    (1u << 7)
#define CLOCKGEN_FOD_EN        (1u << 0)

/* Tables 35/52: integer divider low registers keep the reserved nibble. */
#define CLOCKGEN_INT_LO_MASK   0xF0u

/* Table 62: 0x25 keeps only the reserved D0. */
#define CLOCKGEN_OD1_OFF3_MASK 0xFEu

/* Tables 108-115: output driver config. */
#define CLOCKGEN_OUT_CFG_MASK  0xF3u
#define CLOCKGEN_OUT_CFG_LVDS  (0x3u << 5)
#define CLOCKGEN_OUT_PWR_3V3   (0x3u << 3)
#define CLOCKGEN_OUT_SLEW_1X   0x3u
#define CLOCKGEN_OUT_CTRL_MASK 0x07u
#define CLOCKGEN_OUT_CTRL_EN   (1u << 0)

/*
 * Tables 24/26/27: OE/OS. The OS bits are active-low suspend (truth table:
 * OSn = 0 tri-states the output whatever OE does), so only CLK1_OS stays
 * high and the unused outputs are suspended. D1:0 of 0x69 are reserved.
 */
#define CLOCKGEN_CLK_OE_MASK   0xFFu
#define CLOCKGEN_CLK_OS_MASK   0xFCu
#define CLOCKGEN_CLK_OS1       (1u << 6)

/* Table 39 and the skew-apply example: apply triggers. */
#define CLOCKGEN_CAL_START     (1u << 7)
#define CLOCKGEN_GLOBAL_RST    (1u << 5)

/* F_VCO = F_REF / prediv * M (guide PLL section). */
#define CLOCKGEN_VCO_HZ \
    (DRV_CLOCKGEN_FREF_HZ / DRV_CLOCKGEN_PLL_PREDIV * DRV_CLOCKGEN_PLL_FB_INT)
#define CLOCKGEN_VCO_MIN_HZ 2500000000u
#define CLOCKGEN_VCO_MAX_HZ 2900000000u

/* OD1 = INT + FRAC = (F_VCO / 2) / F_OUT1 (guide Eq.1); the FRAC part is
 * stored in the 30-bit OD1_offset field as FRAC * 2^24 (Eq.2/10, SSCE=0). */
#define CLOCKGEN_FB_INT_HI    ((uint8_t)(DRV_CLOCKGEN_PLL_FB_INT >> 4))
#define CLOCKGEN_FB_INT_LO    ((uint8_t)((DRV_CLOCKGEN_PLL_FB_INT & 0x0Fu) << 4))
#define CLOCKGEN_OD1_INT_HI   ((uint8_t)(DRV_CLOCKGEN_FOD1_N_INT >> 4))
#define CLOCKGEN_OD1_INT_LO   ((uint8_t)((DRV_CLOCKGEN_FOD1_N_INT & 0x0Fu) << 4))
#define CLOCKGEN_OD1_OFF_29_22 ((uint8_t)(DRV_CLOCKGEN_FOD1_N_FRAC >> 22))
#define CLOCKGEN_OD1_OFF_21_14 ((uint8_t)(DRV_CLOCKGEN_FOD1_N_FRAC >> 14))
#define CLOCKGEN_OD1_OFF_13_6  ((uint8_t)(DRV_CLOCKGEN_FOD1_N_FRAC >> 6))
#define CLOCKGEN_OD1_OFF_5_0   ((uint8_t)((DRV_CLOCKGEN_FOD1_N_FRAC & 0x3Fu) << 2))

_Static_assert(CLOCKGEN_VCO_HZ >= CLOCKGEN_VCO_MIN_HZ &&
                   CLOCKGEN_VCO_HZ <= CLOCKGEN_VCO_MAX_HZ,
               "VCO frequency outside the 2500-2900 MHz window");

typedef struct {
    uint8_t reg;
    uint8_t val;
    uint8_t mask;
} DrvClockGenField;

/*
 * Final functional values. mask names the functional bits of the register;
 * everything outside it is factory-reserved and preserved on write. Order:
 * unused channels off first, then the PLL, FOD1 and the OUT1 driver.
 */
static const DrvClockGenField s_config[] = {
    /* Unused FODs in "not used" mode (guide disable pattern 0x80). */
    { CLOCKGEN_REG_FOD2_CTRL, CLOCKGEN_FOD_RESETB, CLOCKGEN_FOD_MASK },
    { CLOCKGEN_REG_FOD3_CTRL, CLOCKGEN_FOD_RESETB, CLOCKGEN_FOD_MASK },
    { CLOCKGEN_REG_FOD4_CTRL, CLOCKGEN_FOD_RESETB, CLOCKGEN_FOD_MASK },
    /* Unused output buffers off (DST: unused drivers to Hi-Z). */
    { CLOCKGEN_REG_OUT2_CTRL, 0x00u, CLOCKGEN_OUT_CTRL_MASK },
    { CLOCKGEN_REG_OUT3_CTRL, 0x00u, CLOCKGEN_OUT_CTRL_MASK },
    { CLOCKGEN_REG_OUT4_CTRL, 0x00u, CLOCKGEN_OUT_CTRL_MASK },
    /* Crystal on, CLKIN off, no doubler, OUT1 fed by FOD1 (Table 25). */
    { CLOCKGEN_REG_SRC_SD, CLOCKGEN_SRC_EN_XTAL, CLOCKGEN_SRC_MASK },
    /* Prediv 2, M = 222, no feedback fraction (Tables 31-38). */
    { CLOCKGEN_REG_REF_DIV, CLOCKGEN_REFDIV_SEL_2, 0xFFu },
    { CLOCKGEN_REG_VCO_CTRL, 0x00u, CLOCKGEN_VCO_BYPASS_PREDIV },
    { CLOCKGEN_REG_FB_INT_HI, CLOCKGEN_FB_INT_HI, 0xFFu },
    { CLOCKGEN_REG_FB_INT_LO, CLOCKGEN_FB_INT_LO, CLOCKGEN_INT_LO_MASK },
    { CLOCKGEN_REG_FB_FRAC0, 0x00u, 0xFFu },
    { CLOCKGEN_REG_FB_FRAC1, 0x00u, 0xFFu },
    { CLOCKGEN_REG_FB_FRAC2, 0x00u, 0xFFu },
    /* FOD1 from PLL, fractional (mode 0001, Table 47). */
    { CLOCKGEN_REG_FOD1_CTRL,
      CLOCKGEN_FOD_RESETB | CLOCKGEN_FOD_EN, CLOCKGEN_FOD_MASK },
    /* OD1 = 11 + 0x0BE4000/2^24 (Tables 51/52, 59-62), spread off. */
    { CLOCKGEN_REG_OD1_OFF0, CLOCKGEN_OD1_OFF_29_22, 0xFFu },
    { CLOCKGEN_REG_OD1_OFF1, CLOCKGEN_OD1_OFF_21_14, 0xFFu },
    { CLOCKGEN_REG_OD1_OFF2, CLOCKGEN_OD1_OFF_13_6, 0xFFu },
    { CLOCKGEN_REG_OD1_OFF3, CLOCKGEN_OD1_OFF_5_0, CLOCKGEN_OD1_OFF3_MASK },
    { CLOCKGEN_REG_OD1_INT_HI, CLOCKGEN_OD1_INT_HI, 0xFFu },
    { CLOCKGEN_REG_OD1_INT_LO, CLOCKGEN_OD1_INT_LO, CLOCKGEN_INT_LO_MASK },
    /* OUT1: LVDS, 3.3 V, slew 1x, buffer on (Tables 108/109). */
    { CLOCKGEN_REG_OUT1_CFG,
      CLOCKGEN_OUT_CFG_LVDS | CLOCKGEN_OUT_PWR_3V3 | CLOCKGEN_OUT_SLEW_1X,
      CLOCKGEN_OUT_CFG_MASK },
    { CLOCKGEN_REG_OUT1_CTRL, CLOCKGEN_OUT_CTRL_EN, CLOCKGEN_OUT_CTRL_MASK },
    /* All OE pins forced on; only CLK1 not suspended (Tables 24/26/27). */
    { CLOCKGEN_REG_CLK_OE, 0xFFu, CLOCKGEN_CLK_OE_MASK },
    { CLOCKGEN_REG_CLK_OS, CLOCKGEN_CLK_OS1, CLOCKGEN_CLK_OS_MASK },
};

static uint8_t s_addr7;

static int clockgen_write_reg(uint8_t reg, uint8_t val) {
    const uint8_t buf[2] = { reg, val };
    return Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, s_addr7, buf, sizeof(buf),
                                DRV_CLOCKGEN_XFER_TIMEOUT_MS);
}

static int clockgen_read_reg(uint8_t reg, uint8_t *val) {
    return Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_1, s_addr7, &reg, 1, val, 1,
                                    DRV_CLOCKGEN_XFER_TIMEOUT_MS);
}

/* Read-modify-write: keeps factory-reserved bits at their live values. */
static int clockgen_update_reg(uint8_t reg, uint8_t set, uint8_t clear) {
    uint8_t val = 0;
    int     rc = clockgen_read_reg(reg, &val);
    if (rc != 0) {
        return rc;
    }
    val = (uint8_t)((val | set) & (uint8_t)~clear);
    return clockgen_write_reg(reg, val);
}

static int clockgen_write_field(const DrvClockGenField *f) {
    if (f->mask == 0xFFu) {
        return clockgen_write_reg(f->reg, f->val);
    }
    uint8_t val = 0;
    int     rc = clockgen_read_reg(f->reg, &val);
    if (rc != 0) {
        return rc;
    }
    val = (uint8_t)((val & (uint8_t)~f->mask) | (f->val & f->mask));
    return clockgen_write_reg(f->reg, val);
}

static int clockgen_probe(void) {
    static const uint8_t addrs[] = { DRV_CLOCKGEN_ADDR7, DRV_CLOCKGEN_ADDR7_ALT };
    const uint8_t           reg = CLOCKGEN_REG_OTP_CTRL;

    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); ++i) {
        uint8_t val = 0;
        if (Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_1, addrs[i], &reg, 1, &val, 1,
                                     DRV_CLOCKGEN_XFER_TIMEOUT_MS) == 0) {
            s_addr7 = addrs[i];
            LOG_DEBUG("found at 0x%02X", (unsigned)s_addr7);
            return 0;
        }
    }
    return -ENODEV;
}

/*
 * Apply sequence from the guide (skew-apply example + Table 39): force a
 * VCO band calibration, then restart the output counters through the I2C
 * global reset so the new dividers land on a common edge.
 */
static int clockgen_apply(void) {
    int rc = clockgen_update_reg(CLOCKGEN_REG_VCO_CAL, 0, CLOCKGEN_CAL_START);
    if (rc != 0) {
        return rc;
    }
    rc = clockgen_update_reg(CLOCKGEN_REG_VCO_CAL, CLOCKGEN_CAL_START, 0);
    if (rc != 0) {
        return rc;
    }
    rc = clockgen_update_reg(CLOCKGEN_REG_GLOBAL_RST, 0, CLOCKGEN_GLOBAL_RST);
    if (rc != 0) {
        return rc;
    }
    return clockgen_update_reg(CLOCKGEN_REG_GLOBAL_RST, CLOCKGEN_GLOBAL_RST, 0);
}

static int clockgen_verify(void) {
    for (size_t i = 0; i < sizeof(s_config) / sizeof(s_config[0]); ++i) {
        const DrvClockGenField *f = &s_config[i];
        uint8_t                 val = 0;
        int                     rc = clockgen_read_reg(f->reg, &val);
        if (rc != 0) {
            return rc;
        }
        if ((uint8_t)(val & f->mask) != (uint8_t)(f->val & f->mask)) {
            LOG_WARN("verify mismatch reg 0x%02X got 0x%02X want 0x%02X",
                     (unsigned)f->reg, (unsigned)val,
                     (unsigned)(uint8_t)(f->val & f->mask));
            return -EIO;
        }
    }
    return 0;
}

#if BSP_LOG_MAX_LEVEL >= BSP_LOG_TRACE
static void clockgen_trace_dump(void) {
    for (unsigned base = 0x00u; base < 0x100u; base += 8u) {
        uint8_t reg = (uint8_t)base;
        uint8_t buf[8];
        if (Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_1, s_addr7, &reg, 1, buf,
                                     sizeof(buf),
                                     DRV_CLOCKGEN_XFER_TIMEOUT_MS) != 0) {
            return;
        }
        LOG_TRACE("%02X: %02X %02X %02X %02X %02X %02X %02X %02X", base,
                  (unsigned)buf[0], (unsigned)buf[1], (unsigned)buf[2],
                  (unsigned)buf[3], (unsigned)buf[4], (unsigned)buf[5],
                  (unsigned)buf[6], (unsigned)buf[7]);
    }
}
#endif

int Drv_ClockGen_Init(void) {
    int rc = clockgen_probe();
    if (rc != 0) {
        LOG_WARN("clock gen not found, continuing");
        return rc;
    }

    for (size_t i = 0; i < sizeof(s_config) / sizeof(s_config[0]); ++i) {
        rc = clockgen_write_field(&s_config[i]);
        if (rc != 0) {
            LOG_DEBUG("write reg 0x%02X failed (%d)",
                      (unsigned)s_config[i].reg, rc);
            return rc;
        }
    }

    rc = clockgen_apply();
    if (rc != 0) {
        LOG_DEBUG("apply failed (%d)", rc);
        return rc;
    }
    /* DST Table 6: PLL lock time from shutdown, max 4 ms. */
    Bsp_Time_DelayMs(5u);

    rc = clockgen_verify();
    if (rc != 0) {
        return rc;
    }
    LOG_DEBUG("clock gen config verified");

#if BSP_LOG_MAX_LEVEL >= BSP_LOG_TRACE
    clockgen_trace_dump();
#endif

    LOG_INFO("out1 %u Hz lvds", (unsigned)DRV_CLOCKGEN_OUT1_HZ);
    return 0;
}
