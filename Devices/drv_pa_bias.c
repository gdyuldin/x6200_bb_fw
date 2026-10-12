#include "drv_pa_bias.h"

#include "bsp_irq.h"
#include "bsp_log.h"
#include "dac.h"

#define LOG_TAG "drv.pa_bias"

#define PA_BIAS_DAC_CH_DRIVE DAC_CHANNEL_1 /* PA4, calib 0x119 */
#define PA_BIAS_DAC_CH_FINAL DAC_CHANNEL_2 /* PA5, calib 0x11b */
#define PA_BIAS_DAC_ALIGN    DAC_ALIGN_12B_R
#define PA_BIAS_DAC_MAX      0x0FFFu

static bool     s_initialized;
static bool     s_enabled;
static uint8_t  s_hw_rev;
static bool     s_drive_off;
static bool     s_final_off;
static uint16_t s_bias_drive;
static uint16_t s_bias_final;
static int16_t  s_drive_offset;
static int16_t  s_final_offset;
/* base + offset, clamped to the DAC span; diagnostic/compare value. */
static uint16_t s_drive_applied;
static uint16_t s_final_applied;

static uint32_t dac_code(uint16_t value) {
    /* Keep the value within the DAC register width. */
    return (uint32_t)(value & PA_BIAS_DAC_MAX);
}

static uint16_t clamped_value(int32_t value) {
    if (value < 0) {
        value = 0;
    } else if (value > (int32_t)PA_BIAS_DAC_MAX) {
        value = (int32_t)PA_BIAS_DAC_MAX;
    }
    return (uint16_t)value;
}

static uint16_t drive_value(void) {
    return clamped_value((int32_t)s_bias_drive + (int32_t)s_drive_offset);
}

static uint16_t final_value(void) {
    return clamped_value((int32_t)s_bias_final + (int32_t)s_final_offset);
}

static int write_channel(uint32_t channel, uint16_t value) {
    if (!s_enabled) {
        return -1;
    }
    if (HAL_DAC_SetValue(&hdac1, channel, PA_BIAS_DAC_ALIGN, dac_code(value)) !=
        HAL_OK) {
        LOG_ERROR("DAC1 ch%lu write failed", (unsigned long)channel);
        return -1;
    }
    return 0;
}

static int apply_bias(void) {
    int rc;

    s_drive_applied = drive_value();
    rc = write_channel(PA_BIAS_DAC_CH_DRIVE,
                       s_drive_off ? 0u : s_drive_applied);
    s_final_applied = final_value();
    if (write_channel(PA_BIAS_DAC_CH_FINAL,
                      s_final_off ? 0u : s_final_applied) != 0) {
        rc = -1;
    }
    return rc;
}

int Drv_PaBias_Init(uint8_t hw_rev, uint16_t bias_drive, uint16_t bias_final) {
    if (s_initialized) {
        return 0;
    }

    s_initialized = true;
    s_hw_rev      = hw_rev;
    s_bias_drive    = bias_drive;
    s_bias_final    = bias_final;
    s_drive_offset  = 0;
    s_final_offset  = 0;
    s_drive_applied = drive_value();
    s_final_applied = final_value();
    s_drive_off     = false;
    s_final_off     = false;
    s_enabled       = (hw_rev == 0u);

    if (!s_enabled) {
        LOG_WARN("PA bias disabled: hw rev %u, DAC writes prohibited",
                 (unsigned)hw_rev);
        return 0;
    }

    /* Preload both holding registers, then enable the channels (value first,
     * enable second). */
    if (HAL_DAC_SetValue(&hdac1, PA_BIAS_DAC_CH_DRIVE, PA_BIAS_DAC_ALIGN,
                         dac_code(s_drive_applied)) != HAL_OK ||
        HAL_DAC_SetValue(&hdac1, PA_BIAS_DAC_CH_FINAL, PA_BIAS_DAC_ALIGN,
                         dac_code(s_final_applied)) != HAL_OK) {
        LOG_ERROR("PA bias DAC preload failed");
        s_enabled = false;
        return -1;
    }

    HAL_StatusTypeDef rc_drive = HAL_DAC_Start(&hdac1, PA_BIAS_DAC_CH_DRIVE);
    HAL_StatusTypeDef rc_final = HAL_DAC_Start(&hdac1, PA_BIAS_DAC_CH_FINAL);
    if (rc_drive != HAL_OK || rc_final != HAL_OK) {
        LOG_ERROR("PA bias DAC start failed (%d/%d)", (int)rc_drive,
                  (int)rc_final);
        if (rc_drive == HAL_OK) {
            (void)HAL_DAC_Stop(&hdac1, PA_BIAS_DAC_CH_DRIVE);
        }
        if (rc_final == HAL_OK) {
            (void)HAL_DAC_Stop(&hdac1, PA_BIAS_DAC_CH_FINAL);
        }
        s_enabled = false;
        return -1;
    }

    LOG_INFO("PA bias drive %u final %u", (unsigned)s_bias_drive,
             (unsigned)s_bias_final);
    return 0;
}

int Drv_PaBias_SetBias(uint16_t bias_drive, uint16_t bias_final) {
    uint32_t key = Bsp_Irq_SaveLow();
    int      rc;

    s_bias_drive = bias_drive;
    s_bias_final = bias_final;

    if (!s_initialized) {
        Bsp_Irq_RestoreLow(key);
        LOG_ERROR("PA bias not initialized");
        return -1;
    }
    if (!s_enabled) {
        Bsp_Irq_RestoreLow(key);
        LOG_WARN("PA bias set refused: hw rev %u, DAC writes prohibited",
                 (unsigned)s_hw_rev);
        return -1;
    }
    rc = apply_bias();
    Bsp_Irq_RestoreLow(key);
    return rc;
}

int Drv_PaBias_SetOff(bool drive_off, bool final_off) {
    uint32_t key = Bsp_Irq_SaveLow();
    bool     changed = (s_drive_off != drive_off) || (s_final_off != final_off);
    int      rc;

    s_drive_off = drive_off;
    s_final_off = final_off;

    if (!s_initialized) {
        Bsp_Irq_RestoreLow(key);
        LOG_ERROR("PA bias not initialized");
        return -1;
    }
    if (!s_enabled) {
        Bsp_Irq_RestoreLow(key);
        if (changed) {
            LOG_WARN("PA bias OFF refused: hw rev %u, DAC writes prohibited",
                     (unsigned)s_hw_rev);
        }
        return -1;
    }
    if (!changed) {
        Bsp_Irq_RestoreLow(key);
        return 0;
    }
    rc = apply_bias();
    Bsp_Irq_RestoreLow(key);
    LOG_INFO("PA bias drive %s final %s", drive_off ? "OFF" : "ON",
             final_off ? "OFF" : "ON");
    return rc;
}

int Drv_PaBias_SetOffset(int16_t drive_offset, int16_t final_offset) {
    uint32_t key;
    int      rc = 0;

    /* A positive offset would raise bias above the calibrated base. */
    if (drive_offset > 0) {
        drive_offset = 0;
    }
    if (final_offset > 0) {
        final_offset = 0;
    }

    key = Bsp_Irq_SaveLow();
    if (!s_initialized || !s_enabled) {
        Bsp_Irq_RestoreLow(key);
        return -1;
    }
    if (s_drive_offset == drive_offset && s_final_offset == final_offset) {
        Bsp_Irq_RestoreLow(key);
        return 0;
    }

    s_drive_offset  = drive_offset;
    s_final_offset  = final_offset;
    s_drive_applied = drive_value();
    s_final_applied = final_value();
    if (!s_drive_off &&
        HAL_DAC_SetValue(&hdac1, PA_BIAS_DAC_CH_DRIVE, PA_BIAS_DAC_ALIGN,
                         dac_code(s_drive_applied)) != HAL_OK) {
        rc = -1;
    }
    if (!s_final_off &&
        HAL_DAC_SetValue(&hdac1, PA_BIAS_DAC_CH_FINAL, PA_BIAS_DAC_ALIGN,
                         dac_code(s_final_applied)) != HAL_OK) {
        rc = -1;
    }
    Bsp_Irq_RestoreLow(key);
    return rc;
}

bool Drv_PaBias_IsEnabled(void) {
    return s_enabled;
}
