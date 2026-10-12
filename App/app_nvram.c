#include "app_nvram.h"

#include <string.h>

#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_memory.h"
#include "drv_nvram.h"

#define LOG_TAG "app.nvram"

/* The host bias command updates both fields in one four-byte write. */
_Static_assert(offsetof(DrvNvramCalib, bias_final) ==
                   offsetof(DrvNvramCalib, bias_drive) + sizeof(uint16_t),
               "bias_drive and bias_final must be contiguous");

/*
 * Boot read sequence: settings, then calibration, then NVPARAM. The settings
 * read is a plain EEPROM read.
 */

static uint8_t s_settings[DRV_NVRAM_SETTINGS_LEN] BSP_RAM_D1_BSS;
static DrvNvramCalib s_calib BSP_RAM_D1_BSS;
/* Written from the host-link main-loop task, read from the TIM6 power-off ISR. */
static volatile bool s_charge_enabled = true;

const DrvNvramCalib *App_Nvram_Calib(void) {
    return &s_calib;
}

bool App_Nvram_TxAllowed(uint32_t freq_hz) {
    if (s_calib.fullband_txe != 0u) {
        return true;
    }

    for (size_t i = 0; i < sizeof(s_calib.bands) / sizeof(s_calib.bands[0]); i++) {
        const DrvNvramCalibBand *band = &s_calib.bands[i];
        if (band->txe != 0u && (uint64_t)freq_hz >= band->lo &&
            (uint64_t)freq_hz <= band->hi) {
            return true;
        }
    }
    return false;
}

bool App_Nvram_ChargeEnabled(void) {
    return s_charge_enabled;
}

void App_Nvram_SetChargeEnabled(bool enabled) {
    s_charge_enabled = enabled;
}

int App_Nvram_ChargeSave(void) {
    const uint8_t chge = s_charge_enabled
                             ? DRV_NVRAM_NVPARAM_CHGE_ENABLED
                             : DRV_NVRAM_NVPARAM_CHGE_DISABLED;
    return Drv_Nvram_NvparamSave(chge);
}

int App_Nvram_CalibWrite(uint16_t off, const uint8_t *src, size_t len) {
    if (src == 0 || len == 0u) {
        return -1;
    }
    if ((size_t)off + len > sizeof(s_calib)) {
        return -1;
    }

    uint32_t basepri = Bsp_Irq_SaveLow();
    memcpy((uint8_t *)&s_calib + off, src, len);
    Bsp_Irq_RestoreLow(basepri);
    return 0;
}

int App_Nvram_CalibSetBias(uint16_t bias_drive, uint16_t bias_final) {
    /* bias_drive at 0x119 and bias_final at 0x11b are contiguous. Build the
     * four EEPROM bytes explicitly so the image stays little-endian regardless
     * of the host-side packing. */
    uint8_t raw[4] = {
        (uint8_t)(bias_drive & 0xFFu),
        (uint8_t)(bias_drive >> 8),
        (uint8_t)(bias_final & 0xFFu),
        (uint8_t)(bias_final >> 8),
    };

    return App_Nvram_CalibWrite(
        (uint16_t)offsetof(DrvNvramCalib, bias_drive), raw, sizeof(raw));
}

const DrvFpgaCalib *App_Nvram_FpgaCalib(void) {
    return &s_calib.fpga_calib;
}

int App_Nvram_Init(void) {
    int first_rc = 0;

    int rc = Drv_Nvram_SettingsRead(s_settings);
    if (rc == 0) {
        LOG_INFO("settings ok");
    } else {
        LOG_ERROR("settings read error %d", rc);
        first_rc = rc;
    }

    rc = Drv_Nvram_CalibRead(&s_calib);
    if (rc == 0) {
        LOG_INFO("calib ok");
    } else if (rc == DRV_NVRAM_ERR_MARKER) {
        /* No valid per-unit calibration and no defaults available, so the
         * block is zeroed to keep the FPGA frontend on safe values instead of
         * uninitialized bytes. */
        s_calib = (DrvNvramCalib){ 0 };
        LOG_WARN("calib markers invalid, zeroed");
        if (first_rc == 0) {
            first_rc = rc;
        }
    } else {
        LOG_ERROR("calib read error %d", rc);
        if (first_rc == 0) {
            first_rc = rc;
        }
    }

    DrvNvramNvparam rec;
    rc = Drv_Nvram_NvparamRead(&rec);
    if (rc == 0) {
        s_charge_enabled = rec.chge != DRV_NVRAM_NVPARAM_CHGE_DISABLED;
        LOG_INFO("nvparam chge=%u", (unsigned)rec.chge);
    } else {
        /* Invalid/absent NVPARAM: leave charging enabled and do not rewrite the
         * EEPROM; a later power-off save replaces the record. */
        s_charge_enabled = true;
        LOG_ERROR("nvparam read error %d, defaulting chge=1", rc);
        if (first_rc == 0) {
            first_rc = rc;
        }
    }

    return first_rc;
}
