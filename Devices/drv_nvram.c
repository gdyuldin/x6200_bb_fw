#include "drv_nvram.h"

#include "drv_eeprom.h"

_Static_assert(DRV_NVRAM_SETTINGS_OFFSET + DRV_NVRAM_SETTINGS_LEN <=
                   DRV_NVRAM_CALIB_OFFSET,
               "settings region overlaps calibration");
_Static_assert(DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_LEN <=
                   DRV_NVRAM_NVPARAM_OFFSET,
               "calibration region overlaps NVPARAM");
_Static_assert(DRV_NVRAM_NVPARAM_OFFSET + DRV_NVRAM_NVPARAM_LEN <=
                   DRV_EEPROM_SIZE,
               "NVPARAM region exceeds the device");
_Static_assert(DRV_NVRAM_CALIB_MARK_OFFSET < DRV_NVRAM_CALIB_LEN,
               "calibration marker lies outside the region");
_Static_assert(DRV_NVRAM_CALIB_VER_OFFSET < DRV_NVRAM_CALIB_LEN,
               "calibration version marker lies outside the region");

uint8_t Drv_Nvram_Crc8(const uint8_t *data, size_t len, uint8_t init) {
    uint8_t crc = init;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; ++bit) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1) ^ DRV_NVRAM_CRC8_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

int Drv_Nvram_SettingsRead(uint8_t *buf) {
    if (buf == 0) {
        return DRV_NVRAM_ERR_ARG;
    }
    const int rc = Drv_Eeprom_Read(DRV_NVRAM_SETTINGS_OFFSET, buf,
                                   DRV_NVRAM_SETTINGS_LEN);
    return rc == 0 ? 0 : DRV_NVRAM_ERR_BUS;
}

int Drv_Nvram_CalibRead(DrvNvramCalib *calib) {
    if (calib == 0) {
        return DRV_NVRAM_ERR_ARG;
    }
    const int rc = Drv_Eeprom_Read(DRV_NVRAM_CALIB_OFFSET, (uint8_t *)calib,
                                   DRV_NVRAM_CALIB_LEN);
    if (rc != 0) {
        return DRV_NVRAM_ERR_BUS;
    }
    /* The markers carry the version; a mismatch means the block is not a
     * valid per-unit calibration image. */
    if (calib->version_marker != DRV_NVRAM_CALIB_MARK ||
        calib->version != DRV_NVRAM_CALIB_MARK) {
        return DRV_NVRAM_ERR_MARKER;
    }
    return 0;
}

int Drv_Nvram_CalibSave(const DrvNvramCalib *calib) {
    if (calib == 0) {
        return DRV_NVRAM_ERR_ARG;
    }

    /* Stamp the markers on a copy so the caller's block is untouched. */
    DrvNvramCalib tmp = *calib;
    tmp.version_marker = (uint8_t)DRV_NVRAM_CALIB_MARK;
    tmp.version        = (uint8_t)DRV_NVRAM_CALIB_MARK;

    const int rc =
        Drv_Eeprom_Write(DRV_NVRAM_CALIB_OFFSET, (const uint8_t *)&tmp,
                         DRV_NVRAM_CALIB_LEN);
    return rc == 0 ? 0 : DRV_NVRAM_ERR_BUS;
}

int Drv_Nvram_NvparamRead(DrvNvramNvparam *rec) {
    if (rec == 0) {
        return DRV_NVRAM_ERR_ARG;
    }

    const int rc = Drv_Eeprom_Read(DRV_NVRAM_NVPARAM_OFFSET, (uint8_t *)rec,
                                   DRV_NVRAM_NVPARAM_LEN);
    if (rc != 0) {
        return DRV_NVRAM_ERR_BUS;
    }
    if (rec->magic != DRV_NVRAM_NVPARAM_HDR) {
        return DRV_NVRAM_ERR_HEADER;
    }
    if (rec->crc8 != Drv_Nvram_Crc8((const uint8_t *)rec, 2, 0)) {
        return DRV_NVRAM_ERR_CRC;
    }
    return 0;
}

int Drv_Nvram_NvparamSave(uint8_t chge) {
    DrvNvramNvparam rec;
    rec.magic = (uint8_t)DRV_NVRAM_NVPARAM_HDR;
    rec.chge  = chge;
    rec.crc8  = Drv_Nvram_Crc8((const uint8_t *)&rec, 2, 0);

    const int rc =
        Drv_Eeprom_Write(DRV_NVRAM_NVPARAM_OFFSET, (const uint8_t *)&rec,
                         DRV_NVRAM_NVPARAM_LEN);
    return rc == 0 ? 0 : DRV_NVRAM_ERR_BUS;
}
