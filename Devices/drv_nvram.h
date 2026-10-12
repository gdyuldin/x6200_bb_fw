#pragma once

#include <stddef.h>
#include <stdint.h>

#include "drv_fpga.h"

/*
 * Regional NVRAM API on top of the external AT24C256C EEPROM (I2C1).
 *
 * Region map:
 *   settings 0x0000 / 0x7b0 : read only (no writer; persisted to backup SRAM)
 *   calib    0x1000 / 0x1fb : read + save, markers calib[0x1f9] and [0x1fa] = 'd'
 *   NVPARAM  0x2000 / 3     : {0x3e, chge, crc8}
 *
 * The calib and NVPARAM regions are exposed as packed structs whose layout
 * matches the EEPROM image byte-for-byte. The settings region stays an opaque
 * byte buffer: its layout is not part of this contract yet. The structs are the
 * single definition of the region layout; do not add padding or reorder fields.
 *
 * The NVPARAM CRC-8 uses polynomial 0x31 (x^8+x^5+x^4+1),
 * init 0, non-reflected, xorout 0, computed over bytes [0..1] and stored in
 * byte [2]. This module owns the single portable implementation (Drv_Nvram_Crc8)
 * used by both the read and save paths; the hardware CRC block is not used.
 *
 * Return codes: 0 success; negative error codes below. Call only from
 * housekeeping context (the save paths block on the EEPROM write cycle).
 */

#define DRV_NVRAM_SETTINGS_OFFSET 0x0000u
#define DRV_NVRAM_SETTINGS_LEN    0x7b0u

#define DRV_NVRAM_CALIB_OFFSET      0x1000u
#define DRV_NVRAM_CALIB_LEN         0x1fbu
#define DRV_NVRAM_CALIB_MARK_OFFSET 0x1f9u
#define DRV_NVRAM_CALIB_VER_OFFSET  0x1fau
#define DRV_NVRAM_CALIB_MARK        0x64u /* 'd' */

#define DRV_NVRAM_NVPARAM_OFFSET        0x2000u
#define DRV_NVRAM_NVPARAM_LEN           3u
#define DRV_NVRAM_NVPARAM_HDR           0x3eu
#define DRV_NVRAM_NVPARAM_CHGE_DISABLED 0u
#define DRV_NVRAM_NVPARAM_CHGE_ENABLED  1u
#define DRV_NVRAM_CRC8_POLY             0x31u

#define DRV_NVRAM_ERR_ARG        (-1)
#define DRV_NVRAM_ERR_BUS        (-2)
#define DRV_NVRAM_ERR_HEADER     (-3)
#define DRV_NVRAM_ERR_CRC        (-4)
#define DRV_NVRAM_ERR_MARKER     (-5)

/* NVPARAM record: magic, chge, CRC-8 over the first two bytes. */
typedef struct __attribute__((packed)) {
    uint8_t magic; /* 0x00: DRV_NVRAM_NVPARAM_HDR */
    uint8_t chge;  /* 0x01: charger enable, 0 = disabled, 1 = enabled */
    uint8_t crc8;  /* 0x02: CRC-8 over {magic, chge} */
} DrvNvramNvparam;

/* One TX band from the calibration table. */
typedef struct __attribute__((packed)) {
    uint64_t lo;  /* 0x00: lower TX edge, Hz (little-endian) */
    uint64_t hi;  /* 0x08: upper TX edge, Hz (little-endian) */
    uint32_t txe; /* 0x10: TX enable for the band; 0 = disabled */
} DrvNvramCalibBand;

/*
 * Calibration block. Field order and sizes are the EEPROM image; the FPGA
 * per-unit bytes are embedded as DrvFpgaCalib (see drv_fpga.h) so that type
 * stays owned by the FPGA driver. Reserved spans keep the layout aligned to
 * the contract; do not remove or resize them.
 */
typedef struct __attribute__((packed)) {
    uint8_t reserved_00[0x8f];

    int16_t smeter_offset_hf;  /* 0x08f */
    int16_t smeter_offset_vhf; /* 0x091 */

    uint8_t reserved_93[0x22];

    DrvFpgaCalib fpga_calib; /* 0x0b5: FPGA per-unit block (mode/band + frame calib) */

    uint8_t reserved_d4[0x07];

    uint8_t fm_dev_scale; /* 0x0db: FM NCO deviation scale */

    uint8_t reserved_dc[0x3c];

    uint8_t  fullband_txe; /* 0x118: full-band TX enable */
    uint16_t bias_drive;   /* 0x119: PA driver bias DAC */
    uint16_t bias_final;   /* 0x11b: PA final bias DAC */

    DrvNvramCalibBand bands[11]; /* 0x11d: TX bands, fixed order */

    uint8_t version_marker; /* 0x1f9: DRV_NVRAM_CALIB_MARK */
    uint8_t version;        /* 0x1fa: DRV_NVRAM_CALIB_MARK */
} DrvNvramCalib;

_Static_assert(sizeof(DrvNvramNvparam) == 3u, "DrvNvramNvparam layout");
_Static_assert(sizeof(DrvNvramCalibBand) == 20u, "DrvNvramCalibBand layout");
_Static_assert(sizeof(DrvNvramCalib) == DRV_NVRAM_CALIB_LEN,
               "DrvNvramCalib layout");

int Drv_Nvram_SettingsRead(uint8_t *buf);
int Drv_Nvram_CalibRead(DrvNvramCalib *calib);
int Drv_Nvram_CalibSave(const DrvNvramCalib *calib);
int Drv_Nvram_NvparamRead(DrvNvramNvparam *rec);
int Drv_Nvram_NvparamSave(uint8_t chge);

uint8_t Drv_Nvram_Crc8(const uint8_t *data, size_t len, uint8_t init);
