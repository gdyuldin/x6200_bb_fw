/*
 * Host-side self-test for the NVRAM layer (no hardware, host gcc).
 *
 * Builds the real Devices/drv_eeprom.c and Devices/drv_nvram.c and mocks the
 * Platform BSP layer with a 32 KiB in-memory EEPROM plus a write journal. This
 * exercises the actual page chunking and region logic, not a copy of it.
 *
 * Coverage:
 *   1. CRC-8 check vectors;
 *   2. NVPARAM record format, validation and corruption handling;
 *   3. calibration save chunking (8 transactions, 7x64 + 59);
 *   4. region boundaries and out-of-range access.
 */

#include <stdio.h>
#include <string.h>

#include "bsp_i2c_bus.h"
#include "drv_eeprom.h"
#include "drv_nvram.h"

#define MEM_SIZE ((size_t)DRV_EEPROM_SIZE)

static uint8_t s_mem[MEM_SIZE];

typedef struct {
    uint16_t addr;
    size_t   len;
} WriteRec;

static WriteRec s_journal[64];
static size_t   s_journal_count;

int Bsp_I2c_WriteTimeout(BspI2cBus bus, uint8_t addr, const uint8_t *data,
                         size_t len, uint32_t timeout_ms) {
    (void)timeout_ms;
    if (bus != BSP_I2C_BUS_1 || addr != DRV_EEPROM_ADDR7 || len < 2) {
        return -2;
    }
    const uint16_t a = (uint16_t)((data[0] << 8) | data[1]);
    const size_t n = len - 2;
    if ((size_t)a + n > MEM_SIZE) {
        return -2;
    }
    memcpy(&s_mem[a], &data[2], n);
    if (s_journal_count < sizeof(s_journal) / sizeof(s_journal[0])) {
        s_journal[s_journal_count].addr = a;
        s_journal[s_journal_count].len = n;
        s_journal_count++;
    }
    return 0;
}

int Bsp_I2c_WriteReadTimeout(BspI2cBus bus, uint8_t addr, const uint8_t *tx,
                             size_t tx_len, uint8_t *rx, size_t rx_len,
                             uint32_t timeout_ms) {
    (void)timeout_ms;
    if (bus != BSP_I2C_BUS_1 || addr != DRV_EEPROM_ADDR7 || tx_len < 2) {
        return -2;
    }
    const uint16_t a = (uint16_t)((tx[0] << 8) | tx[1]);
    if ((size_t)a + rx_len > MEM_SIZE) {
        return -3;
    }
    memcpy(rx, &s_mem[a], rx_len);
    return 0;
}

void Bsp_Time_DelayMs(uint32_t ms) {
    (void)ms;
}

void Bsp_Iwdg_Kick(void) {
}

static int s_failures;

static void check(int cond, const char *what) {
    if (!cond) {
        printf("FAIL: %s\n", what);
        s_failures++;
    }
}

static void reset_mem(void) {
    memset(s_mem, 0, sizeof(s_mem));
    s_journal_count = 0;
}

static void test_crc8_vectors(void) {
    const uint8_t a[2] = { 0x3e, 0x00 };
    const uint8_t b[2] = { 0x3e, 0x01 };
    const uint8_t c[2] = { 0x3e, 0xff };
    const uint8_t d[2] = { 0x00, 0x00 };

    check(Drv_Nvram_Crc8(a, 2, 0) == 0xdf, "crc8 {3e,00} -> DF");
    check(Drv_Nvram_Crc8(b, 2, 0) == 0xee, "crc8 {3e,01} -> EE");
    check(Drv_Nvram_Crc8(c, 2, 0) == 0x73, "crc8 {3e,FF} -> 73");
    check(Drv_Nvram_Crc8(d, 2, 0) == 0x00, "crc8 {00,00} -> 00");
}

static void test_nvparam(void) {
    reset_mem();

    check(Drv_Nvram_NvparamSave(DRV_NVRAM_NVPARAM_CHGE_ENABLED) == 0,
          "nvparam save");
    check(s_mem[DRV_NVRAM_NVPARAM_OFFSET + 0] == DRV_NVRAM_NVPARAM_HDR,
          "nvparam header written");
    check(s_mem[DRV_NVRAM_NVPARAM_OFFSET + 1] ==
              DRV_NVRAM_NVPARAM_CHGE_ENABLED,
          "nvparam chge written");
    check(s_mem[DRV_NVRAM_NVPARAM_OFFSET + 2] ==
              Drv_Nvram_Crc8(&s_mem[DRV_NVRAM_NVPARAM_OFFSET], 2, 0),
          "nvparam crc written");
    check(s_journal_count == 1, "nvparam save = one transaction");
    check(s_journal[0].addr == DRV_NVRAM_NVPARAM_OFFSET &&
              s_journal[0].len == DRV_NVRAM_NVPARAM_LEN,
          "nvparam save within region");

    DrvNvramNvparam rec;
    check(Drv_Nvram_NvparamRead(&rec) == 0, "nvparam read valid");
    check(rec.chge == DRV_NVRAM_NVPARAM_CHGE_ENABLED, "nvparam read chge");

    s_mem[DRV_NVRAM_NVPARAM_OFFSET] = 0x00;
    check(Drv_Nvram_NvparamRead(&rec) == DRV_NVRAM_ERR_HEADER,
          "nvparam read rejects bad header");

    s_mem[DRV_NVRAM_NVPARAM_OFFSET] = DRV_NVRAM_NVPARAM_HDR;
    s_mem[DRV_NVRAM_NVPARAM_OFFSET + 2] ^= 0xff;
    check(Drv_Nvram_NvparamRead(&rec) == DRV_NVRAM_ERR_CRC,
          "nvparam read rejects bad crc");
}

static void test_calib_chunking(void) {
    reset_mem();

    DrvNvramCalib calib;
    uint8_t *raw = (uint8_t *)&calib;
    for (size_t i = 0; i < sizeof(calib); ++i) {
        raw[i] = (uint8_t)(i & 0xffu);
    }
    calib.version_marker = DRV_NVRAM_CALIB_MARK;
    calib.version = DRV_NVRAM_CALIB_MARK;

    check(Drv_Nvram_CalibSave(&calib) == 0, "calib save");

    check(s_journal_count == 8, "calib save = 8 transactions");
    static const uint16_t expect_addr[8] = {
        0x1000, 0x1040, 0x1080, 0x10c0, 0x1100, 0x1140, 0x1180, 0x11c0
    };
    static const size_t expect_len[8] = { 64, 64, 64, 64, 64, 64, 64, 59 };
    for (size_t i = 0; i < 8 && i < s_journal_count; ++i) {
        check(s_journal[i].addr == expect_addr[i], "calib chunk address");
        check(s_journal[i].len == expect_len[i], "calib chunk length");
    }

    for (size_t i = 0; i < s_journal_count; ++i) {
        check(s_journal[i].addr >= DRV_NVRAM_CALIB_OFFSET &&
                  s_journal[i].addr + s_journal[i].len <=
                      DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_LEN,
              "calib write within region");
    }

    check(s_mem[DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_MARK_OFFSET] ==
              DRV_NVRAM_CALIB_MARK,
          "calib marker 'd' written");
    check(calib.version_marker == DRV_NVRAM_CALIB_MARK, "source marker untouched");
}

static void test_calib_read(void) {
    reset_mem();

    DrvNvramCalib calib;
    uint8_t *raw = (uint8_t *)&calib;
    for (size_t i = 0; i < sizeof(calib); ++i) {
        raw[i] = (uint8_t)(i & 0xffu);
    }
    calib.version_marker = DRV_NVRAM_CALIB_MARK;
    calib.version = DRV_NVRAM_CALIB_MARK;
    check(Drv_Nvram_CalibSave(&calib) == 0, "calib save for read");

    DrvNvramCalib readback;
    check(Drv_Nvram_CalibRead(&readback) == 0, "calib read valid");
    check(memcmp(&calib, &readback, sizeof(calib)) == 0, "calib read round-trip");

    s_mem[DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_MARK_OFFSET] = 0x00;
    check(Drv_Nvram_CalibRead(&readback) == DRV_NVRAM_ERR_MARKER,
          "calib read rejects bad version marker");

    s_mem[DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_MARK_OFFSET] =
        DRV_NVRAM_CALIB_MARK;
    s_mem[DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_VER_OFFSET] = 0x00;
    check(Drv_Nvram_CalibRead(&readback) == DRV_NVRAM_ERR_MARKER,
          "calib read rejects bad version byte");

    /* A valid image must read back clean again. */
    s_mem[DRV_NVRAM_CALIB_OFFSET + DRV_NVRAM_CALIB_VER_OFFSET] =
        DRV_NVRAM_CALIB_MARK;
    check(Drv_Nvram_CalibRead(&readback) == 0, "calib read valid after restore");
}

static void test_bounds(void) {
    reset_mem();

    uint8_t buf[8] = { 0 };
    check(Drv_Eeprom_Read(0, 0, 8) == -1, "read null buffer rejected");
    check(Drv_Eeprom_Write(0, 0, 8) == -1, "write null buffer rejected");
    check(Drv_Eeprom_Read((uint16_t)(MEM_SIZE - 4), buf, 8) == -1,
          "read past device rejected");
    check(Drv_Eeprom_Write((uint16_t)(MEM_SIZE - 4), buf, 8) == -1,
          "write past device rejected");
    check(Drv_Eeprom_Read(0, buf, 0) == 0, "zero-length read is a no-op");
    check(Drv_Eeprom_Write(0, buf, 0) == 0, "zero-length write is a no-op");
    check(s_journal_count == 0, "no transactions for no-op writes");
}

int main(void) {
    test_crc8_vectors();
    test_nvparam();
    test_calib_chunking();
    test_calib_read();
    test_bounds();

    if (s_failures != 0) {
        printf("%d check(s) failed\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}
