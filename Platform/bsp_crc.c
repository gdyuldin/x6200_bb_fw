#include "bsp_crc.h"

#include "crc.h"
#include "stm32h7xx_hal.h"

/* CRC-32/MPEG-2 word vectors; values depend on the word feeding order. */
#define BSP_CRC32_VEC_ZEROS      0xF54D034Fu
#define BSP_CRC32_VEC_RAMP       0x0A133F4Fu
#define BSP_CRC32_SELFTEST_WORDS 137u

int Bsp_Crc32_Calc(const void *data, size_t len, uint32_t *out) {
    if (data == 0 || out == 0 || (len % 4u) != 0u) {
        return -1;
    }
    *out = HAL_CRC_Calculate(&hcrc, (const uint32_t *)data,
                             (uint32_t)(len / 4u));
    return 0;
}

int Bsp_Crc32_SelfTest(void) {
    static uint32_t buf[BSP_CRC32_SELFTEST_WORDS];
    uint32_t crc;
    size_t   i;

    for (i = 0; i < BSP_CRC32_SELFTEST_WORDS; i++) {
        buf[i] = 0u;
    }
    if (Bsp_Crc32_Calc(buf, sizeof(buf), &crc) != 0) {
        return -1;
    }
    if (crc != BSP_CRC32_VEC_ZEROS) {
        return -2;
    }

    for (i = 0; i < BSP_CRC32_SELFTEST_WORDS; i++) {
        buf[i] = (uint32_t)i;
    }
    if (Bsp_Crc32_Calc(buf, sizeof(buf), &crc) != 0) {
        return -1;
    }
    return crc == BSP_CRC32_VEC_RAMP ? 0 : -2;
}
