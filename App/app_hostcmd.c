#include "app_hostcmd.h"

#include <string.h>

#include "app_nvram.h"
#include "app_power.h"
#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_boot.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "drv_nvram.h"

#define LOG_TAG "app.hostcmd"

/* Host calibration RAM shadow windows (see app_hostcmd.h). */
#define HOSTCMD_CALIB_READ_BASE  0x0100u
#define HOSTCMD_CALIB_WRITE_BASE 0xF000u

static void hostcmd_poll(void) {
    uint8_t c = 0;
    while (Bsp_Uart_ReadChar(BSP_UART_AUX, &c) == 1) {
        LOG_DEBUG("0x%02X '%c'", (unsigned)c,
                  (c >= 0x20u && c < 0x7Fu) ? (char)c : '.');

        /* (c & 0xDF) == 0x43 matches exactly 'C' (0x43) and 'c' (0x63);
         * no other byte value matches. */
        if ((c & 0xDFu) == 0x43u) {
            /* The parallel I2C host command 0x800A (register 0xfffe) calls
             * Bsp_Boot_EnterIap() as well; see app_hostlink.c. The UI update
             * sends both. */
            LOG_INFO("host request, entering bootloader");
            Bsp_Time_DelayMs(100);
            Bsp_Boot_EnterIap();
        }

        /* (c & 0xDF) == 0x53 matches exactly 'S' (0x53) and 's' (0x73);
         * no other byte value matches. */
        if ((c & 0xDFu) == 0x53u) {
            /* The parallel I2C host command 0x8009 also calls this; see
             * app_hostlink.c. The UI update sends both. */
            App_PowerOff_Request();
        }
    }
}

int App_HostCmd_Init(void) {
    if (Bsp_Uart_StartRx(BSP_UART_AUX) != 0) {
        LOG_ERROR("aux rx start failed");
        return -1;
    }
    return App_Scheduler_Add(hostcmd_poll, 10u);
}

int App_HostCmd_CalibSave(void) {
    /* Host command 0x8005: persist the calibration block to the EEPROM. */
    return Drv_Nvram_CalibSave(App_Nvram_Calib());
}

int App_HostCmd_CalibDump(void) {
    /* Host command 0x8003: report the calibration block. The block is a
     * fixed-size EEPROM image; log its size and the marker/version bytes so the
     * host log shows the dump ran. */
    const DrvNvramCalib *calib = App_Nvram_Calib();
    const uint8_t       *raw   = (const uint8_t *)calib;

    LOG_INFO("calib dump: %u B, marker 0x%02x, version 0x%02x",
             (unsigned)sizeof(*calib), (unsigned)raw[DRV_NVRAM_CALIB_MARK_OFFSET],
             (unsigned)raw[DRV_NVRAM_CALIB_VER_OFFSET]);
    return 0;
}

int App_HostCmd_CalibWindowRead(uint16_t addr, uint8_t *dst, size_t len) {
    /* Read shadow window 0x0100..0x02FA: absolute addr -> intra-region offset. */
    if (dst == 0 || len == 0u) {
        return -1;
    }
    if (addr < HOSTCMD_CALIB_READ_BASE) {
        return -1;
    }

    size_t off = (size_t)(addr - HOSTCMD_CALIB_READ_BASE);
    if (off + len > DRV_NVRAM_CALIB_LEN) {
        return -1;
    }

    const DrvNvramCalib *calib = App_Nvram_Calib();
    memcpy(dst, (const uint8_t *)calib + off, len);
    return 0;
}

int App_HostCmd_CalibWindowWrite(uint16_t addr, const uint8_t *src, size_t len) {
    /* Write shadow window 0xF000..0xF1FA: absolute addr -> intra-region offset. */
    if (src == 0 || len == 0u) {
        return -1;
    }
    if (addr < HOSTCMD_CALIB_WRITE_BASE) {
        return -1;
    }

    size_t off = (size_t)(addr - HOSTCMD_CALIB_WRITE_BASE);
    if (off + len > DRV_NVRAM_CALIB_LEN) {
        return -1;
    }

    return App_Nvram_CalibWrite((uint16_t)off, src, len);
}
