#include "drv_eeprom.h"

#include "bsp_i2c_bus.h"
#include "bsp_iwdg.h"
#include "bsp_time.h"

/*
 * AT24C256C access rules: 2-byte memory address, 64-byte write pages, ~10 ms
 * write cycle. The BSP takes the 7-bit address and shifts it for the bus.
 */

static uint32_t eeprom_timeout_ms(size_t len) {
    return DRV_EEPROM_TIMEOUT_BASE_MS +
           DRV_EEPROM_TIMEOUT_PER_BYTE_MS * (uint32_t)len;
}

int Drv_Eeprom_Read(uint16_t offset, uint8_t *buf, size_t len) {
    if (buf == 0) {
        return -1;
    }
    if ((size_t)offset + len > DRV_EEPROM_SIZE) {
        return -1;
    }
    if (len == 0) {
        return 0;
    }

    /* 16-bit memory address, MSB first (device contract). */
    const uint8_t addr[DRV_EEPROM_MEM_ADDR_SIZE] = {
        (uint8_t)(offset >> 8),
        (uint8_t)(offset & 0xffu),
    };
    return Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_1, DRV_EEPROM_ADDR7, addr,
                                    sizeof(addr), buf, len,
                                    eeprom_timeout_ms(len));
}

int Drv_Eeprom_Write(uint16_t offset, const uint8_t *buf, size_t len) {
    if (buf == 0) {
        return -1;
    }
    if ((size_t)offset + len > DRV_EEPROM_SIZE) {
        return -1;
    }
    if (len == 0) {
        return 0;
    }

    size_t written = 0;
    while (written < len) {
        const size_t pos = (size_t)offset + written;
        size_t chunk = DRV_EEPROM_PAGE_SIZE - (pos % DRV_EEPROM_PAGE_SIZE);
        if (chunk > len - written) {
            chunk = len - written;
        }

        /* Frame = 2-byte memory address + up to one page of data. */
        uint8_t frame[DRV_EEPROM_MEM_ADDR_SIZE + DRV_EEPROM_PAGE_SIZE];
        frame[0] = (uint8_t)(pos >> 8);
        frame[1] = (uint8_t)(pos & 0xffu);
        for (size_t i = 0; i < chunk; ++i) {
            frame[DRV_EEPROM_MEM_ADDR_SIZE + i] = buf[written + i];
        }

        const int rc = Bsp_I2c_WriteTimeout(
            BSP_I2C_BUS_1, DRV_EEPROM_ADDR7, frame,
            DRV_EEPROM_MEM_ADDR_SIZE + chunk, eeprom_timeout_ms(chunk));
        if (rc != 0) {
            return rc;
        }

        written += chunk;
        Bsp_Iwdg_Kick();
        if (written < len) {
            Bsp_Time_DelayMs(DRV_EEPROM_WRITE_CYCLE_MS);
        }
    }
    return 0;
}
