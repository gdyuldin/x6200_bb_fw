#include "drv_pca9535.h"

#include <errno.h>
#include <stddef.h>

#include "bsp_i2c_bus.h"
#include "bsp_log.h"

#define LOG_TAG "drv.pca9535"

/* Register pointer bytes; the output register is a 16-bit pair that the device
 * auto-increments after the pointer byte. */
#define PCA9535_REG_INPUT  0x00u
#define PCA9535_REG_OUTPUT 0x02u

typedef struct {
    uint8_t  addr7;
    uint16_t shadow;
} DrvPca9535State;

/* Only the output expanders carry a shadow; the key expander is read-only for
 * now and is included so its address resolves in the lookup. */
static DrvPca9535State s_dev[] = {
    { DRV_PCA9535_ADDR_PREAMP,     0u },
    { DRV_PCA9535_ADDR_BAND_RELAY, 0u },
    { DRV_PCA9535_ADDR_ATU,       0u },
};

static DrvPca9535State *pca9535_find(uint8_t addr7) {
    for (size_t i = 0; i < sizeof(s_dev) / sizeof(s_dev[0]); ++i) {
        if (s_dev[i].addr7 == addr7) {
            return &s_dev[i];
        }
    }
    return 0;
}

static int pca9535_flush(DrvPca9535State *d) {
    const uint8_t buf[3] = { PCA9535_REG_OUTPUT,
                             (uint8_t)(d->shadow & 0xFFu),
                             (uint8_t)(d->shadow >> 8) };
    return Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, d->addr7, buf, sizeof(buf),
                                DRV_PCA9535_XFER_TIMEOUT_MS);
}

int Drv_Pca9535_WriteMasked(uint8_t addr7, uint16_t mask, uint16_t value) {
    DrvPca9535State *d = pca9535_find(addr7);
    if (d == 0) {
        return -ENODEV;
    }
    d->shadow = (uint16_t)((d->shadow & (uint16_t)~mask) | (value & mask));
    return pca9535_flush(d);
}

int Drv_Pca9535_Write(uint8_t addr7, uint16_t value) {
    DrvPca9535State *d = pca9535_find(addr7);
    if (d == 0) {
        return -ENODEV;
    }
    d->shadow = value;
    return pca9535_flush(d);
}

int Drv_Pca9535_ReadInputs(uint8_t addr7, uint16_t *value) {
    const uint8_t reg = PCA9535_REG_INPUT;
    uint8_t       buf[2] = { 0, 0 };
    int           rc;

    if (value == 0 || pca9535_find(addr7) == 0) {
        return -ENODEV;
    }
    rc = Bsp_I2c_WriteReadTimeout(BSP_I2C_BUS_1, addr7, &reg, 1, buf, sizeof(buf),
                                  DRV_PCA9535_XFER_TIMEOUT_MS);
    if (rc != 0) {
        return rc;
    }
    *value = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    return 0;
}

int Drv_Pca9535_ReadButtons(uint32_t *mask) {
    (void)mask;
    return 0;
}

int Drv_Pca9535_Init(void) {
    /* Outputs 0, polarity 0, configuration 0 (all 16 pins outputs). The eight
     * registers are four pairs, so each one is written separately: a
     * multi-byte write only toggles inside the addressed pair. */
    static const uint8_t addrs[] = { DRV_PCA9535_ADDR_PREAMP,
                                     DRV_PCA9535_ADDR_BAND_RELAY };

    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); ++i) {
        DrvPca9535State *d = pca9535_find(addrs[i]);

        for (uint8_t reg = PCA9535_REG_OUTPUT; reg <= 0x07u; ++reg) {
            const uint8_t buf[2] = { reg, 0u };
            int           rc = Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, addrs[i], buf,
                                                    sizeof(buf),
                                                    DRV_PCA9535_XFER_TIMEOUT_MS);
            if (rc != 0) {
                LOG_WARN("init 0x%02X reg %u failed (%d)", (unsigned)addrs[i],
                         (unsigned)reg, rc);
                return rc;
            }
        }
        d->shadow = 0u;
    }
    return 0;
}
