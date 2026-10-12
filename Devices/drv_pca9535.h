#pragma once

#include <stdint.h>

/*
 * PCA9535 16-bit I2C I/O expander on I2C1 (BSP_I2C_BUS_1).
 *
 * Three parts sit on the bus: the preamp/BPF expander, the band-relay/LPF
 * expander and the ATU L/C relay expander. The driver is the single owner of a
 * 16-bit output shadow per address, so every write is a read-modify-write
 * against the shadow and never clobbers bits owned by another writer (preamp,
 * attenuator, gain, ...). Only the two output expanders are configured by
 * Init; the ATU expander is configured by Devices/drv_atu_relay.
 *
 * Output register 0x02/0x03 is written as one 3-byte burst [0x02, lo, hi] using
 * the device auto-increment. Returns 0 on success, -ENODEV for an address that
 * is not a known expander, or the negative Bsp_I2c error code.
 */

#define DRV_PCA9535_ADDR_PREAMP     0x20u  /* BPF [11:8], preamp [14], atten [13], gain [5:0] */
#define DRV_PCA9535_ADDR_BAND_RELAY 0x21u  /* LPF [15:6], tuner_on [2:0] */
#define DRV_PCA9535_ADDR_ATU        0x24u  /* ATU L/C relays, configured by drv_atu_relay */

#define DRV_PCA9535_XFER_TIMEOUT_MS 10u

int Drv_Pca9535_Init(void);
int Drv_Pca9535_WriteMasked(uint8_t addr7, uint16_t mask, uint16_t value);
int Drv_Pca9535_Write(uint8_t addr7, uint16_t value);
int Drv_Pca9535_ReadInputs(uint8_t addr7, uint16_t *value);
int Drv_Pca9535_ReadButtons(uint32_t *mask);
