#include "bsp_i2c_bus.h"

#include <stdbool.h>

#include "bsp_irq.h"
#include "i2c.h"

/* I2C2 is owned by the TIM6 protection tick after Bsp_I2c_Bus2Claim(); a
 * main-loop transfer must hold the guard so the tick cannot preempt the HAL
 * state machine mid-transfer. */
#define BSP_I2C_BUS2_DENIED (-4)

static bool     s_bus2_claimed;
static uint32_t s_bus2_guard;

static I2C_HandleTypeDef *bus_handle(BspI2cBus bus) {
    switch (bus) {
    case BSP_I2C_BUS_1: return &hi2c1;
    case BSP_I2C_BUS_2: return &hi2c2;
    case BSP_I2C_BUS_3: return &hi2c3;
    case BSP_I2C_BUS_4: return &hi2c4;
    default:            return 0;
    }
}

/* 1 = this call must be refused: I2C2 is owned by the tick and neither the
 * TIM6 ISR context nor a guard is active. */
static int bus2_denied(BspI2cBus bus) {
    if (bus != BSP_I2C_BUS_2 || !s_bus2_claimed) {
        return 0;
    }
    if (__get_IPSR() == (uint32_t)(TIM6_DAC_IRQn + 16)) {
        return 0;
    }
    return (s_bus2_guard == 0u) ? 1 : 0;
}

void Bsp_I2c_Bus2Claim(void) {
    s_bus2_claimed = true;
}

void Bsp_I2c_Bus2GuardEnter(void) {
    uint32_t basepri = Bsp_Irq_SaveLow();
    if (s_bus2_guard == 0u) {
        HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
    }
    s_bus2_guard++;
    Bsp_Irq_RestoreLow(basepri);
}

void Bsp_I2c_Bus2GuardExit(void) {
    uint32_t basepri = Bsp_Irq_SaveLow();
    if (s_bus2_guard != 0u) {
        s_bus2_guard--;
        if (s_bus2_guard == 0u) {
            HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
        }
    }
    Bsp_Irq_RestoreLow(basepri);
}

int Bsp_I2c_Write(BspI2cBus bus, uint8_t addr, const uint8_t *data, size_t len) {
    I2C_HandleTypeDef *h = bus_handle(bus);
    if (h == 0) {
        return -1;
    }
    if (bus2_denied(bus)) {
        return BSP_I2C_BUS2_DENIED;
    }
    return HAL_I2C_Master_Transmit(h, (uint16_t)(addr << 1), (uint8_t *)data,
                                   (uint16_t)len, HAL_MAX_DELAY) == HAL_OK ? 0 : -2;
}

int Bsp_I2c_Read(BspI2cBus bus, uint8_t addr, uint8_t *data, size_t len) {
    I2C_HandleTypeDef *h = bus_handle(bus);
    if (h == 0) {
        return -1;
    }
    if (bus2_denied(bus)) {
        return BSP_I2C_BUS2_DENIED;
    }
    return HAL_I2C_Master_Receive(h, (uint16_t)(addr << 1), data,
                                  (uint16_t)len, HAL_MAX_DELAY) == HAL_OK ? 0 : -2;
}

int Bsp_I2c_WriteRead(BspI2cBus bus, uint8_t addr,
                      const uint8_t *tx, size_t tx_len,
                      uint8_t *rx, size_t rx_len) {
    I2C_HandleTypeDef *h = bus_handle(bus);
    if (h == 0) {
        return -1;
    }
    if (bus2_denied(bus)) {
        return BSP_I2C_BUS2_DENIED;
    }
    if (HAL_I2C_Master_Transmit(h, (uint16_t)(addr << 1), (uint8_t *)tx,
                                (uint16_t)tx_len, HAL_MAX_DELAY) != HAL_OK) {
        return -2;
    }
    if (rx_len == 0) {
        return 0;
    }
    return HAL_I2C_Master_Receive(h, (uint16_t)(addr << 1), rx,
                                  (uint16_t)rx_len,
                                  HAL_MAX_DELAY) == HAL_OK ? 0 : -3;
}

int Bsp_I2c_WriteTimeout(BspI2cBus bus, uint8_t addr, const uint8_t *data,
                         size_t len, uint32_t timeout_ms) {
    I2C_HandleTypeDef *h = bus_handle(bus);
    if (h == 0) {
        return -1;
    }
    if (bus2_denied(bus)) {
        return BSP_I2C_BUS2_DENIED;
    }
    return HAL_I2C_Master_Transmit(h, (uint16_t)(addr << 1), (uint8_t *)data,
                                   (uint16_t)len, timeout_ms) == HAL_OK ? 0 : -2;
}

int Bsp_I2c_WriteReadTimeout(BspI2cBus bus, uint8_t addr,
                             const uint8_t *tx, size_t tx_len,
                             uint8_t *rx, size_t rx_len,
                             uint32_t timeout_ms) {
    I2C_HandleTypeDef *h = bus_handle(bus);
    if (h == 0) {
        return -1;
    }
    if (bus2_denied(bus)) {
        return BSP_I2C_BUS2_DENIED;
    }
    if (HAL_I2C_Master_Transmit(h, (uint16_t)(addr << 1), (uint8_t *)tx,
                                (uint16_t)tx_len, timeout_ms) != HAL_OK) {
        return -2;
    }
    if (rx_len == 0) {
        return 0;
    }
    return HAL_I2C_Master_Receive(h, (uint16_t)(addr << 1), rx,
                                  (uint16_t)rx_len,
                                  timeout_ms) == HAL_OK ? 0 : -3;
}
