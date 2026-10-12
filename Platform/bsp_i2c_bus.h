#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_I2C_BUS_1 = 0,
    BSP_I2C_BUS_2,
    BSP_I2C_BUS_3,
    BSP_I2C_BUS_4,
    BSP_I2C_BUS_COUNT
} BspI2cBus;

int Bsp_I2c_Write(BspI2cBus bus, uint8_t addr, const uint8_t *data, size_t len);
int Bsp_I2c_Read(BspI2cBus bus, uint8_t addr, uint8_t *data, size_t len);
int Bsp_I2c_WriteRead(BspI2cBus bus, uint8_t addr,
                      const uint8_t *tx, size_t tx_len,
                      uint8_t *rx, size_t rx_len);

/* Timeout variants: identical to the calls above but with an explicit HAL
 * timeout in milliseconds instead of an unbounded wait. They exist so a
 * device driver can bound a transfer (and avoid a system hang if a slave
 * holds SCL) without changing the bus configuration. */
int Bsp_I2c_WriteTimeout(BspI2cBus bus, uint8_t addr,
                         const uint8_t *data, size_t len,
                         uint32_t timeout_ms);
int Bsp_I2c_WriteReadTimeout(BspI2cBus bus, uint8_t addr,
                             const uint8_t *tx, size_t tx_len,
                             uint8_t *rx, size_t rx_len,
                             uint32_t timeout_ms);

/*
 * I2C2 ownership: the TIM6 protection tick owns the
 * gauge + charger bus. Bsp_I2c_Bus2Claim() is called once by App_Protect_Init()
 * just before TIM6 starts; after that a main-loop caller must wrap its I2C2
 * transfers in Bsp_I2c_Bus2GuardEnter()/Exit(). The guard masks TIM6_DAC_IRQn
 * only (not PRIMASK), so SysTick and the HAL timeouts keep running. A Bsp_I2c_*
 * call on BSP_I2C_BUS_2 without the guard and outside the TIM6 ISR is refused
 * with -4 instead of racing the HAL state machine.
 */
void Bsp_I2c_Bus2Claim(void);
void Bsp_I2c_Bus2GuardEnter(void);
void Bsp_I2c_Bus2GuardExit(void);
