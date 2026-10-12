#include "bsp_i2c_slave.h"

#include <stdbool.h>

#include "bsp_log.h"
#include "bsp_memory.h"
#include "i2c.h"

#define LOG_TAG "bsp.i2c_slave"

/*
 * I2C3 slave transport. See bsp_i2c_slave.h for the contract and the wire
 * format. The HAL callbacks below run in the I2C3 event/error ISR; they only
 * copy and dispatch, never block.
 */

/* Register address + largest region (calib 507 B) with headroom. */
#define BSP_I2C_SLAVE_BUF_LEN 520u

static BspI2cSlaveWriteFn s_on_write;
static BspI2cSlaveReadFn  s_on_read;

static uint8_t s_rxbuf[BSP_I2C_SLAVE_BUF_LEN] BSP_RAM_D1_BSS;
static uint8_t s_txbuf[BSP_I2C_SLAVE_BUF_LEN] BSP_RAM_D1_BSS;

/* Latched register address and whether a receive phase is in flight. */
static uint16_t s_reg;
static bool     s_reg_valid;
static bool     s_rx_active;

static void slave_listen(void) {
    (void)HAL_I2C_EnableListen_IT(&hi2c3);
}

int Bsp_I2cSlave_Init(BspI2cSlaveWriteFn on_write, BspI2cSlaveReadFn on_read) {
    if (on_write == 0 || on_read == 0) {
        return -1;
    }
    s_on_write   = on_write;
    s_on_read    = on_read;
    s_reg        = 0u;
    s_reg_valid  = false;
    s_rx_active  = false;
    slave_listen();
    return 0;
}

/* Master writes to the slave: start a receive sequence. The register address
 * is the first two bytes; the sequence stays open so a repeated START can
 * follow for a read (see the AddrCallback receive branch). */
static void slave_on_write_addr(void) {
    s_rx_active = true;
    if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c3, s_rxbuf, sizeof(s_rxbuf),
                                     I2C_FIRST_FRAME) != HAL_OK) {
        s_rx_active = false;
        slave_listen();
    }
}

/* Master reads from the slave: the register address was captured in the
 * preceding receive phase. Latch it, build the reply, and transmit it. */
static void slave_on_read_addr(void) {
    size_t got = 0u;

    if (s_rx_active) {
        got = (size_t)(hi2c3.pBuffPtr - s_rxbuf);
        s_rx_active = false;
    }
    if (got >= 2u) {
        s_reg       = (uint16_t)(((uint16_t)s_rxbuf[0] << 8) | (uint16_t)s_rxbuf[1]);
        s_reg_valid = true;
    }

    size_t n = 0u;
    if (s_on_read != 0) {
        n = s_on_read(s_reg_valid ? s_reg : 0u, s_txbuf, sizeof(s_txbuf));
    }
    if (n == 0u) {
        /* Always answer at least one byte so the listen is not left armed. */
        s_txbuf[0] = 0u;
        n = 1u;
    }
    if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c3, s_txbuf, (uint16_t)n,
                                      I2C_LAST_FRAME) != HAL_OK) {
        slave_listen();
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
                          uint16_t AddrMatchCode) {
    (void)AddrMatchCode;

    if (hi2c->Instance != I2C3) {
        return;
    }

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        slave_on_write_addr();
    } else {
        slave_on_read_addr();
    }
}

/*
 * Latch the register address and dispatch the payload once per receive phase.
 * Called from whichever completion path fires first (see the note below), and
 * gated on s_rx_active so a normal completion followed by ListenCplt does not
 * dispatch twice. The buffer pointer counts every received byte; the first two
 * are the register address.
 */
static void slave_dispatch_write(void) {
    size_t total = (size_t)(hi2c3.pBuffPtr - s_rxbuf);
    s_rx_active  = false;

    if (total >= 2u) {
        s_reg = (uint16_t)(((uint16_t)s_rxbuf[0] << 8) | (uint16_t)s_rxbuf[1]);
        s_reg_valid = true;
    }
    /* A register-only write (no payload) must not be treated as a write. */
    if (total > 2u && s_on_write != 0) {
        s_on_write(s_reg, s_rxbuf + 2, total - 2u);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C3) {
        return;
    }
    slave_dispatch_write();
}

/*
 * A master write that sends fewer bytes than the armed buffer and then STOPs is
 * reported as HAL_I2C_ERROR_AF through this path, not as SlaveRxCplt (HAL
 * I2C_ITSlaveCplt sets the AF error when XferCount != 0). Since host commands
 * are short, dispatch the received payload here as well. s_rx_active keeps the
 * normal and error paths mutually exclusive.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C3) {
        return;
    }
    if (s_rx_active &&
        (hi2c->ErrorCode & (HAL_I2C_ERROR_AF | HAL_I2C_ERROR_BERR |
                            HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_OVR)) != 0u) {
        slave_dispatch_write();
    } else {
        s_rx_active = false;
    }
    if ((hi2c->ErrorCode & ~(uint32_t)HAL_I2C_ERROR_AF) != 0u) {
        LOG_DEBUG("i2c3 error 0x%08x", (unsigned)hi2c->ErrorCode);
    }
    slave_listen();
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C3) {
        return;
    }
    s_rx_active = false;
    slave_listen();
}
