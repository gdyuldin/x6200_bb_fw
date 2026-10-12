#include "bsp_uart.h"

#include <stdbool.h>

#include "bsp_memory.h"
#include "bsp_prof.h"
#include "usart.h"

#define BSP_AUX_DMA_SIZE  256u
#define BSP_AUX_DMA_MASK  (BSP_AUX_DMA_SIZE - 1u)
#define BSP_AUX_RING_SIZE 128u
#define BSP_AUX_RING_MASK (BSP_AUX_RING_SIZE - 1u)
#define BSP_DBG_RING_SIZE 128u
#define BSP_DBG_RING_MASK (BSP_DBG_RING_SIZE - 1u)

static UART_HandleTypeDef *uart_handle(BspUart port) {
    switch (port) {
    case BSP_UART_CAT:   return &huart4;
    case BSP_UART_DEBUG: return &huart1;
    case BSP_UART_AUX:   return &huart2;
    default:             return 0;
    }
}

static BspUart uart_port(UART_HandleTypeDef *h) {
    for (BspUart p = BSP_UART_CAT; p < BSP_UART_COUNT; p++) {
        if (uart_handle(p) == h) {
            return p;
        }
    }
    return BSP_UART_COUNT;
}

static BspUartTxDoneFn s_tx_done[BSP_UART_COUNT];
static void           *s_tx_done_ctx[BSP_UART_COUNT];

static uint8_t s_aux_dma[BSP_AUX_DMA_SIZE]
    __attribute__((section(".dma_d2"), aligned(32)));

static uint8_t           s_aux_ring[BSP_AUX_RING_SIZE] BSP_RAM_D1_BSS;
static volatile uint16_t s_aux_head;
static volatile uint16_t s_aux_tail;
static volatile uint16_t s_aux_pos;
static volatile bool     s_aux_started;
static volatile size_t   s_aux_dropped;

/*
 * USART1 has no RX DMA (only TX, Core/Src/usart.c), so the debug port receives
 * one byte at a time through HAL_UART_Receive_IT. The interrupt is already
 * routed: USART1_IRQHandler calls HAL_UART_IRQHandler(&huart1).
 */
static uint8_t           s_dbg_ring[BSP_DBG_RING_SIZE] BSP_RAM_D1_BSS;
static volatile uint16_t s_dbg_head;
static volatile uint16_t s_dbg_tail;
static volatile size_t   s_dbg_dropped;
static uint8_t           s_dbg_byte;
static volatile bool     s_dbg_started;

int Bsp_Uart_Write(BspUart port, const uint8_t *data, size_t len) {
    UART_HandleTypeDef *h = uart_handle(port);
    if (h == 0) {
        return -1;
    }
    return HAL_UART_Transmit(h, (uint8_t *)data, (uint16_t)len,
                             HAL_MAX_DELAY) == HAL_OK ? 0 : -2;
}

int Bsp_Uart_WriteAsync(BspUart port, const uint8_t *data, uint16_t len,
                        BspUartTxDoneFn done, void *ctx) {
    UART_HandleTypeDef *h = uart_handle(port);
    if (h == 0 || data == 0 || len == 0) {
        return -1;
    }
    s_tx_done[port] = done;
    s_tx_done_ctx[port] = ctx;
    return HAL_UART_Transmit_DMA(h, data, len) == HAL_OK ? 0 : -2;
}

static void aux_rx_start(void) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_aux_dma, BSP_AUX_DMA_SIZE) ==
        HAL_OK) {
        s_aux_pos = 0;
        s_aux_started = true;
    }
}

static int dbg_rx_start(void) {
    if (HAL_UART_Receive_IT(&huart1, &s_dbg_byte, 1u) != HAL_OK) {
        s_dbg_started = false;
        return -2;
    }
    s_dbg_started = true;
    return 0;
}

int Bsp_Uart_StartRx(BspUart port) {
    if (port == BSP_UART_AUX) {
        s_aux_head = 0;
        s_aux_tail = 0;
        s_aux_dropped = 0;
        s_aux_started = false;
        aux_rx_start();
        return s_aux_started ? 0 : -2;
    }
    if (port == BSP_UART_DEBUG) {
        s_dbg_head = 0;
        s_dbg_tail = 0;
        s_dbg_dropped = 0;
        s_dbg_started = false;
        return dbg_rx_start();
    }
    return -1;
}

int Bsp_Uart_ReadChar(BspUart port, uint8_t *b) {
    if (b == 0) {
        return -1;
    }
    if (port == BSP_UART_AUX) {
        if (s_aux_tail == s_aux_head) {
            return 0;
        }
        *b = s_aux_ring[s_aux_tail];
        s_aux_tail = (uint16_t)((s_aux_tail + 1u) & BSP_AUX_RING_MASK);
        return 1;
    }
    if (port == BSP_UART_DEBUG) {
        if (s_dbg_tail == s_dbg_head) {
            return 0;
        }
        *b = s_dbg_ring[s_dbg_tail];
        s_dbg_tail = (uint16_t)((s_dbg_tail + 1u) & BSP_DBG_RING_MASK);
        return 1;
    }
    return -1;
}

size_t Bsp_Uart_RxAvailable(BspUart port) {
    if (port == BSP_UART_AUX) {
        return (size_t)((uint16_t)(s_aux_head - s_aux_tail) & BSP_AUX_RING_MASK);
    }
    if (port == BSP_UART_DEBUG) {
        return (size_t)((uint16_t)(s_dbg_head - s_dbg_tail) & BSP_DBG_RING_MASK);
    }
    return 0;
}

int Bsp_Uart_Shutdown(BspUart port) {
    UART_HandleTypeDef *h = uart_handle(port);
    if (h == 0) {
        return -1;
    }
    if (port == BSP_UART_AUX) {
        s_aux_started = false;
    }
    HAL_UART_DMAStop(h);
    return HAL_UART_DeInit(h) == HAL_OK ? 0 : -2;
}

int Bsp_Uart_Reinit(BspUart port) {
    UART_HandleTypeDef *h = uart_handle(port);
    if (h == 0) {
        return -1;
    }
    return HAL_UART_Init(h) == HAL_OK ? 0 : -2;
}

size_t Bsp_Uart_RxDropped(BspUart port) {
    if (port == BSP_UART_AUX) {
        return s_aux_dropped;
    }
    if (port == BSP_UART_DEBUG) {
        return s_dbg_dropped;
    }
    return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart != &huart1 || !s_dbg_started) {
        return;
    }
    BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_USART1, prof_start);

    uint16_t next = (uint16_t)((s_dbg_head + 1u) & BSP_DBG_RING_MASK);
    if (next == s_dbg_tail) {
        s_dbg_dropped++;
    } else {
        s_dbg_ring[s_dbg_head] = s_dbg_byte;
        s_dbg_head = next;
    }
    (void)dbg_rx_start();

    BSP_PROF_ISR_END(BSP_PROF_ISR_USART1, prof_start);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (uart_port(huart) != BSP_UART_AUX) {
        return;
    }
    BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_USART2, prof_start);

    uint16_t last = s_aux_pos;
    uint16_t delta = (Size >= last) ? (uint16_t)(Size - last)
                                    : (uint16_t)(Size + BSP_AUX_DMA_SIZE - last);
    uint16_t idx = (uint16_t)(last & BSP_AUX_DMA_MASK);
    for (uint16_t i = 0; i < delta; i++) {
        uint8_t  b    = s_aux_dma[idx];
        uint16_t next = (uint16_t)((s_aux_head + 1u) & BSP_AUX_RING_MASK);
        if (next == s_aux_tail) {
            s_aux_dropped++;
        } else {
            s_aux_ring[s_aux_head] = b;
            s_aux_head = next;
        }
        idx = (uint16_t)((idx + 1u) & BSP_AUX_DMA_MASK);
    }
    s_aux_pos = Size;

    BSP_PROF_ISR_END(BSP_PROF_ISR_USART2, prof_start);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    BspUart p = uart_port(huart);
    if (p < BSP_UART_COUNT && s_tx_done[p] != 0) {
        BSP_PROF_ISR_BEGIN((p == BSP_UART_AUX) ? BSP_PROF_ISR_USART2
                                               : BSP_PROF_ISR_USART1,
                           prof_start);
        s_tx_done[p](0, s_tx_done_ctx[p]);
        BSP_PROF_ISR_END((p == BSP_UART_AUX) ? BSP_PROF_ISR_USART2
                                             : BSP_PROF_ISR_USART1,
                         prof_start);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    BspUart p = uart_port(huart);
    if (p >= BSP_UART_COUNT) {
        return;
    }
    BSP_PROF_ISR_BEGIN((p == BSP_UART_AUX) ? BSP_PROF_ISR_USART2
                                           : BSP_PROF_ISR_USART1,
                       prof_start);

    uint32_t err = huart->ErrorCode;
    int rx_error = (err & (HAL_UART_ERROR_PE | HAL_UART_ERROR_NE |
                           HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE |
                           HAL_UART_ERROR_RTO)) != 0u;
    /* AUX uses DMA only for RX, so a DMA error there is a receive error. */
    if (p == BSP_UART_AUX && (err & HAL_UART_ERROR_DMA) != 0u) {
        rx_error = 1;
    }

    if (p == BSP_UART_AUX && rx_error && s_aux_started) {
        aux_rx_start();
    }
    if (p == BSP_UART_DEBUG && rx_error && s_dbg_started) {
        (void)dbg_rx_start();
    }

    if (!rx_error && s_tx_done[p] != 0) {
        BspUartTxDoneFn fn  = s_tx_done[p];
        void            *ctx = s_tx_done_ctx[p];
        s_tx_done[p]     = 0;
        s_tx_done_ctx[p] = 0;
        fn(-1, ctx);
    }

    BSP_PROF_ISR_END((p == BSP_UART_AUX) ? BSP_PROF_ISR_USART2
                                         : BSP_PROF_ISR_USART1,
                     prof_start);
}
