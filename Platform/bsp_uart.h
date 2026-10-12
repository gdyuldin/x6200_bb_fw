#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_UART_CAT = 0,
    BSP_UART_DEBUG,
    BSP_UART_AUX,
    BSP_UART_COUNT
} BspUart;

typedef void (*BspUartTxDoneFn)(int status, void *ctx);

int Bsp_Uart_Write(BspUart port, const uint8_t *data, size_t len);

int Bsp_Uart_WriteAsync(BspUart port, const uint8_t *data, uint16_t len,
                        BspUartTxDoneFn done, void *ctx);

int    Bsp_Uart_StartRx(BspUart port);
int    Bsp_Uart_ReadChar(BspUart port, uint8_t *b);
size_t Bsp_Uart_RxAvailable(BspUart port);
size_t Bsp_Uart_RxDropped(BspUart port);

/* Stop the port's DMA and de-initialise it (HAL_UART_DMAStop + DeInit).
 * Used by the IAP hand-off; does not disturb the rest of the BSP API. */
int    Bsp_Uart_Shutdown(BspUart port);

/* Reconfigure the port from its handle config (HAL_UART_Init) after an RCC
 * peripheral reset pulse. Used by the IAP hand-off: its RSTR pulse resets
 * USART1 along with the other peripherals. */
int    Bsp_Uart_Reinit(BspUart port);
