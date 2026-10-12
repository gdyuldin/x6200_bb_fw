#include "bsp_log.h"

#include <stdarg.h>
#include <stdio.h>

#include "bsp_irq.h"
#include "bsp_uart.h"

#define BSP_LOG_RING_SIZE 4096u
#define BSP_LOG_RING_MASK (BSP_LOG_RING_SIZE - 1u)
#define BSP_LOG_LINE_MAX  256u

static uint8_t s_log_ring[BSP_LOG_RING_SIZE]
    __attribute__((section(".dma_d2"), aligned(32)));

static volatile uint16_t s_head;
static volatile uint16_t s_tail;
static volatile uint16_t s_tx_len;
static volatile int      s_tx_active;
static volatile size_t   s_dropped;

static void log_pump(void);

static void log_tx_done(int status, void *ctx) {
    (void)ctx;
    uint32_t irq = Bsp_Irq_SaveLow();
    if (status != 0) {
        s_dropped += s_tx_len;
    }
    s_tail = (uint16_t)((s_tail + s_tx_len) & BSP_LOG_RING_MASK);
    s_tx_len = 0;
    s_tx_active = 0;
    Bsp_Irq_RestoreLow(irq);
    log_pump();
}

static void log_pump(void) {
    uint32_t irq = Bsp_Irq_SaveLow();
    if (s_tx_active || s_head == s_tail) {
        Bsp_Irq_RestoreLow(irq);
        return;
    }
    uint16_t tail = s_tail;
    uint16_t chunk = (s_head > tail) ? (uint16_t)(s_head - tail)
                                     : (uint16_t)(BSP_LOG_RING_SIZE - tail);
    s_tx_len = chunk;
    s_tx_active = 1;
    Bsp_Irq_RestoreLow(irq);

    if (Bsp_Uart_WriteAsync(BSP_UART_DEBUG, &s_log_ring[tail], chunk,
                            log_tx_done, 0) != 0) {
        uint32_t irq2 = Bsp_Irq_SaveLow();
        s_tx_len = 0;
        s_tx_active = 0;
        Bsp_Irq_RestoreLow(irq2);
    }
}

static void log_push(const char *data, size_t len) {
    uint32_t irq = Bsp_Irq_SaveLow();
    for (size_t i = 0; i < len; i++) {
        uint16_t next = (uint16_t)((s_head + 1) & BSP_LOG_RING_MASK);
        if (next == s_tail) {
            s_dropped += len - i;
            break;
        }
        s_log_ring[s_head] = (uint8_t)data[i];
        s_head = next;
    }
    Bsp_Irq_RestoreLow(irq);
    log_pump();
}

int Bsp_Log_Init(void) {
    uint32_t irq = Bsp_Irq_SaveLow();
    s_head = 0;
    s_tail = 0;
    s_tx_len = 0;
    s_tx_active = 0;
    s_dropped = 0;
    Bsp_Irq_RestoreLow(irq);
    return 0;
}

int Bsp_Log_Write(const char *data, size_t len) {
    if (data == 0 || len == 0) {
        return -1;
    }
    log_push(data, len);
    return 0;
}

int Bsp_Log_WriteStr(const char *str) {
    if (str == 0) {
        return -1;
    }
    size_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    log_push(str, len);
    return 0;
}

int Bsp_Log_Print(BspLogLevel level, const char *tag, const char *fmt, ...) {
    char        line[BSP_LOG_LINE_MAX];
    const char *t;
    char        lc;
    va_list     ap;
    int         n;
    int         m;
    size_t      used;

    if (fmt == 0) {
        return -1;
    }

    switch (level) {
    case BSP_LOG_ERROR: lc = 'E'; break;
    case BSP_LOG_WARN:  lc = 'W'; break;
    case BSP_LOG_INFO:  lc = 'I'; break;
    case BSP_LOG_DEBUG: lc = 'D'; break;
    case BSP_LOG_TRACE: lc = 'T'; break;
    default:            return -1;
    }

    t = (tag != 0 && tag[0] != '\0') ? tag : "?";

    n = snprintf(line, sizeof(line), "[%c][%s] ", lc, t);
    if (n < 0) {
        return -1;
    }
    if ((size_t)n >= sizeof(line)) {
        n = (int)(sizeof(line) - 1u);
    }

    va_start(ap, fmt);
    m = vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
    va_end(ap);
    if (m < 0) {
        return -1;
    }

    used = (size_t)n;
    if ((size_t)m < sizeof(line) - used) {
        used += (size_t)m;
    } else {
        used = sizeof(line) - 1u;
    }

    if (used + 2u <= sizeof(line)) {
        line[used++] = '\r';
        line[used++] = '\n';
    }

    log_push(line, used);
    return 0;
}

size_t Bsp_Log_Dropped(void) {
    return s_dropped;
}
