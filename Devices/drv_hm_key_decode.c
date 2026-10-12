#include "drv_hm_key_decode.h"

/* A decoded code stays valid this long after the last edge, then reads as 0.
 * The host treats any 0 as "released". */
#define HM_KEY_IDLE_TIMEOUT_MS 50u
#define HM_KEY_PREAMBLE_CELLS  7u
#define HM_KEY_DATA_BITS       39u

typedef struct {
    uint32_t state;
    uint32_t idx;
    uint32_t w0;
    uint32_t w1;
    uint8_t  code;
    uint32_t last_edge_ms;
} HmKeyDecoder;

static HmKeyDecoder s_dec;

/*
 * Classify one cell by pause/high ratio. Class 0 and 1 are data bits, class 2
 * is the start/stop separator; anything else aborts the frame. The ratio is
 * scaled by 1000 to stay in integer math.
 */
static int cell_class(uint32_t period, uint32_t high) {
    uint32_t r;

    if (period == 0u || high == 0u || period <= high) {
        return -1;
    }
    r = (period - high) * 1000u / high;
    if (r >= 650u && r <= 1350u) {
        return 0;
    }
    if (r >= 1650u && r <= 2350u) {
        return 1;
    }
    if (r >= 3650u && r <= 4350u) {
        return 2;
    }
    return -1;
}

static void reset_stream(void) {
    s_dec.state = 0u;
    s_dec.idx   = 0u;
    s_dec.w0    = 0u;
    s_dec.w1    = 0u;
    s_dec.code  = 0u;
}

void Drv_HmKeyDecode_Reset(void) {
    reset_stream();
    s_dec.last_edge_ms = 0u;
}

uint8_t Drv_HmKeyDecode_Edge(uint32_t period, uint32_t high, uint32_t now_ms) {
    int cls;

    /* A gap longer than the code lifetime means the previous frame is over;
     * without this a late preamble cell would be taken as data. */
    if ((now_ms - s_dec.last_edge_ms) >= HM_KEY_IDLE_TIMEOUT_MS) {
        reset_stream();
    }
    s_dec.last_edge_ms = now_ms;
    cls                = cell_class(period, high);

    if (s_dec.state < HM_KEY_PREAMBLE_CELLS) {
        s_dec.state = (cls == 0) ? s_dec.state + 1u : 0u;
        return s_dec.code;
    }
    if (s_dec.state == HM_KEY_PREAMBLE_CELLS) {
        if (cls == 2) {
            s_dec.idx   = 0u;
            s_dec.w0    = 0u;
            s_dec.w1    = 0u;
            s_dec.state = HM_KEY_PREAMBLE_CELLS + 1u;
        } else {
            s_dec.state = 0u;
        }
        return s_dec.code;
    }

    if (cls != 0 && cls != 1) {
        s_dec.state = 0u;
        return s_dec.code;
    }
    if (s_dec.idx < 20u) {
        s_dec.w0 |= (uint32_t)cls << s_dec.idx;
    } else if (s_dec.idx < HM_KEY_DATA_BITS) {
        s_dec.w1 |= (uint32_t)cls << (s_dec.idx - 20u);
    }
    s_dec.idx++;
    if (s_dec.idx == HM_KEY_DATA_BITS) {
        /* Bits 12..19 of both copies must match; bit 19 is the unsent top bit
         * of the second copy and is always 0. */
        if (((s_dec.w1 ^ s_dec.w0) & 0x000FF000u) == 0u) {
            s_dec.code = (uint8_t)(((s_dec.w0 >> 6) & 0xF0u) |
                                   ((s_dec.w0 >> 15) & 0x0Fu));
        }
        s_dec.state = 0u;
    }
    return s_dec.code;
}

uint8_t Drv_HmKeyDecode_Get(uint32_t now_ms) {
    /* Only the exposed code is cleared here; the frame state is reset by the
     * next edge, where a fresh frame cannot race with this read. */
    if ((now_ms - s_dec.last_edge_ms) >= HM_KEY_IDLE_TIMEOUT_MS) {
        s_dec.code = 0u;
    }
    return s_dec.code;
}