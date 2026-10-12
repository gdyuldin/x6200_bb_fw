/*
 * Host-side self-test for the hand-mic key frame decoder (no hardware, host
 * gcc). Builds the real Devices/drv_hm_key_decode.c and feeds synthetic pulse
 * cells with controlled timing, so the frame state machine, the duplicate-slice
 * check and the code lifetime are exercised without a logic analyzer.
 */

#include <stdio.h>
#include <string.h>

#include "drv_hm_key_decode.h"

#define HIGH_TICKS   50u
#define PREAMBLE     7u
#define DATA_BITS    39u

/* Cell ratios: data bit 0/1 and the start/stop separator. */
#define RATIO_ZERO   1u
#define RATIO_ONE    2u
#define RATIO_SYNC   4u

static uint32_t s_now;
static int      s_failures;

static void check(int cond, const char *what) {
    if (!cond) {
        printf("FAIL: %s\n", what);
        s_failures++;
    }
}

static void emit_ratio(uint32_t ratio_num) {
    uint32_t period = HIGH_TICKS + HIGH_TICKS * ratio_num;

    s_now += 1u;
    (void)Drv_HmKeyDecode_Edge(period, HIGH_TICKS, s_now);
}

/* 20-bit word from the matrix row/column nibble pair: the high code nibble is
 * W0[13:10], the low nibble is W0[18:15]. */
static uint32_t word_from_code(uint8_t code) {
    return ((uint32_t)(code & 0xF0u) << 6) | ((uint32_t)(code & 0x0Fu) << 15);
}

/* 39 transmitted bits: the word, then the same word without its top bit. */
static void make_bits(uint8_t code, uint8_t bits[DATA_BITS]) {
    uint32_t w = word_from_code(code);
    unsigned i;

    for (i = 0; i < 20u; i++) {
        bits[i] = (uint8_t)((w >> i) & 1u);
    }
    for (i = 0; i < 19u; i++) {
        bits[20u + i] = (uint8_t)((w >> i) & 1u);
    }
}

static void feed_bits(const uint8_t bits[DATA_BITS]) {
    unsigned i;

    for (i = 0; i < DATA_BITS; i++) {
        emit_ratio(bits[i] ? RATIO_ONE : RATIO_ZERO);
    }
}

static void feed_frame(const uint8_t bits[DATA_BITS]) {
    unsigned i;

    for (i = 0; i < PREAMBLE; i++) {
        emit_ratio(RATIO_ZERO);
    }
    emit_ratio(RATIO_SYNC);
    feed_bits(bits);
    emit_ratio(RATIO_SYNC);
}

static void test_valid_codes(void) {
    static const uint8_t codes[] = { 0xC1u, 0x04u, 0x81u, 0xE8u, 0xA4u, 0x28u };
    uint8_t bits[DATA_BITS];

    for (unsigned i = 0; i < sizeof(codes); i++) {
        Drv_HmKeyDecode_Reset();
        s_now = 1000u;
        make_bits(codes[i], bits);
        feed_frame(bits);
        check(Drv_HmKeyDecode_Get(s_now + 10u) == codes[i],
              "valid frame decodes");
    }
}

static void test_lifetime(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 2000u;
    make_bits(0xC1u, bits);
    feed_frame(bits);

    check(Drv_HmKeyDecode_Get(s_now + 49u) == 0xC1u, "code held before timeout");
    check(Drv_HmKeyDecode_Get(s_now + 50u) == 0u, "code cleared at timeout");
}

static void test_repeat_mismatch(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 3000u;
    make_bits(0xC1u, bits);
    bits[32] ^= 1u; /* second copy of bit 12 no longer matches the first */
    feed_frame(bits);

    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0u, "repeat mismatch rejected");
}

static void test_top_bit(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 4000u;
    make_bits(0xC1u, bits);
    bits[19] = 1u; /* W0 top bit must be 0 */
    feed_frame(bits);

    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0u, "top bit rejected");
}

static void test_bad_ratio(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 5000u;
    make_bits(0xC1u, bits);
    for (unsigned i = 0; i < PREAMBLE; i++) {
        emit_ratio(RATIO_ZERO);
    }
    emit_ratio(RATIO_SYNC);
    for (unsigned i = 0; i < DATA_BITS; i++) {
        /* ratio 3 is between the data-bit classes and aborts the frame */
        emit_ratio(i == 5u ? 3u : (bits[i] ? RATIO_ONE : RATIO_ZERO));
    }
    emit_ratio(RATIO_SYNC);

    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0u, "bad ratio rejected");
}

/* 0x04 starts with seventeen zero bits, so a broken preamble cannot rebuild
 * one from the data stream by accident. */
static void test_bad_preamble_short(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 6000u;
    make_bits(0x04u, bits);
    for (unsigned i = 0; i < PREAMBLE - 1u; i++) {
        emit_ratio(RATIO_ZERO);
    }
    emit_ratio(RATIO_SYNC);
    feed_bits(bits);
    emit_ratio(RATIO_SYNC);

    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0u, "short preamble rejected");
}

static void test_bad_preamble_long(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 7000u;
    make_bits(0x04u, bits);
    for (unsigned i = 0; i < PREAMBLE + 1u; i++) {
        emit_ratio(RATIO_ZERO);
    }
    emit_ratio(RATIO_SYNC);
    feed_bits(bits);
    emit_ratio(RATIO_SYNC);

    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0u, "long preamble rejected");
}

static void test_timeout_midframe(void) {
    uint8_t bits[DATA_BITS];

    Drv_HmKeyDecode_Reset();
    s_now = 8000u;
    make_bits(0xC1u, bits);
    for (unsigned i = 0; i < PREAMBLE; i++) {
        emit_ratio(RATIO_ZERO);
    }
    emit_ratio(RATIO_SYNC);
    for (unsigned i = 0; i < 10u; i++) {
        emit_ratio(bits[i] ? RATIO_ONE : RATIO_ZERO);
    }

    /* Longer gap than the code lifetime: the partial frame is dropped. */
    s_now += 60u;
    for (unsigned i = 10u; i < DATA_BITS; i++) {
        emit_ratio(bits[i] ? RATIO_ONE : RATIO_ZERO);
    }
    emit_ratio(RATIO_SYNC);

    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0u, "stale frame dropped");

    /* A fresh frame after the gap still decodes. */
    feed_frame(bits);
    check(Drv_HmKeyDecode_Get(s_now + 10u) == 0xC1u, "frame after gap decodes");
}

int main(void) {
    test_valid_codes();
    test_lifetime();
    test_repeat_mismatch();
    test_top_bit();
    test_bad_ratio();
    test_bad_preamble_short();
    test_bad_preamble_long();
    test_timeout_midframe();

    if (s_failures != 0) {
        printf("%d check(s) failed\n", s_failures);
        return 1;
    }
    printf("all checks passed\n");
    return 0;
}