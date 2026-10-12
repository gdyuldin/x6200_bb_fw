#include "drv_atu_relay.h"

#include <stddef.h>

#include "bsp_i2c_bus.h"
#include "bsp_log.h"
#include "bsp_time.h"

#define LOG_TAG "drv.atu_relay"

#define ATU_RELAY_REG_OUTPUT 0x02u

/* Expander addresses: the L/C banks share 0x24, the switch and tuner_on live in
 * bits [2:0] of 0x21 (see header for the ownership note). */
#define ATU_RELAY_ADDR_LC 0x24u
#define ATU_RELAY_ADDR_SW 0x21u

/* Fill masks that energize every coil of one L/C bank before the code is
 * latched; bits outside the bank stay zero. */
#define ATU_RELAY_L_FILL 0x007Eu
#define ATU_RELAY_C_FILL 0x3F00u

/* Dwell per phase: shorter for the L/C coil banks, longer for the switch/tuner_on
 * relays. All tunable on hardware. */
#define ATU_RELAY_LC_STEP_MS     10u
#define ATU_RELAY_SW_STEP_MS     30u

typedef struct {
    uint8_t addr;
    uint8_t phases;  /* number of writes in the sequence */
    uint8_t step_ms; /* dwell between phases */
} AtuRelayGroupCfg;

static const AtuRelayGroupCfg s_cfg[DRV_ATU_GROUP_COUNT] = {
    [DRV_ATU_GROUP_L]      = { ATU_RELAY_ADDR_LC, 4u, ATU_RELAY_LC_STEP_MS },
    [DRV_ATU_GROUP_C]      = { ATU_RELAY_ADDR_LC, 4u, ATU_RELAY_LC_STEP_MS },
    [DRV_ATU_GROUP_SWITCH] = { ATU_RELAY_ADDR_SW, 2u, ATU_RELAY_SW_STEP_MS },
    [DRV_ATU_GROUP_ON]     = { ATU_RELAY_ADDR_SW, 2u, ATU_RELAY_SW_STEP_MS },
};

/* Last committed selection per group; -1 = never written (cache poisoned). */
static int16_t     s_cached[DRV_ATU_GROUP_COUNT];

/* Deferred request per group; -1 = none. Only one pulse sequence can run at a
 * time (shared I2C bus and expander words), so a request that arrives while
 * another sequence is in flight is held here instead of aborting it. */
static int16_t     s_want[DRV_ATU_GROUP_COUNT];

static bool        s_busy;
static DrvAtuGroup s_active;
static uint8_t     s_sel;
static uint8_t     s_phase;
static uint32_t    s_deadline;

/* Code-bit to relay-coil mapping for the two latching banks (board wire layout).
 * The code bits drive the coils active-low; code bits 6/7 are not connected.
 *   L code bit 0..5 -> P0.4, P0.2, P0.1, P0.6, P0.3, P0.5
 *   C code bit 0..5 -> P1.0, P1.1, P1.2, P1.4, P1.5, P1.3 */
static uint16_t atu_relay_sw1f_code(uint8_t code) {
    uint16_t v = 0x007Fu;

    if ((code & 0x01u) != 0u) {
        v = (uint16_t)(v & ~0x0010u);
    }
    if ((code & 0x02u) != 0u) {
        v = (uint16_t)(v & ~0x0004u);
    }
    if ((code & 0x04u) != 0u) {
        v = (uint16_t)(v & ~0x0002u);
    }
    if ((code & 0x08u) != 0u) {
        v = (uint16_t)(v & ~0x0040u);
    }
    if ((code & 0x10u) != 0u) {
        v = (uint16_t)(v & ~0x0008u);
    }
    if ((code & 0x20u) != 0u) {
        v = (uint16_t)(v & ~0x0020u);
    }
    return v;
}

static uint16_t atu_relay_sw20_code(uint8_t code) {
    uint16_t v = 0x3F80u;

    if ((code & 0x01u) != 0u) {
        v = (uint16_t)(v & ~0x0100u);
    }
    if ((code & 0x02u) != 0u) {
        v = (uint16_t)(v & ~0x0200u);
    }
    if ((code & 0x04u) != 0u) {
        v = (uint16_t)(v & ~0x0400u);
    }
    if ((code & 0x08u) != 0u) {
        v = (uint16_t)(v & ~0x1000u);
    }
    if ((code & 0x10u) != 0u) {
        v = (uint16_t)(v & ~0x2000u);
    }
    if ((code & 0x20u) != 0u) {
        v = (uint16_t)(v & ~0x0800u);
    }
    return v;
}

/* Value of the given phase for group g at selection sel:
 *   L:      fill, 0, code, 0
 *   C:      fill, 0, code, 0
 *   switch: sel ? 0x0005 : 0x0002, 0
 *   tuner_on:   sel ? 0x0006 : 0x0001, 0 */
static uint16_t atu_relay_value(DrvAtuGroup g, uint8_t phase, uint8_t sel) {
    switch (g) {
    case DRV_ATU_GROUP_L:
        if (phase == 0u) {
            return ATU_RELAY_L_FILL;
        }
        if (phase == 2u) {
            return atu_relay_sw1f_code(sel);
        }
        return 0u;
    case DRV_ATU_GROUP_C:
        if (phase == 0u) {
            return ATU_RELAY_C_FILL;
        }
        if (phase == 2u) {
            return atu_relay_sw20_code(sel);
        }
        return 0u;
    case DRV_ATU_GROUP_SWITCH:
        return (phase == 0u) ? (sel ? 0x0005u : 0x0002u) : 0u;
    case DRV_ATU_GROUP_ON:
        return (phase == 0u) ? (sel ? 0x0006u : 0x0001u) : 0u;
    default:
        return 0u;
    }
}

static int atu_relay_write(DrvAtuGroup g, uint16_t value) {
    const uint8_t buf[3] = { ATU_RELAY_REG_OUTPUT,
                             (uint8_t)(value & 0xFFu),
                             (uint8_t)(value >> 8) };
    int rc = Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, s_cfg[g].addr, buf, sizeof(buf),
                                  DRV_ATU_RELAY_XFER_TIMEOUT_MS);

    if (rc != 0) {
        LOG_WARN("write 0x%02X failed (%d)", (unsigned)s_cfg[g].addr, rc);
    }
    return rc;
}

/* Start the pulse sequence for g; its deferred request is consumed. */
static void atu_relay_begin(DrvAtuGroup g, uint8_t sel) {
    s_want[g]  = -1;
    s_active   = g;
    s_sel      = sel;
    s_phase    = 1u;
    s_busy     = true;
    s_deadline = Bsp_Time_GetMs() + s_cfg[g].step_ms;

    /* Phase 0 energizes the coil bank with the fill mask immediately. */
    atu_relay_write(g, atu_relay_value(g, 0u, sel));
}

/* Run the oldest deferred request that still differs from the relay state. */
static void atu_relay_start_next(void) {
    for (uint8_t g = 0u; g < DRV_ATU_GROUP_COUNT; ++g) {
        if (s_want[g] < 0) {
            continue;
        }
        if (s_cached[g] == s_want[g]) {
            s_want[g] = -1;
            continue;
        }
        atu_relay_begin((DrvAtuGroup)g, (uint8_t)s_want[g]);
        return;
    }
}

void Drv_AtuRelay_Request(DrvAtuGroup g, uint8_t sel) {
    if (g >= DRV_ATU_GROUP_COUNT) {
        return;
    }

    /* Never interrupt a running pulse: an incomplete sequence would leave the
     * latching relay in an undefined state and the group cache unwritten. */
    if (!s_busy) {
        if (s_cached[g] == (int16_t)sel) {
            s_want[g] = -1;
            return; /* already at this selection: no pulse, no dwell */
        }
        atu_relay_begin(g, sel);
        return;
    }

    if (s_active == g) {
        if (s_sel == sel) {
            s_want[g] = -1;
        } else {
            s_want[g] = (int16_t)sel; /* re-run after the current pulse */
        }
        return;
    }
    if (s_cached[g] == (int16_t)sel) {
        s_want[g] = -1; /* the pending move is cancelled by returning home */
        return;
    }
    s_want[g] = (int16_t)sel;
}

bool Drv_AtuRelay_Busy(void) {
    return s_busy;
}

void Drv_AtuRelay_Tick(void) {
    if (!s_busy) {
        return;
    }
    if ((int32_t)(Bsp_Time_GetMs() - s_deadline) < 0) {
        return;
    }

    if (s_phase < s_cfg[s_active].phases) {
        atu_relay_write(s_active, atu_relay_value(s_active, s_phase, s_sel));
        s_phase++;
        s_deadline += s_cfg[s_active].step_ms;
        return;
    }

    /* Dwell after the last write elapsed: the sequence is latched. */
    s_cached[s_active] = (int16_t)s_sel;
    s_busy             = false;

    if (s_want[s_active] >= 0) {
        uint8_t sel = (uint8_t)s_want[s_active];

        if (s_cached[s_active] == (int16_t)sel) {
            s_want[s_active] = -1;
            atu_relay_start_next();
        } else {
            atu_relay_begin(s_active, sel);
        }
        return;
    }
    atu_relay_start_next();
}

int Drv_AtuRelay_Init(void) {
    /* 0x24 is the L/C expander; drv_pca9535 leaves it unconfigured. Force the
     * output, polarity and configuration registers to zero (all pins outputs,
     * no inversion). They are four register pairs, so each one is written
     * separately: a multi-byte write only toggles inside the addressed pair. */
    int rc = 0;

    for (uint8_t reg = ATU_RELAY_REG_OUTPUT; reg <= 0x07u; ++reg) {
        const uint8_t buf[2] = { reg, 0u };

        rc = Bsp_I2c_WriteTimeout(BSP_I2C_BUS_1, ATU_RELAY_ADDR_LC, buf,
                                  sizeof(buf), DRV_ATU_RELAY_XFER_TIMEOUT_MS);
        if (rc != 0) {
            LOG_WARN("init 0x%02X reg %u failed (%d)",
                     (unsigned)ATU_RELAY_ADDR_LC, (unsigned)reg, rc);
            break;
        }
    }

    for (size_t i = 0; i < sizeof(s_cached) / sizeof(s_cached[0]); ++i) {
        s_cached[i] = -1;
        s_want[i]   = -1;
    }
    s_busy = false;
    return rc;
}
