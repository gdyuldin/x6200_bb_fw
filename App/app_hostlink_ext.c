#include "app_hostlink_ext.h"

#include <string.h>

#include "app_scheduler.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_memory.h"

#define LOG_TAG "app.hostlink_ext"

/* Extended register map (see app_hostlink_ext.h). */
#define HOSTLINK_EXT_MAGIC_ADDR   0x4000u
#define HOSTLINK_EXT_MAGIC_LEN    8u
#define HOSTLINK_EXT_VERSION_ADDR 0x4008u
#define HOSTLINK_EXT_CMD_ADDR     0x4100u
#define HOSTLINK_EXT_CMD_WORDS    64u
#define HOSTLINK_EXT_CMD_BYTES    (HOSTLINK_EXT_CMD_WORDS * 4u)

/* Opens the page; a command block is ignored until this matches. */
static const uint8_t s_ext_magic[HOSTLINK_EXT_MAGIC_LEN] = {
    'R', '2', 'R', 'F', 'E', '_', 'F', 'W'
};

static uint8_t           s_magic_rx[HOSTLINK_EXT_MAGIC_LEN] BSP_RAM_D1_BSS;
static uint32_t          s_version_rx;
static uint32_t          s_knobs_rx[HOSTLINK_EXT_CMD_WORDS] BSP_RAM_D1_BSS;
static uint32_t          s_knobs_shadow[HOSTLINK_EXT_CMD_WORDS] BSP_RAM_D1_BSS;
static volatile bool     s_magic_dirty;
static volatile bool     s_version_dirty;
static volatile bool     s_knobs_dirty;
static uint32_t          s_version;
static bool              s_ext_active;

/* NULL = command not handled yet; fill in as support is added. */
static void (*const s_knob_handlers[HOSTLINK_EXT_CMD_WORDS])(uint32_t) = {
};

bool App_HostLinkExt_Write(uint16_t reg, const uint8_t *data, size_t len) {
    if (data == 0 || len == 0u) {
        return false;
    }

    if (reg == HOSTLINK_EXT_MAGIC_ADDR) {
        size_t n = (len < HOSTLINK_EXT_MAGIC_LEN) ? len
                                                  : HOSTLINK_EXT_MAGIC_LEN;
        memcpy(s_magic_rx, data, n);
        /* A magic write may carry the version word in the same transaction. */
        if (len >= HOSTLINK_EXT_MAGIC_LEN + 4u) {
            memcpy(&s_version_rx, data + HOSTLINK_EXT_MAGIC_LEN, 4u);
            s_version_dirty = true;
        }
        s_magic_dirty = true;
        App_Scheduler_Notify(APP_SCHED_EV_HOSTLINK_EXT);
        return true;
    }

    if (reg == HOSTLINK_EXT_VERSION_ADDR) {
        size_t n = (len < 4u) ? len : 4u;
        memcpy(&s_version_rx, data, n);
        s_version_dirty = true;
        App_Scheduler_Notify(APP_SCHED_EV_HOSTLINK_EXT);
        return true;
    }

    if (reg >= HOSTLINK_EXT_CMD_ADDR &&
        reg < HOSTLINK_EXT_CMD_ADDR + HOSTLINK_EXT_CMD_BYTES) {
        uint8_t *cmd = (uint8_t *)s_knobs_rx;
        for (size_t i = 0; i < len; ++i) {
            cmd[(size_t)(reg - HOSTLINK_EXT_CMD_ADDR + i) %
                HOSTLINK_EXT_CMD_BYTES] = data[i];
        }
        s_knobs_dirty = true;
        App_Scheduler_Notify(APP_SCHED_EV_HOSTLINK_EXT);
        return true;
    }

    return false;
}

static void hostlink_ext_task(void) {
    if (s_magic_dirty) {
        uint8_t  magic[HOSTLINK_EXT_MAGIC_LEN];
        uint32_t basepri = Bsp_Irq_SaveLow();
        memcpy(magic, s_magic_rx, sizeof(magic));
        s_magic_dirty = false;
        Bsp_Irq_RestoreLow(basepri);

        if (memcmp(magic, s_ext_magic, HOSTLINK_EXT_MAGIC_LEN) == 0) {
            if (!s_ext_active) {
                s_ext_active = true;
                LOG_INFO("ext protocol enabled");
            }
        } else {
            LOG_DEBUG("ext magic mismatch, page stays closed");
        }
    }

    if (s_version_dirty) {
        uint32_t basepri = Bsp_Irq_SaveLow();
        s_version        = s_version_rx;
        s_version_dirty  = false;
        Bsp_Irq_RestoreLow(basepri);

        if (s_ext_active) {
            LOG_INFO("ext layout version %u", (unsigned)s_version);
        }
    }

    if (s_knobs_dirty) {
        uint32_t snapshot[HOSTLINK_EXT_CMD_WORDS];
        uint32_t basepri = Bsp_Irq_SaveLow();
        memcpy(snapshot, s_knobs_rx, sizeof(snapshot));
        s_knobs_dirty = false;
        Bsp_Irq_RestoreLow(basepri);

        for (uint32_t i = 0; i < HOSTLINK_EXT_CMD_WORDS; ++i) {
            if (snapshot[i] == s_knobs_shadow[i]) {
                continue;
            }
            s_knobs_shadow[i] = snapshot[i];
            if (!s_ext_active) {
                LOG_DEBUG("ext cmd[%u] ignored, page closed", (unsigned)i);
                continue;
            }
            if (s_knob_handlers[i] != 0) {
                s_knob_handlers[i](snapshot[i]);
            }
        }
    }
}

int App_HostLinkExt_Init(void) {
    memset(s_knobs_shadow, 0xFF, sizeof(s_knobs_shadow));
    s_magic_dirty   = false;
    s_version_dirty = false;
    s_knobs_dirty   = false;
    s_version       = 0u;
    s_ext_active    = false;

    return App_Scheduler_AddEvent(hostlink_ext_task, APP_SCHED_EV_HOSTLINK_EXT);
}
