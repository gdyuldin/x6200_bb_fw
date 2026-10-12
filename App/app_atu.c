#include "app_atu.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_alc.h"
#include "app_protect.h"
#include "app_ptt.h"
#include "app_radio.h"
#include "app_radio_state.h"
#include "app_scheduler.h"
#include "app_tx_monitor.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "drv_atu_relay.h"

#define LOG_TAG "app.atu"

#define ATU_CODE_MAX 63

_Static_assert(APP_ATU_TARGET > 1.0f, "atu target swr must exceed 1");
_Static_assert(APP_ATU_SUCCESS > APP_ATU_TARGET,
               "atu success swr must exceed target");
_Static_assert(APP_ATU_GAMMA_TARGET > 0.0f,
               "atu gamma target must be positive");
_Static_assert(APP_ATU_GAMMA_SUCCESS < 1.0f,
               "atu gamma success must be below 1");
_Static_assert(APP_ATU_GAMMA_PLATEAU > 0.0f,
               "atu gamma plateau must be positive");
_Static_assert(APP_ATU_GAMMA_BRANCH_CLOSE > 0.0f,
               "atu gamma branch close must be positive");
_Static_assert(APP_ATU_COARSE_ABORT_SWR > APP_ATU_COARSE_WORSE_SWR,
               "atu coarse abort swr must exceed worse swr");
_Static_assert(APP_ATU_COARSE_WORSE_SWR > 0.0f,
               "atu coarse worse swr must be positive");

/* Top-level sequence. */
typedef enum {
    ATU_STATE_IDLE = 0,
    ATU_STATE_TUNER_ON,
    ATU_STATE_TX_ON,
    ATU_STATE_ZERO_NET,
    ATU_STATE_PRE,
    ATU_STATE_FINE,
    ATU_STATE_FINALIZE,
    ATU_STATE_ABORT
} AtuState;

/* One candidate measurement: drop the carrier, apply the network, switch the
 * carrier back on, settle, take N SWR samples. */
typedef enum {
    MPHASE_BLANK = 0,
    MPHASE_APPLY,
    MPHASE_SETTLE,
    MPHASE_SAMPLE,
    MPHASE_DONE
} MPhase;

/* Fine (coordinate descent) phases. */
typedef enum {
    F_ORDER = 0, /* pick the axis order once, by one improving probe per axis */
    F_DIR,       /* detect the walk direction of the current axis */
    F_WALK,      /* walk the current axis while SWR drops */
    F_EVAL,      /* end of a pass: target / next pass / fallback */
    F_DONE
} FinePhase;

/* Coarse L/C code table (0..48, 6-bit). */
static const uint8_t s_pre_code[8] = { 0u, 1u, 2u, 4u, 8u, 16u, 32u, 48u };

/* Top-level state machine. */
static struct {
    volatile bool tuning;
    AtuState      state;
    bool          started;
    uint32_t      deadline;
    uint32_t      deadline2;
    bool          tuner_on; /* host-requested ATU in the TX path */
    uint32_t      result_params;
    bool          pending_apply;
} s_top;

/* Apply-only sub-machine (ZERO_NET/FINALIZE/host direct). */
static struct {
    uint8_t  group[3];
    uint8_t  sel[3];
    uint8_t  idx;
    bool     req;
    bool     active;
    uint32_t deadline;
} s_apply;

/* Measure sub-machine. */
static struct {
    MPhase   phase;
    uint8_t  group[3];
    uint8_t  sel[3];
    uint8_t  idx;
    bool     req;
    uint32_t settle;
    uint8_t  samples;
    float    sum;
    uint8_t  count;
    float    prev;
    float    result;
    uint32_t deadline;
} s_meas;

/* Coarse search. */
static struct {
    uint8_t pre_sw;
    uint8_t pre_li;
    uint8_t pre_ci;
    bool    wait;
    float   gamma[2];
    uint8_t l[2];
    uint8_t c[2];
    float   best_gamma;
    uint8_t best_sw;
    uint8_t best_l;
    uint8_t best_c;
    float   row_best;
    float   row_ref;
    bool    row_ref_valid;
    uint8_t worse_streak;
} s_coarse;

/* Fine search (coordinate descent). Axis index 0 = C, 1 = L. */
static struct {
    uint8_t   sw;        /* fixed switch for this run */
    uint8_t   branch;    /* coarse branch the run started from */
    float     best;      /* best gamma reached in this run */
    int       best_l;    /* codes at best (committed to the champion) */
    int       best_c;
    int       l;         /* current working codes */
    int       c;
    int8_t    dir[2];    /* walk direction per axis, 0 = none */
    uint8_t   step[2];   /* current step size per axis */
float     pres[2];   /* one-step probe gamma per axis (ordering) */
    uint8_t   order[2];  /* processing order: 0 = C, 1 = L */
    uint8_t   axis;      /* position in order[] for the current walk */
    uint8_t   oprobe;    /* ordering probe index 0 = C, 1 = L */
    uint8_t   probe;     /* direction probe substep of the current axis */
    uint8_t   dir_ax;    /* axis a direction probe is running for */
    uint8_t   pass;
    bool      started;   /* a fine measurement is in flight */
    bool      fallback;
    int       cand_l;    /* candidate in flight */
    int       cand_c;
    FinePhase phase;
} s_fine;

/* Champion network across all fine runs (published at the very end). */
static struct {
    float   gamma;
    uint8_t sw;
    uint8_t l;
    uint8_t c;
} s_fin;

static bool atu_elapsed(uint32_t deadline) {
    return (int32_t)(Bsp_Time_GetMs() - deadline) >= 0;
}

static float atu_swr_from_gamma(float gamma) {
    if (gamma < 1.0f) {
        return (1.0f + gamma) / (1.0f - gamma);
    }
    return 99.0f;
}

static void atu_goto(AtuState st) {
    s_top.state   = st;
    s_top.started = false;
}

static void atu_publish(uint8_t sw, uint8_t l, uint8_t c) {
    s_top.result_params = ((uint32_t)c & APP_ATU_PARAMS_C_MASK) |
                          (((uint32_t)l << APP_ATU_PARAMS_L_SHIFT) &
                           APP_ATU_PARAMS_L_MASK) |
                          (sw ? APP_ATU_PARAMS_SW_MASK : 0u);
}

static void atu_recover(void) {
    App_Radio_SetTuneCarrier(false);
    App_Ptt_SetAtuTune(false);
    App_Alc_SetAtuActive(false);
    Drv_AtuRelay_Request(DRV_ATU_GROUP_ON, s_top.tuner_on ? 1u : 0u);
    s_top.tuning = false;
    atu_goto(ATU_STATE_IDLE);
}

/* --- apply-only sub-machine --------------------------------------------- */

static void apply_begin(uint8_t sw, uint8_t l, uint8_t c, bool mute) {
    s_apply.group[0] = (uint8_t)DRV_ATU_GROUP_SWITCH;
    s_apply.sel[0]   = sw;
    s_apply.group[1] = (uint8_t)DRV_ATU_GROUP_L;
    s_apply.sel[1]   = l;
    s_apply.group[2] = (uint8_t)DRV_ATU_GROUP_C;
    s_apply.sel[2]   = c;
    s_apply.idx      = 0u;
    s_apply.req      = false;
    s_apply.deadline = Bsp_Time_GetMs() + (mute ? APP_ATU_BLANK_MS : 0u);
    s_apply.active   = true;
    if (mute) {
        App_Radio_SetTuneCarrier(false);
    }
}

static bool apply_step(void) {
    if (!s_apply.active) {
        return true;
    }
    if (!atu_elapsed(s_apply.deadline)) {
        return false; /* carrier blank still running */
    }
    if (s_apply.idx >= 3u) {
        s_apply.active = false;
        return true;
    }
    if (!s_apply.req) {
        Drv_AtuRelay_Request((DrvAtuGroup)s_apply.group[s_apply.idx],
                             s_apply.sel[s_apply.idx]);
        s_apply.req = true;
    }
    if (!Drv_AtuRelay_Busy()) {
        s_apply.idx++;
        s_apply.req = false;
    }
    return false;
}

/* --- measure sub-machine ------------------------------------------------ */

static void measure_begin(uint8_t sw, uint8_t l, uint8_t c, uint32_t settle_ms,
                          uint8_t samples) {
    s_meas.group[0] = (uint8_t)DRV_ATU_GROUP_SWITCH;
    s_meas.sel[0]   = sw;
    s_meas.group[1] = (uint8_t)DRV_ATU_GROUP_L;
    s_meas.sel[1]   = l;
    s_meas.group[2] = (uint8_t)DRV_ATU_GROUP_C;
    s_meas.sel[2]   = c;
    s_meas.idx      = 0u;
    s_meas.req      = false;
    s_meas.settle   = settle_ms;
    s_meas.samples  = (samples == 0u) ? 1u : samples;
    s_meas.sum      = 0.0f;
    s_meas.count    = 0u;
    s_meas.prev     = 0.0f;
    s_meas.result   = 0.0f;
    s_meas.deadline = Bsp_Time_GetMs() + APP_ATU_BLANK_MS;
    s_meas.phase    = MPHASE_BLANK;
    App_Radio_SetTuneCarrier(false);
}

static bool measure_step(void) {
    switch (s_meas.phase) {
    case MPHASE_BLANK:
        if (!atu_elapsed(s_meas.deadline)) {
            return false;
        }
        s_meas.phase = MPHASE_APPLY;
        return false;

    case MPHASE_APPLY:
        if (s_meas.idx >= 3u) {
            App_Radio_SetTuneCarrier(true);
            s_meas.deadline = Bsp_Time_GetMs() + s_meas.settle;
            s_meas.phase    = MPHASE_SETTLE;
            return false;
        }
        if (!s_meas.req) {
            Drv_AtuRelay_Request((DrvAtuGroup)s_meas.group[s_meas.idx],
                                 s_meas.sel[s_meas.idx]);
            s_meas.req = true;
        }
        if (!Drv_AtuRelay_Busy()) {
            s_meas.idx++;
            s_meas.req = false;
        }
        return false;

    case MPHASE_SETTLE:
        if (!atu_elapsed(s_meas.deadline)) {
            return false;
        }
        App_TxMonitor_AtuArm(APP_ATU_WINDOW_MS);
        s_meas.deadline = Bsp_Time_GetMs() + APP_ATU_FAST_STEP_MS;
        s_meas.phase    = MPHASE_SAMPLE;
        return false;

    case MPHASE_SAMPLE: {
        float v;

        if (!atu_elapsed(s_meas.deadline)) {
            return false;
        }
        if (App_TxMonitor_AtuTake(&v)) {
            s_meas.count++;
            s_meas.sum += v;
            if (s_meas.count >= s_meas.samples) {
                s_meas.result = s_meas.sum / (float)s_meas.count;
                s_meas.phase  = MPHASE_DONE;
                return true;
            }
            /* Early exit once consecutive samples are close. */
            if (s_meas.count >= 2u &&
                fabsf(v - s_meas.prev) < APP_ATU_GAMMA_SAMPLE_STABLE) {
                s_meas.result = v;
                s_meas.phase  = MPHASE_DONE;
                return true;
            }
            s_meas.prev = v;
            App_TxMonitor_AtuArm(APP_ATU_WINDOW_MS);
        }
        s_meas.deadline = Bsp_Time_GetMs() + APP_ATU_FAST_STEP_MS;
        return false;
    }

    case MPHASE_DONE:
    default:
        return true;
    }
}

/* --- coarse search ------------------------------------------------------ */

static void coarse_begin(void) {
    s_coarse.pre_sw  = 0u;
    s_coarse.pre_li  = 0u;
    s_coarse.pre_ci  = 0u;
    s_coarse.wait    = false;
    for (uint8_t b = 0u; b < 2u; ++b) {
        s_coarse.gamma[b] = APP_TX_MON_ATU_GAMMA_INVALID;
        s_coarse.l[b]     = 0u;
        s_coarse.c[b]     = 0u;
    }
    s_coarse.best_gamma = APP_TX_MON_ATU_GAMMA_INVALID;
    s_coarse.best_sw    = 0u;
    s_coarse.best_l     = 0u;
    s_coarse.best_c     = 0u;
    s_coarse.row_best      = APP_TX_MON_ATU_GAMMA_INVALID;
    s_coarse.row_ref       = APP_TX_MON_ATU_GAMMA_INVALID;
    s_coarse.row_ref_valid = false;
    s_coarse.worse_streak  = 0u;
}

/* Returns true when the sweep is finished (all combos or an early target). */
static bool coarse_step(void) {
    uint8_t l;
    uint8_t c;

    l = s_pre_code[s_coarse.pre_li];
    c = s_pre_code[s_coarse.pre_ci];

    if (!s_coarse.wait) {
        if (s_coarse.pre_ci == 0u) {
            s_coarse.row_best      = APP_TX_MON_ATU_GAMMA_INVALID;
            s_coarse.row_ref       = s_coarse.gamma[s_coarse.pre_sw];
            s_coarse.row_ref_valid = s_coarse.row_ref <
                                     APP_TX_MON_ATU_GAMMA_INVALID;
        }
        measure_begin(s_coarse.pre_sw, l, c, APP_ATU_SETTLE_MS, 1u);
        s_coarse.wait = true;
        return false;
    }
    if (!measure_step()) {
        return false;
    }

    s_coarse.wait = false;

    LOG_DEBUG("atu coarse sw %u L %u C %u gamma %f swr %f",
              (unsigned)s_coarse.pre_sw, (unsigned)l, (unsigned)c,
              s_meas.result, atu_swr_from_gamma(s_meas.result));

    if (s_meas.result < s_coarse.row_best) {
        s_coarse.row_best = s_meas.result;
    }
    if (s_meas.result < s_coarse.gamma[s_coarse.pre_sw]) {
        s_coarse.gamma[s_coarse.pre_sw] = s_meas.result;
        s_coarse.l[s_coarse.pre_sw]     = l;
        s_coarse.c[s_coarse.pre_sw]     = c;
    }
    if (s_meas.result < s_coarse.best_gamma) {
        s_coarse.best_gamma = s_meas.result;
        s_coarse.best_sw    = s_coarse.pre_sw;
        s_coarse.best_l     = l;
        s_coarse.best_c     = c;
    }

    if (s_coarse.best_gamma <= APP_ATU_GAMMA_TARGET) {
        return true; /* early exit */
    }

    /* Row complete: drop the rest of this branch when the row trend says the
     * minimum is behind. Only the first branch is cut; the last one has no
     * alternative to switch to. */
    if (s_coarse.pre_ci == 7u && s_coarse.pre_sw == 0u &&
        s_coarse.row_ref_valid &&
        s_coarse.row_best < APP_TX_MON_ATU_GAMMA_INVALID) {
        float ref_swr = atu_swr_from_gamma(s_coarse.row_ref);
        float row_swr = atu_swr_from_gamma(s_coarse.row_best);
        bool  abort;

        if (row_swr > ref_swr + APP_ATU_COARSE_WORSE_SWR) {
            s_coarse.worse_streak++;
        } else {
            s_coarse.worse_streak = 0u;
        }
        abort = (s_coarse.worse_streak >= APP_ATU_COARSE_WORSE_ROWS) ||
                (row_swr > ref_swr + APP_ATU_COARSE_ABORT_SWR);
        if (abort) {
            LOG_DEBUG("atu coarse abort sw %u L %u row swr %f ref swr %f "
                      "streak %u",
                      (unsigned)s_coarse.pre_sw, (unsigned)l, row_swr,
                      ref_swr, (unsigned)s_coarse.worse_streak);
            s_coarse.pre_ci        = 0u;
            s_coarse.pre_li        = 0u;
            s_coarse.pre_sw        = (uint8_t)(s_coarse.pre_sw + 1u);
            s_coarse.worse_streak  = 0u;
            s_coarse.row_ref_valid = false;
            return s_coarse.pre_sw >= 2u;
        }
    }

    s_coarse.pre_ci++;
    if (s_coarse.pre_ci >= 8u) {
        s_coarse.pre_ci = 0u;
        s_coarse.pre_li++;
        if (s_coarse.pre_li >= 8u) {
            s_coarse.pre_li      = 0u;
            s_coarse.pre_sw++;
            s_coarse.worse_streak = 0u;
        }
    }
    return s_coarse.pre_sw >= 2u;
}

/* --- fine search -------------------------------------------------------- */

static void fine_commit_run(void) {
    if (s_fine.best < s_fin.gamma) {
        s_fin.gamma = s_fine.best;
        s_fin.sw    = s_fine.sw;
        s_fin.l     = (uint8_t)s_fine.best_l;
        s_fin.c     = (uint8_t)s_fine.best_c;
    }
}

/* Seed a fine run from a coarse branch; also (re)initialise the phase fields. */
static void fine_seed_run(uint8_t branch) {
    s_fine.sw        = branch;
    s_fine.branch    = branch;
    s_fine.best      = s_coarse.gamma[branch];
    s_fine.best_l    = s_coarse.l[branch];
    s_fine.best_c    = s_coarse.c[branch];
    s_fine.l         = s_coarse.l[branch];
    s_fine.c         = s_coarse.c[branch];
    s_fine.dir[0]    = 0;
    s_fine.dir[1]    = 0;
    s_fine.step[0]   = (uint8_t)APP_ATU_FINE_STEP;
    s_fine.step[1]   = (uint8_t)APP_ATU_FINE_STEP;
    s_fine.pres[0]   = APP_TX_MON_ATU_GAMMA_INVALID;
    s_fine.pres[1]   = APP_TX_MON_ATU_GAMMA_INVALID;
    s_fine.order[0]  = 0u;
    s_fine.order[1]  = 1u;
    s_fine.axis      = 0u;
    s_fine.oprobe    = 0u;
    s_fine.probe     = 0u;
    s_fine.dir_ax    = 0u;
    s_fine.pass      = 1u;
    s_fine.started   = false;
    s_fine.phase     = F_ORDER;
}

/* Determine the walk direction of axis ax; returns true when finished. */
static bool fine_dir_tick(void) {
    uint8_t ax   = s_fine.dir_ax;
    int     step = (int)s_fine.step[ax];

    if (s_fine.started) {
        if (!measure_step()) {
            return false;
        }
        s_fine.started = false;
        LOG_DEBUG("atu fine dir sw %u L %u C %u gamma %f swr %f",
                  (unsigned)s_fine.sw, (unsigned)s_fine.cand_l,
                  (unsigned)s_fine.cand_c, s_meas.result,
                  atu_swr_from_gamma(s_meas.result));
        if (s_meas.result < s_fine.best) {
            s_fine.best     = s_meas.result;
            s_fine.best_l   = s_fine.cand_l;
            s_fine.best_c   = s_fine.cand_c;
            s_fine.l        = s_fine.cand_l;
            s_fine.c        = s_fine.cand_c;
            s_fine.dir[ax]  = (int8_t)((s_fine.probe == 0u) ? step : -step);
            s_fine.pres[ax] = s_meas.result;
            return true;
        }
        s_fine.probe++;
    }

    for (; s_fine.probe < 2u; s_fine.probe++) {
        int base  = (ax == 0u) ? s_fine.c : s_fine.l;
        int delta = (s_fine.probe == 0u) ? step : -step;
        int cand  = base + delta;

        if (cand < 0) {
            cand = 0;
        } else if (cand > ATU_CODE_MAX) {
            cand = ATU_CODE_MAX;
        }
        if (cand == base) {
            continue;
        }
        if (ax == 0u) {
            s_fine.cand_c = cand;
            s_fine.cand_l = s_fine.l;
        } else {
            s_fine.cand_l = cand;
            s_fine.cand_c = s_fine.c;
        }
        measure_begin(s_fine.sw, (uint8_t)s_fine.cand_l, (uint8_t)s_fine.cand_c,
                      APP_ATU_FINE_SETTLE_MS, APP_ATU_FINE_SAMPLES);
        s_fine.started = true;
        return false;
    }

    s_fine.dir[ax]  = 0;
    s_fine.pres[ax] = s_fine.best;
    return true;
}

/* Walk the current axis; returns true when the axis is done. */
static bool fine_walk_tick(void) {
    uint8_t ax = s_fine.order[s_fine.axis];

    if (s_fine.started) {
        if (!measure_step()) {
            return false;
        }
        s_fine.started = false;
        LOG_DEBUG("atu fine walk sw %u L %u C %u gamma %f swr %f",
                  (unsigned)s_fine.sw, (unsigned)s_fine.cand_l,
                  (unsigned)s_fine.cand_c, s_meas.result,
                  atu_swr_from_gamma(s_meas.result));
        if (s_meas.result < s_fine.best) {
            s_fine.best   = s_meas.result;
            s_fine.best_l = s_fine.cand_l;
            s_fine.best_c = s_fine.cand_c;
            s_fine.l      = s_fine.cand_l;
            s_fine.c      = s_fine.cand_c;
            return false;
        }
        if (s_meas.result - s_fine.best < APP_ATU_GAMMA_PLATEAU) {
            if (s_fine.step[ax] < APP_ATU_FINE_STEP_MAX) {
                uint8_t next = (uint8_t)(s_fine.step[ax] * 2u);
                if (next > (uint8_t)APP_ATU_FINE_STEP_MAX) {
                    next = (uint8_t)APP_ATU_FINE_STEP_MAX;
                }
                s_fine.step[ax] = next;
                return false; /* retry with a larger step */
            }
        }
        return true;
    }

    if (s_fine.dir[ax] == 0) {
        return true;
    }
    {
        int base = (ax == 0u) ? s_fine.c : s_fine.l;
        int cand = base + s_fine.dir[ax];

        if (cand < 0 || cand > ATU_CODE_MAX) {
            return true;
        }
        if (ax == 0u) {
            s_fine.cand_c = cand;
            s_fine.cand_l = s_fine.l;
        } else {
            s_fine.cand_l = cand;
            s_fine.cand_c = s_fine.c;
        }
        measure_begin(s_fine.sw, (uint8_t)s_fine.cand_l, (uint8_t)s_fine.cand_c,
                      APP_ATU_FINE_SETTLE_MS, APP_ATU_FINE_SAMPLES);
        s_fine.started = true;
        return false;
    }
}

/* Pick the axis order once: probe one improving step on each axis. */
static void fine_order_tick(void) {
    if (s_fine.oprobe < 2u) {
        s_fine.dir_ax = s_fine.oprobe;
        if (fine_dir_tick()) {
            s_fine.oprobe++;
            s_fine.probe   = 0u;
            s_fine.started = false;
        }
        return;
    }
    if (s_fine.pres[0] <= s_fine.pres[1]) {
        s_fine.order[0] = 0u;
        s_fine.order[1] = 1u;
    } else {
        s_fine.order[0] = 1u;
        s_fine.order[1] = 0u;
    }
    s_fine.axis  = 0u;
    s_fine.phase = F_WALK;
}

static void fine_adv(void) {
    s_fine.axis++;
    if (s_fine.axis < 2u) {
        if (s_fine.pass == 1u) {
            s_fine.phase = F_WALK; /* direction already known from ordering */
        } else {
            s_fine.dir_ax  = s_fine.order[s_fine.axis];
            s_fine.probe   = 0u;
            s_fine.started = false;
            s_fine.phase   = F_DIR;
        }
    } else {
        s_fine.phase = F_EVAL;
    }
}

static void fine_eval(void) {
    LOG_DEBUG("atu fine run sw %u L %u C %u best gamma %f swr %f",
              (unsigned)s_fine.sw, (unsigned)s_fine.best_l,
              (unsigned)s_fine.best_c, s_fine.best,
              atu_swr_from_gamma(s_fine.best));
    if (s_fine.best <= APP_ATU_GAMMA_TARGET) {
        fine_commit_run();
        atu_goto(ATU_STATE_FINALIZE);
        return;
    }
    if (s_fine.pass < APP_ATU_FINE_PASSES) {
        s_fine.pass++;
        s_fine.axis    = 0u;
        s_fine.dir[0]  = 0;
        s_fine.dir[1]  = 0;
        s_fine.step[0] = (uint8_t)APP_ATU_FINE_STEP;
        s_fine.step[1] = (uint8_t)APP_ATU_FINE_STEP;
        s_fine.dir_ax  = s_fine.order[0];
        s_fine.probe   = 0u;
        s_fine.started = false;
        s_fine.phase   = F_DIR;
        return;
    }
    if (!s_fine.fallback) {
        float diff = s_coarse.gamma[0] - s_coarse.gamma[1];

        if (diff < 0.0f) {
            diff = -diff;
        }
        if (diff <= APP_ATU_GAMMA_BRANCH_CLOSE) {
            s_fine.fallback = true;
            fine_commit_run();
            fine_seed_run((uint8_t)(1u - s_fine.branch));
            return;
        }
    }
    fine_commit_run();
    atu_goto(ATU_STATE_FINALIZE);
}

static void fine_tick(void) {
    switch (s_fine.phase) {
    case F_ORDER:
        fine_order_tick();
        break;
    case F_DIR:
        if (fine_dir_tick()) {
            s_fine.phase = F_WALK;
        }
        break;
    case F_WALK:
        if (fine_walk_tick()) {
            fine_adv();
        }
        break;
    case F_EVAL:
        fine_eval();
        break;
    case F_DONE:
    default:
        break;
    }
}

/* --- top-level tick ----------------------------------------------------- */

static void Atu_Tick(void) {
    Drv_AtuRelay_Tick();

    /* Host direct-apply (outside a tune) shares the apply sub-machine. */
    if (s_top.pending_apply && !s_top.tuning) {
        if (apply_step()) {
            s_top.pending_apply = false;
        }
    }
    if (!s_top.tuning) {
        return;
    }

    switch (s_top.state) {
    case ATU_STATE_TUNER_ON:
        if (!s_top.started) {
            Drv_AtuRelay_Request(DRV_ATU_GROUP_ON, 1u);
            App_Alc_SetAtuActive(true);
            s_top.deadline = Bsp_Time_GetMs() + 10u;
            s_top.started  = true;
            return;
        }
        if (!Drv_AtuRelay_Busy() && atu_elapsed(s_top.deadline)) {
            atu_goto(ATU_STATE_TX_ON);
        }
        break;

    case ATU_STATE_TX_ON:
        if (!s_top.started) {
            App_Ptt_SetAtuTune(true);
            s_top.deadline  = Bsp_Time_GetMs() + 2000u;
            s_top.deadline2 = 0u;
            s_top.started   = true;
            return;
        }
        if (App_Protect_IsTxInhibited()) {
            LOG_WARN("atu abort: tx inhibited");
            atu_goto(ATU_STATE_ABORT);
            break;
        }
        if (App_Radio_GetRadioMode() == APP_RADIO_MODE_TX) {
            if (s_top.deadline2 == 0u) {
                /* Let the TX path settle before the first relay move; the
                 * carrier itself is off until the first measurement. */
                s_top.deadline2 = Bsp_Time_GetMs() + 100u;
            }
            if (atu_elapsed(s_top.deadline2)) {
                atu_goto(ATU_STATE_ZERO_NET);
            }
        } else if (atu_elapsed(s_top.deadline)) {
            LOG_WARN("atu abort: tx did not start");
            atu_goto(ATU_STATE_ABORT);
        }
        break;

    case ATU_STATE_ZERO_NET:
        if (!s_top.started) {
            apply_begin(0u, 0u, 0u, true);
            s_top.deadline = Bsp_Time_GetMs() + 500u;
            s_top.started  = true;
            return;
        }
        if (apply_step() && atu_elapsed(s_top.deadline)) {
            atu_goto(ATU_STATE_PRE);
        }
        break;

    case ATU_STATE_PRE:
        if (!s_top.started) {
            coarse_begin();
            s_top.started = true;
            return;
        }
        if (coarse_step()) {
            atu_goto(ATU_STATE_FINE);
        }
        break;

    case ATU_STATE_FINE:
        if (!s_top.started) {
            s_top.started = true;
            if (s_coarse.best_gamma <= APP_ATU_GAMMA_TARGET) {
                LOG_INFO("atu coarse best gamma %f swr %f <= target, "
                         "fine skipped, L %u C %u sw %u",
                         s_coarse.best_gamma,
                         atu_swr_from_gamma(s_coarse.best_gamma),
                         (unsigned)s_coarse.best_l,
                         (unsigned)s_coarse.best_c,
                         (unsigned)s_coarse.best_sw);
                s_fin.gamma = s_coarse.best_gamma;
                s_fin.sw    = s_coarse.best_sw;
                s_fin.l     = s_coarse.best_l;
                s_fin.c     = s_coarse.best_c;
                atu_goto(ATU_STATE_FINALIZE);
                return;
            }
            LOG_INFO("atu coarse best gamma %f swr %f > target, fine start, "
                     "L %u C %u sw %u",
                     s_coarse.best_gamma,
                     atu_swr_from_gamma(s_coarse.best_gamma),
                     (unsigned)s_coarse.best_l,
                     (unsigned)s_coarse.best_c,
                     (unsigned)s_coarse.best_sw);
            fine_seed_run(s_coarse.best_sw);
        }
        fine_tick();
        break;

    case ATU_STATE_FINALIZE:
        if (!s_top.started) {
            LOG_INFO("atu final gamma %f swr %f", s_fin.gamma,
                     atu_swr_from_gamma(s_fin.gamma));
            apply_begin(s_fin.sw, s_fin.l, s_fin.c, true);
            atu_publish(s_fin.sw, s_fin.l, s_fin.c);
            s_top.deadline = Bsp_Time_GetMs() + 100u;
            s_top.started  = true;
            return;
        }
        if (apply_step() && atu_elapsed(s_top.deadline)) {
            float swr = atu_swr_from_gamma(s_fin.gamma);

            if (s_fin.gamma <= APP_ATU_GAMMA_SUCCESS) {
                LOG_INFO("atu done swr %f gamma %f L %u C %u sw %u", swr,
                         s_fin.gamma, (unsigned)s_fin.l, (unsigned)s_fin.c,
                         (unsigned)s_fin.sw);
                App_Alc_ResetSwrClamp();
            } else {
                LOG_WARN("atu failed swr %f gamma %f L %u C %u sw %u", swr,
                         s_fin.gamma, (unsigned)s_fin.l, (unsigned)s_fin.c,
                         (unsigned)s_fin.sw);
            }
            atu_recover();
        }
        break;

    case ATU_STATE_ABORT:
        if (!s_top.started) {
            App_Radio_SetTuneCarrier(false);
            s_top.deadline = Bsp_Time_GetMs() + APP_ATU_BLANK_MS;
            s_top.started  = true;
            return;
        }
        if (atu_elapsed(s_top.deadline)) {
            s_top.tuning = false; /* keep the best known result published */
            atu_recover();
        }
        break;

    case ATU_STATE_IDLE:
    default:
        break;
    }
}

/* --- public API --------------------------------------------------------- */

int App_Atu_Init(void) {
    int rc;

    s_top.tuning        = false;
    s_top.state         = ATU_STATE_IDLE;
    s_top.started       = false;
    s_top.pending_apply = false;
    s_top.tuner_on      = false;
    s_top.result_params = 0u;

    rc = Drv_AtuRelay_Init();
    if (rc != 0) {
        LOG_WARN("relay init failed (%d)", rc);
    }
    if (App_Scheduler_Add(Atu_Tick, APP_ATU_TICK_MS) != 0) {
        LOG_ERROR("tick add failed");
        return -1;
    }
    LOG_INFO("atu ready");
    return 0;
}

void App_Atu_RequestTune(void) {
    if (s_top.tuning) {
        return;
    }
    if (App_RadioState_GetSwrScan()) {
        LOG_WARN("atu request refused, swr scan active");
        return;
    }
    s_top.pending_apply    = false;
    s_coarse.best_gamma    = APP_TX_MON_ATU_GAMMA_INVALID;
    s_coarse.best_sw       = 0u;
    s_coarse.best_l        = 0u;
    s_coarse.best_c        = 0u;
    s_coarse.row_best      = APP_TX_MON_ATU_GAMMA_INVALID;
    s_coarse.row_ref       = APP_TX_MON_ATU_GAMMA_INVALID;
    s_coarse.row_ref_valid = false;
    s_coarse.worse_streak  = 0u;
    s_fin.gamma            = APP_TX_MON_ATU_GAMMA_INVALID;
    s_fin.sw               = 0u;
    s_fin.l                = 0u;
    s_fin.c                = 0u;
    s_top.tuning           = true;
    atu_goto(ATU_STATE_TUNER_ON);
    LOG_INFO("atu tune start, freq %u", (unsigned)App_RadioState_GetTxFreq());
}

void App_Atu_ApplyNetwork(uint8_t c, uint8_t l, uint8_t sw) {
    if (s_top.tuning) {
        return;
    }
    if (l > ATU_CODE_MAX) {
        l = ATU_CODE_MAX;
    }
    if (c > ATU_CODE_MAX) {
        c = ATU_CODE_MAX;
    }
    sw = (sw != 0u) ? 1u : 0u;

    atu_publish(sw, l, c);
    LOG_INFO("atu net L %u C %u sw %u", (unsigned)l, (unsigned)c,
             (unsigned)sw);
    if (!s_top.tuner_on) {
        return; /* off: remember only, applied when switched on */
    }
    apply_begin(sw, l, c, false);
    s_top.pending_apply = true;
}

void App_Atu_SetOn(bool on) {
    if (s_top.tuner_on == on) {
        return;
    }
    s_top.tuner_on = on;
    LOG_DEBUG("atu on %u", (unsigned)on);
    if (s_top.tuning) {
        return; /* TXPATH is forced on while tuning, restored on recover */
    }
    Drv_AtuRelay_Request(DRV_ATU_GROUP_ON, on ? 1u : 0u);
    if (!on) {
        s_top.pending_apply = false; /* keep the current L/C/SW values */
        return;
    }
    apply_begin((uint8_t)((s_top.result_params & APP_ATU_PARAMS_SW_MASK) != 0u),
                (uint8_t)((s_top.result_params & APP_ATU_PARAMS_L_MASK) >>
                          APP_ATU_PARAMS_L_SHIFT),
                (uint8_t)(s_top.result_params & APP_ATU_PARAMS_C_MASK), false);
    s_top.pending_apply = true;
}

bool App_Atu_IsTuning(void) {
    return s_top.tuning;
}

uint32_t App_Atu_GetParams(void) {
    return s_top.result_params;
}
