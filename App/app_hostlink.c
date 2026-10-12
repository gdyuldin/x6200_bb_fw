#include "app_hostlink.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app_atu.h"
#include "app_hostcmd.h"
#include "app_hostlink_ext.h"
#include "app_nvram.h"
#include "app_periph.h"
#include "app_power.h"
#include "app_protect.h"
#include "app_ptt.h"
#include "app_radio.h"
#include "app_radio_state.h"
#include "app_scheduler.h"
#include "board_config.h"
#include "bsp_boot.h"
#include "bsp_i2c_slave.h"
#include "bsp_irq.h"
#include "bsp_log.h"
#include "bsp_memory.h"
#include "bsp_time.h"
#include "drv_fpga.h"
#include "drv_nvram.h"
#include "drv_pa_bias.h"
#include "sdr_agc.h"
#include "sdr_compressor.h"
#include "sdr_nb.h"
#include "sdr_notch.h"
#include "sdr_nr.h"

#define LOG_TAG "app.hostlink"

/* Register map. */
#define HOSTLINK_BASE_INFO_LEN   129u
#define HOSTLINK_BASE_INFO_END   0x0080u
#define HOSTLINK_CALIB_READ_ADDR 0x0100u
#define HOSTLINK_CALIB_READ_END  0x02FAu
#define HOSTLINK_READY_ADDR      0x2000u
#define HOSTLINK_CW_TEXT_ADDR    0x3000u
#define HOSTLINK_CW_TEXT_END     0x303Fu
#define HOSTLINK_CW_WPM_ADDR     0x3040u
#define HOSTLINK_CW_WPM_END      0x3043u
#define HOSTLINK_CALIB_WR_ADDR   0xF000u
#define HOSTLINK_CALIB_WR_END    0xF1FAu
#define HOSTLINK_CMD_ADDR        0xFFFEu

/* Command block: 0x0000..0x00D7, one 32-bit word per command at cmd*4. */
#define HOSTLINK_CMD_WORDS 54u
#define HOSTLINK_CMD_BYTES (HOSTLINK_CMD_WORDS * 4u)

#define HOSTLINK_LOW_BYTE_MASK 0x000000FFu

/* Command-block words are seeded to 0xFF, so an all-ones word means the host
 * never wrote it. */
#define HOSTLINK_CMD_WORD_UNSET 0xFFFFFFFFu

/* sple_atue_trx field bits. */
#define SPLE_ATUE_TRX_SPLIT          0x00000002u
#define SPLE_ATUE_TRX_VOICE_REC      0x00000008u
#define SPLE_ATUE_TRX_SWR_SCAN       0x00000010u
#define SPLE_ATUE_TRX_TUNE_FLAG      0x00000020u
#define SPLE_ATUE_TRX_RSVD_72        0x00000040u
#define SPLE_ATUE_TRX_ATUE           0x00001000u
#define SPLE_ATUE_TRX_TUNE           0x00002000u
#define SPLE_ATUE_TRX_MTX            0x00004000u
#define SPLE_ATUE_TRX_CAL            0x00008000u
#define SPLE_ATUE_TRX_POF            0x00010000u
#define SPLE_ATUE_TRX_IPTT           0x00040000u
#define SPLE_ATUE_TRX_CIVPTT         0x00080000u
#define SPLE_ATUE_TRX_CALTX          0x00100000u
#define SPLE_ATUE_TRX_BIAS_DRIVE_OFF 0x00200000u /* bit21 */
#define SPLE_ATUE_TRX_BIAS_FINAL_OFF 0x00400000u /* bit22 */
#define SPLE_ATUE_TRX_IQSWAP         0x00800000u

/* base-info field offsets and the maximum field length. */
#define HOSTLINK_FIELD_MAX 32u
#define HOSTLINK_VER1_OFF  0x01u
#define HOSTLINK_VER2_OFF  0x21u
#define HOSTLINK_FWVER_OFF 0x41u
#define HOSTLINK_FWDATE_OFF 0x61u

/* Host command codes. */
#define HOSTLINK_CMD_AGC      0x8002u
#define HOSTLINK_CMD_DUMP     0x8003u
#define HOSTLINK_CMD_ATU      0x8004u
#define HOSTLINK_CMD_CAL_SAVE 0x8005u
#define HOSTLINK_CMD_PWR_OFF  0x8009u
#define HOSTLINK_CMD_IAP      0x800Au

/* Command-block word indices and field masks we act on. */
typedef enum {
    HOSTLINK_CMD_IDX_VFOA_FREQ = 1u,
    HOSTLINK_CMD_IDX_VFOA_ATT  = 2u,
    HOSTLINK_CMD_IDX_VFOA_PRE  = 3u,
    HOSTLINK_CMD_IDX_VFOA_MODE = 4u,
    HOSTLINK_CMD_IDX_VFOA_AGC  = 5u,
    HOSTLINK_CMD_IDX_VFOB_FREQ = 7u,
    HOSTLINK_CMD_IDX_VFOB_ATT  = 8u,
    HOSTLINK_CMD_IDX_VFOB_PRE  = 9u,
    HOSTLINK_CMD_IDX_VFOB_MODE = 10u,
    HOSTLINK_CMD_IDX_VFOB_AGC  = 11u,
    HOSTLINK_CMD_IDX_SPLE_ATUE_TRX     = 12u,
    HOSTLINK_CMD_IDX_VFO_SEL   = 13u,
    HOSTLINK_CMD_IDX_RXVOL     = 14u,
    HOSTLINK_CMD_IDX_RFG_TXPWR = 15u,
    HOSTLINK_CMD_IDX_ATU_NETWORK = 17u,
    HOSTLINK_CMD_IDX_LING_LOUTG_IMICG_HMICG = 20u,
    HOSTLINK_CMD_IDX_MICSEL_PTTMODE_CHGE_SPMODE_AUXIQGEN_SQLTHR = 21u,
    HOSTLINK_CMD_IDX_NRTHR_NBW_NBTHR_NRE_NBE = 23u,
    HOSTLINK_CMD_IDX_DNFCNT_DNFWIDTH_DNFMODE = 24u,
    HOSTLINK_CMD_IDX_CMPLEVEL_CMPE = 25u,
    HOSTLINK_CMD_IDX_MONI_FFTDEC_FFTZOOMCW = 29u,
    HOSTLINK_CMD_IDX_RXFILTER = 30u,
    HOSTLINK_CMD_IDX_KS_KM_KIMB_CWTONE_CWVOL_CWTRAIN = 33u,
    HOSTLINK_CMD_IDX_QSKTIME_KR = 34u,
    HOSTLINK_CMD_IDX_BIASDRIVE_BIASFINAL = 41u,
    HOSTLINK_CMD_IDX_RIT = 42u,
    HOSTLINK_CMD_IDX_XIT = 43u
} HostlinkCmdIndex;

static uint8_t s_base_info[HOSTLINK_BASE_INFO_LEN] BSP_RAM_D1_BSS;

/* Command block double buffer: the ISR fills s_cmd_rx, the task diffs it
 * against s_cmd_shadow and logs changed words. */
static uint32_t          s_cmd_rx[HOSTLINK_CMD_WORDS] BSP_RAM_D1_BSS;
static uint32_t          s_cmd_shadow[HOSTLINK_CMD_WORDS] BSP_RAM_D1_BSS;
static volatile bool     s_cmd_dirty;
static volatile uint16_t s_pending_host_cmd;
static volatile bool     s_host_cmd_pending;
static bool              s_cal_host;
static bool              s_swr_scan_host;

static void build_base_info(void) {
    memset(s_base_info, 0, sizeof(s_base_info));

    uint8_t hw = App_Periph_HwRev();
    (void)snprintf((char *)s_base_info + HOSTLINK_VER1_OFF, HOSTLINK_FIELD_MAX,
                   "1.0.%u", (unsigned)hw);

    uint8_t ver[3];
    Drv_Fpga_GetVersion(ver);
    (void)snprintf((char *)s_base_info + HOSTLINK_VER2_OFF, HOSTLINK_FIELD_MAX,
                   "%u.%u.%u", (unsigned)ver[0], (unsigned)ver[1],
                   (unsigned)ver[2]);

    (void)snprintf((char *)s_base_info + HOSTLINK_FWVER_OFF, HOSTLINK_FIELD_MAX,
                   "%s", BOARD_FW_VERSION);
    (void)snprintf((char *)s_base_info + HOSTLINK_FWDATE_OFF, HOSTLINK_FIELD_MAX,
                   "%s", BOARD_FW_DATE);
}

/* --- transport callbacks (ISR context) ---------------------------------- */

/* The CW text and WPM registers are not decoded yet; answer zeros with the
 * requested length clamped to the region. */
static size_t hostlink_read_zero_region(uint16_t reg, uint16_t start,
                                        uint16_t end, uint8_t *dst,
                                        size_t cap) {
    size_t n = (size_t)(end - start) + 1u - (size_t)(reg - start);
    if (n > cap) {
        n = cap;
    }
    memset(dst, 0, n);
    return n;
}

static size_t hostlink_read(uint16_t reg, uint8_t *dst, size_t cap) {
    App_Power_NotifyHostFrame();

    if (reg <= HOSTLINK_BASE_INFO_END) {
        size_t off = (size_t)reg;
        if (off >= sizeof(s_base_info)) {
            off = 0u;
        }
        size_t n = sizeof(s_base_info) - off;
        if (n > cap) {
            n = cap;
        }
        memcpy(dst, s_base_info + off, n);
        return n;
    }

    if (reg >= HOSTLINK_CALIB_READ_ADDR && reg <= HOSTLINK_CALIB_READ_END) {
        const DrvNvramCalib *calib = App_Nvram_Calib();
        size_t off = (size_t)(reg - HOSTLINK_CALIB_READ_ADDR);
        size_t n   = sizeof(*calib) - off;
        if (n > cap) {
            n = cap;
        }
        memcpy(dst, (const uint8_t *)calib + off, n);
        return n;
    }

    if (reg == HOSTLINK_READY_ADDR) {
        if (cap >= 1u) {
            dst[0] = 1u;
        }
        return 1u;
    }

    if (reg >= HOSTLINK_CW_TEXT_ADDR && reg <= HOSTLINK_CW_TEXT_END) {
        return hostlink_read_zero_region(reg, HOSTLINK_CW_TEXT_ADDR,
                                         HOSTLINK_CW_TEXT_END, dst, cap);
    }

    if (reg >= HOSTLINK_CW_WPM_ADDR && reg <= HOSTLINK_CW_WPM_END) {
        return hostlink_read_zero_region(reg, HOSTLINK_CW_WPM_ADDR,
                                         HOSTLINK_CW_WPM_END, dst, cap);
    }

    /* Unused regions: the client does not read them, answer zeros. */
    if (cap >= 1u) {
        dst[0] = 0u;
    }
    return 1u;
}

static void hostlink_write(uint16_t reg, const uint8_t *data, size_t len) {
    App_Power_NotifyHostFrame();

    if (App_HostLinkExt_Write(reg, data, len)) {
        return;
    }

    if (reg <= (HOSTLINK_CMD_BYTES - 1u)) {
        uint8_t *cmd = (uint8_t *)s_cmd_rx;
        for (size_t i = 0; i < len; ++i) {
            cmd[(reg + i) % HOSTLINK_CMD_BYTES] = data[i];
        }
        s_cmd_dirty = true;
        App_Scheduler_Notify(APP_SCHED_EV_HOSTLINK);
        return;
    }

    if (reg >= HOSTLINK_CALIB_WR_ADDR && reg <= HOSTLINK_CALIB_WR_END) {
        uint16_t off = (uint16_t)((reg - HOSTLINK_CALIB_WR_ADDR) %
                                  DRV_NVRAM_CALIB_LEN);
        size_t   done = 0u;
        while (done < len) {
            size_t seg = DRV_NVRAM_CALIB_LEN - off;
            if (seg > len - done) {
                seg = len - done;
            }
            (void)App_Nvram_CalibWrite(off, data + done, seg);
            done += seg;
            off = 0u;
        }
        return;
    }

    if (reg == HOSTLINK_CMD_ADDR) {
        if (len >= 2u) {
            s_pending_host_cmd = (uint16_t)((uint16_t)data[0] |
                                            ((uint16_t)data[1] << 8));
            s_host_cmd_pending = true;
            App_Scheduler_Notify(APP_SCHED_EV_HOSTLINK);
        }
        return;
    }
}

/* --- deferred work (main loop) ------------------------------------------ */

static void hostlink_run_host_cmd(uint16_t cmd) {
    switch (cmd) {
    case HOSTLINK_CMD_AGC:
        LOG_INFO("host cmd 0x8002 (AGC reset stats)");
        Sdr_AGC_ResetStats();
        break;
    case HOSTLINK_CMD_DUMP:
        LOG_INFO("host cmd 0x8003 (dump)");
        (void)App_HostCmd_CalibDump();
        break;
    case HOSTLINK_CMD_ATU:
        LOG_INFO("host cmd 0x8004 (ATU) start tune");
        App_Atu_RequestTune();
        break;
    case HOSTLINK_CMD_CAL_SAVE:
        LOG_INFO("host cmd 0x8005 (calib save)");
        (void)App_HostCmd_CalibSave();
        break;
    case HOSTLINK_CMD_PWR_OFF:
        LOG_INFO("host cmd 0x8009 (power off)");
        App_PowerOff_Request();
        break;
    case HOSTLINK_CMD_IAP:
        LOG_INFO("host cmd 0x800A (IAP)");
        Bsp_Time_DelayMs(100);
        Bsp_Boot_EnterIap();
        break;
    default:
        LOG_INFO("host cmd 0x%04x unknown, ignored", (unsigned)cmd);
        break;
    }
}

/* --- command-block handlers (main loop, staged into the radio state) ----- */

/*
 * Each handler parses one command word and writes the desired state through the
 * App_RadioState_* setters. The setters only stage the change; hostlink_task
 * calls App_RadioState_Apply() once after the whole block. The staged edits are
 * committed once they form a valid set, which may span several host blocks.
 */

static void cmd_vfoa_freq(uint32_t value) {
    if (value == 0u) {
        LOG_DEBUG("vfoa_freq 0 ignored");
        return;
    }
    App_RadioState_SetFreq(APP_RADIO_VFO_A, value);
}

static void cmd_vfob_freq(uint32_t value) {
    if (value == 0u) {
        LOG_DEBUG("vfob_freq 0 ignored");
        return;
    }
    App_RadioState_SetFreq(APP_RADIO_VFO_B, value);
}

static void cmd_vfoa_att(uint32_t value) {
    App_RadioState_SetAtt(APP_RADIO_VFO_A, (value & HOSTLINK_LOW_BYTE_MASK) != 0u);
}

static void cmd_vfoa_pre(uint32_t value) {
    App_RadioState_SetPre(APP_RADIO_VFO_A, (value & HOSTLINK_LOW_BYTE_MASK) != 0u);
}

static void cmd_vfob_att(uint32_t value) {
    App_RadioState_SetAtt(APP_RADIO_VFO_B, (value & HOSTLINK_LOW_BYTE_MASK) != 0u);
}

static void cmd_vfob_pre(uint32_t value) {
    App_RadioState_SetPre(APP_RADIO_VFO_B, (value & HOSTLINK_LOW_BYTE_MASK) != 0u);
}

static void cmd_vfoa_mode(uint32_t value) {
    App_RadioState_SetMode(APP_RADIO_VFO_A,
                           (SdrModulation)(value & HOSTLINK_LOW_BYTE_MASK));
}

static void cmd_vfob_mode(uint32_t value) {
    App_RadioState_SetMode(APP_RADIO_VFO_B,
                           (SdrModulation)(value & HOSTLINK_LOW_BYTE_MASK));
}

static void cmd_vfoa_agc(uint32_t value) {
    LOG_INFO("AGC mode: %u", value);
    App_RadioState_SetAgcMode(APP_RADIO_VFO_A, value);
}

static void cmd_vfob_agc(uint32_t value) {
    App_RadioState_SetAgcMode(APP_RADIO_VFO_B, value);
}

static void cmd_moni_fftdec(uint32_t value) {
    /* fftdec is bits 8..11, monitor_level bits 0..7, fftzoomcw bits 12..15.
     * fftzoomcw is an exponent (the spectrum zoom linear factor is 1 << n). */
    uint8_t fftdec        = (uint8_t)((value >> 8) & 0x0Fu);
    uint8_t monitor_level = (uint8_t)(value & HOSTLINK_LOW_BYTE_MASK);
    uint8_t fftzoomcw     = (uint8_t)((value >> 12) & 0x0Fu);
    App_RadioState_SetFftdec(fftdec);
    App_RadioState_SetMonitorLevel(monitor_level);
    App_RadioState_SetFftzoomcw(fftzoomcw);
}

static void cmd_rit(uint32_t value) {
    /* Signed Hz offset for the RX tuning (FPGA NCO only), +-2000 clamped in
     * the state setter. */
    App_RadioState_SetRit((int32_t)value);
}

static void cmd_xit(uint32_t value) {
    /* Signed Hz offset for the TX tuning (FPGA NCO only), +-2000 clamped in
     * the state setter. */
    App_RadioState_SetXit((int32_t)value);
}

static void cmd_rxfilter(uint32_t value) {
    /* f_low is the HPF edge in bits 31..16, f_high the LPF edge in 15..0. */
    int32_t low = (int16_t)(value >> 16);
    int32_t high = (int16_t)(value & 0xFFFFu);
    App_RadioState_SetRxFilter(low, high);
}

static void cmd_rfg_txpwr(uint32_t value) {
    /* Low byte is rfg; high byte is txpwr in 0.1 W units (0 = 0 W). */
    float txpwr_w = (float)((value >> 8) & HOSTLINK_LOW_BYTE_MASK) * 0.1f;

    App_RadioState_SetRfg((uint8_t)(value & HOSTLINK_LOW_BYTE_MASK));
    App_RadioState_SetTxPwr(txpwr_w);
}

static void cmd_biasdrive_biasfinal(uint32_t value) {
    /* x6200_biasdrive_biasfinal = 41: drive in bits 0..15, final in 16..31. */
    uint16_t bias_drive = (uint16_t)(value & 0xFFFFu);
    uint16_t bias_final = (uint16_t)(value >> 16);

    if (App_Nvram_CalibSetBias(bias_drive, bias_final) != 0) {
        LOG_ERROR("bias calib update failed");
        return;
    }

    int rc = Drv_PaBias_SetBias(bias_drive, bias_final);
    LOG_INFO("bias drive %u final %u (DAC %s)", (unsigned)bias_drive,
             (unsigned)bias_final, rc == 0 ? "applied" : "refused");
}

static void cmd_rxvol(uint32_t value) {
    /* Only the low byte is the volume; upper bytes are ignored silently. */
    App_RadioState_SetRxVol((uint8_t)(value & HOSTLINK_LOW_BYTE_MASK));
}

static void cmd_sple_atue_trx(uint32_t value) {
    bool split      = (value & SPLE_ATUE_TRX_SPLIT);
    bool drive_off  = (value & SPLE_ATUE_TRX_BIAS_DRIVE_OFF);
    bool final_off  = (value & SPLE_ATUE_TRX_BIAS_FINAL_OFF);
    bool mtx        = (value & SPLE_ATUE_TRX_MTX);
    bool iptt       = (value & SPLE_ATUE_TRX_IPTT);
    bool civptt     = (value & SPLE_ATUE_TRX_CIVPTT);
    bool voice_rec  = (value & SPLE_ATUE_TRX_VOICE_REC);
    bool swr_scan   = (value & SPLE_ATUE_TRX_SWR_SCAN);
    bool tune_flag  = (value & SPLE_ATUE_TRX_TUNE_FLAG);
    bool rsvd_72    = (value & SPLE_ATUE_TRX_RSVD_72);
    bool atue       = (value & SPLE_ATUE_TRX_ATUE);
    bool tune       = (value & SPLE_ATUE_TRX_TUNE);
    bool cal        = (value & SPLE_ATUE_TRX_CAL);
    bool pof        = (value & SPLE_ATUE_TRX_POF);
    bool caltx      = (value & SPLE_ATUE_TRX_CALTX);
    bool iqswap     = (value & SPLE_ATUE_TRX_IQSWAP);

    uint32_t known = SPLE_ATUE_TRX_SPLIT | SPLE_ATUE_TRX_VOICE_REC |
                     SPLE_ATUE_TRX_SWR_SCAN | SPLE_ATUE_TRX_TUNE_FLAG |
                     SPLE_ATUE_TRX_RSVD_72 | SPLE_ATUE_TRX_ATUE |
                     SPLE_ATUE_TRX_TUNE | SPLE_ATUE_TRX_MTX |
                     SPLE_ATUE_TRX_CAL | SPLE_ATUE_TRX_POF |
                     SPLE_ATUE_TRX_IPTT | SPLE_ATUE_TRX_CIVPTT |
                     SPLE_ATUE_TRX_CALTX | SPLE_ATUE_TRX_BIAS_DRIVE_OFF |
                     SPLE_ATUE_TRX_BIAS_FINAL_OFF | SPLE_ATUE_TRX_IQSWAP;

    App_RadioState_SetSplit(split);
    App_RadioState_SetSwrScan(swr_scan);
    App_RadioState_SetVoiceRec(voice_rec);
    (void)Drv_PaBias_SetOff(drive_off, final_off);

    if (s_swr_scan_host != swr_scan) {
        s_swr_scan_host = swr_scan;
        LOG_INFO("swr scan %u", (unsigned)swr_scan);
    }

    /* Host calibration TX: keys the PA without a signal. */
    if (s_cal_host != cal) {
        s_cal_host = cal;
        LOG_INFO("host cal %u", (unsigned)cal);
    }
    App_Ptt_SetCalibrate(cal);

    /* The arbiter only stages the flags; the TX transition runs after
     * App_RadioState_Apply(). pof does not power off: that is host cmd 0x8009. */
    App_Ptt_SetMtx(mtx);
    App_Ptt_SetIptt(iptt);
    App_Ptt_SetCivPtt(civptt);
    App_Ptt_SetSwrScan(swr_scan);

    /* tuner_on: store the host wish in the state and hand it to the ATU, which
     * applies it when idle and forces it to 1 while tuning. */
    App_RadioState_SetTunerOn(atue);
    App_Atu_SetOn(atue);

    LOG_DEBUG("sple_atue_trx split %u drive_off %u final_off %u mtx %u "
              "iptt %u civptt %u voice_rec %u swr_scan %u tune_flag %u "
              "rsvd_72 %u atue %u tune %u cal %u pof %u caltx %u iqswap %u "
              "reserved 0x%08X",
              (unsigned)split, (unsigned)drive_off, (unsigned)final_off,
              (unsigned)mtx, (unsigned)iptt, (unsigned)civptt,
              (unsigned)voice_rec, (unsigned)swr_scan, (unsigned)tune_flag,
              (unsigned)rsvd_72, (unsigned)atue, (unsigned)tune,
              (unsigned)cal, (unsigned)pof, (unsigned)caltx,
              (unsigned)iqswap, (unsigned)(value & ~known));
}

static void cmd_vi_vm(uint32_t value) {
    uint32_t sel = value & 0x000000FFu;

    if (sel < APP_RADIO_VFO_COUNT) {
        App_RadioState_SetActiveVfo((AppRadioVfo)sel);
    } else {
        LOG_DEBUG("vi_vm vfo %u out of range, ignored", (unsigned)sel);
    }
    if ((value & 0x00010000u) != 0u) {
        LOG_DEBUG("vi_vm VM bit set, ignored");
    }
}

static void cmd_ling_loutg_imicg_hmicg(uint32_t value) {
    uint8_t ling  = (uint8_t)(value & 0xFFu);
    uint8_t loutg = (uint8_t)((value >> 8) & 0xFFu);
    uint8_t imicg = (uint8_t)((value >> 16) & 0xFFu);
    uint8_t hmicg = (uint8_t)((value >> 24) & 0xFFu);

    LOG_DEBUG("cmd20 ling %u loutg %u imicg %u hmicg %u", (unsigned)ling,
              (unsigned)loutg, (unsigned)imicg, (unsigned)hmicg);
    App_RadioState_SetMicGains(ling, loutg, imicg, hmicg);
}

static void cmd_micsel_pttmode_chge_spmode_auxiqgen_sqlthr(uint32_t value) {
    /* micsel [1:0], pttmode [3:2], chge [4], spmode [5], auxiqgen [6],
     * sqlthr [15:8], sql_fm [23:16], sql_en [24]. chge, spmode and micsel are
     * applied. */
    uint8_t  micsel   = (uint8_t)(value & 0x03u);
    uint8_t  pttmode  = (uint8_t)((value >> 2) & 0x03u);
    bool     chge     = (value & 0x10u) != 0u;
    bool     spmode   = (value & 0x20u) != 0u;
    bool     auxiqgen = (value & 0x40u) != 0u;
    uint8_t  sqlthr   = (uint8_t)((value >> 8) & 0xFFu);
    uint8_t  sql_fm   = (uint8_t)((value >> 16) & 0xFFu);
    bool     sql_en   = (value & 0x01000000u) != 0u;

    LOG_DEBUG("cmd21 micsel %u pttmode %u chge %u spmode %u auxiqgen %u "
              "sqlthr %u sql_fm %u sql_en %u",
              (unsigned)micsel, (unsigned)pttmode, (unsigned)chge,
              (unsigned)spmode, (unsigned)auxiqgen, (unsigned)sqlthr,
              (unsigned)sql_fm, (unsigned)sql_en);
    App_RadioState_SetMicSel(micsel);
    (void)pttmode;
    (void)auxiqgen;
    App_RadioState_SetSquelch(sql_en, sqlthr, sql_fm);
    /* chge=0 disables battery charging; the protection tick applies the change
     * to the charger. The RAM copy is persisted to NVPARAM at power-off. */
    App_Nvram_SetChargeEnabled(chge);
    App_Protect_SetChargeEnabled(chge);
    App_RadioState_SetSpmode(spmode);
}

static void cmd_nrthr_nbw_nbthr_nre_nbe(uint32_t value) {
    /* nrthr [7:0], nbw [15:8], nbthr [23:16], nre bit24, nbe bit25. */
    uint8_t nrthr = (uint8_t)(value & 0xFFu);
    uint8_t nbw   = (uint8_t)((value >> 8) & 0xFFu);
    uint8_t nbthr = (uint8_t)((value >> 16) & 0xFFu);
    bool    nre   = (value & 0x01000000u) != 0u;
    bool    nbe   = (value & 0x02000000u) != 0u;

    LOG_DEBUG("cmd23 nrthr %u nbw %u nbthr %u nre %u nbe %u",
              (unsigned)nrthr, (unsigned)nbw, (unsigned)nbthr,
              (unsigned)nre, (unsigned)nbe);
    Sdr_NR_SetThreshold((float)nrthr);
    Sdr_NR_SetEnabled(nre);
    Sdr_NB_SetParams(nbe, nbw, nbthr);
}

static void cmd_dnfcnt_dnfwidth_dnfmode(uint32_t value) {
    /* Center [11:0] Hz, width [23:12] Hz, mode [25:24] (SdrNotchMode). */
    uint32_t center = value & 0xFFFu;
    uint32_t width  = (value >> 12) & 0xFFFu;
    uint32_t mode   = (value >> 24) & 0x3u;

    LOG_DEBUG("cmd24 center %u width %u mode %u", (unsigned)center,
              (unsigned)width, (unsigned)mode);
    if (mode > (uint32_t)SDR_NOTCH_MODE_AUTO) {
        return;
    }
    Sdr_Notch_SetParams((SdrNotchMode)mode, (float)center, (float)width);
}

static void cmd_cmplevel_cmpe(uint32_t value) {
    /* cmplevel [3:0] (0..3 = ratio 1:2/1:4/1:6/1:8, otherwise 1:10),
     * cmpe bit4 enables the compressor. */
    uint32_t cmplevel = value & 0xFu;
    bool     cmpe     = (value & 0x10u) != 0u;

    static const uint8_t ratio[4] = {2u, 4u, 6u, 8u};
    uint8_t r = (cmplevel < 4u) ? ratio[cmplevel] : 10u;

    LOG_DEBUG("cmd25 cmplevel %u cmpe %u", (unsigned)cmplevel, (unsigned)cmpe);
    Sdr_Compressor_SetRatio(r);
    Sdr_Compressor_Enable(cmpe);
}

static void cmd_ks_km_kimb_cwtone_cwvol_cwtrain(uint32_t value) {
    /* ks [7:0] key speed (WPM, clamped 5..50), km [9:8] key mode,
     * kimb [11:10] iambic mode, cwtone [22:12] (unsigned 11-bit),
     * cwvol [28:23] sidetone volume (0..63, 30 = 0 dB), cwtrain [29]. */
    uint8_t  ks      = (uint8_t)(value & 0xFFu);
    uint8_t  km      = (uint8_t)((value >> 8) & 0x03u);
    uint8_t  kimb    = (uint8_t)((value >> 10) & 0x03u);
    uint32_t cwtone  = (value >> 12) & 0x7FFu;
    uint8_t  cwvol   = (uint8_t)((value >> 23) & 0x3Fu);
    bool     cwtrain = (value & 0x20000000u) != 0u;

    LOG_DEBUG("cmd33 ks %u km %u kimb %u cwtone %u cwvol %u cwtrain %u",
              (unsigned)ks, (unsigned)km, (unsigned)kimb, (unsigned)cwtone,
              (unsigned)cwvol, (unsigned)cwtrain);
    App_RadioState_SetCwTone(cwtone);
    App_RadioState_SetCwKeySpeed(ks);
    if (km <= (uint8_t)SDR_CW_KEY_DUAL_RIGHT) {
        App_RadioState_SetCwKeyMode((SdrCwKeyMode)km);
    }
    if (kimb <= (uint8_t)SDR_CW_IAMBIC_B) {
        App_RadioState_SetCwIambicMode((SdrCwIambicMode)kimb);
    }
    App_RadioState_SetCwVol(cwvol);
    App_RadioState_SetCwTrain(cwtrain);
}

static void cmd_qsktime_kr(uint32_t value) {
    /* qsk_time [15:0] ms (0..1000), dot_dash_ratio [31:16] x0.1 (2.5..4.5). */
    uint32_t qsk_time       = value & 0xFFFFu;
    uint32_t dot_dash_ratio = (value >> 16) & 0xFFFFu;

    LOG_DEBUG("cmd34 qsk %u ratio %u", (unsigned)qsk_time,
              (unsigned)dot_dash_ratio);
    App_RadioState_SetCwQskTimeAndRatio(qsk_time, dot_dash_ratio);
}

static void cmd_atu_network(uint32_t value) {
    /* [7:0] = C code, [15:8] = L code, [16] = switch. */
    uint8_t c  = (uint8_t)(value & 0xFFu);
    uint8_t l  = (uint8_t)((value >> 8) & 0xFFu);
    uint8_t sw = (uint8_t)((value >> 16) & 1u);

    LOG_DEBUG("cmd17 atu word 0x%08X L %u C %u sw %u", (unsigned)value,
              (unsigned)l, (unsigned)c, (unsigned)sw);
    App_Atu_ApplyNetwork(c, l, sw);
}

/* NULL = command not handled yet; fill in as support is added. */
static void (*const s_cmd_handlers[HOSTLINK_CMD_WORDS])(uint32_t) = {
    [HOSTLINK_CMD_IDX_VFOA_FREQ] = cmd_vfoa_freq,
    [HOSTLINK_CMD_IDX_VFOA_ATT]  = cmd_vfoa_att,
    [HOSTLINK_CMD_IDX_VFOA_PRE]  = cmd_vfoa_pre,
    [HOSTLINK_CMD_IDX_VFOA_MODE] = cmd_vfoa_mode,
    [HOSTLINK_CMD_IDX_VFOA_AGC]  = cmd_vfoa_agc,
    [HOSTLINK_CMD_IDX_VFOB_FREQ] = cmd_vfob_freq,
    [HOSTLINK_CMD_IDX_VFOB_ATT]  = cmd_vfob_att,
    [HOSTLINK_CMD_IDX_VFOB_PRE]  = cmd_vfob_pre,
    [HOSTLINK_CMD_IDX_VFOB_MODE] = cmd_vfob_mode,
    [HOSTLINK_CMD_IDX_VFOB_AGC]  = cmd_vfob_agc,
    [HOSTLINK_CMD_IDX_SPLE_ATUE_TRX]     = cmd_sple_atue_trx,
    [HOSTLINK_CMD_IDX_ATU_NETWORK] = cmd_atu_network,
    [HOSTLINK_CMD_IDX_VFO_SEL]   = cmd_vi_vm,
    [HOSTLINK_CMD_IDX_RXVOL]     = cmd_rxvol,
    [HOSTLINK_CMD_IDX_RFG_TXPWR] = cmd_rfg_txpwr,
    [HOSTLINK_CMD_IDX_BIASDRIVE_BIASFINAL] = cmd_biasdrive_biasfinal,
    [HOSTLINK_CMD_IDX_RIT] = cmd_rit,
    [HOSTLINK_CMD_IDX_XIT] = cmd_xit,
    [HOSTLINK_CMD_IDX_LING_LOUTG_IMICG_HMICG] =
        cmd_ling_loutg_imicg_hmicg,
    [HOSTLINK_CMD_IDX_MICSEL_PTTMODE_CHGE_SPMODE_AUXIQGEN_SQLTHR] =
        cmd_micsel_pttmode_chge_spmode_auxiqgen_sqlthr,
    [HOSTLINK_CMD_IDX_NRTHR_NBW_NBTHR_NRE_NBE] = cmd_nrthr_nbw_nbthr_nre_nbe,
    [HOSTLINK_CMD_IDX_DNFCNT_DNFWIDTH_DNFMODE] = cmd_dnfcnt_dnfwidth_dnfmode,
    [HOSTLINK_CMD_IDX_CMPLEVEL_CMPE] = cmd_cmplevel_cmpe,
    [HOSTLINK_CMD_IDX_MONI_FFTDEC_FFTZOOMCW] = cmd_moni_fftdec,
    [HOSTLINK_CMD_IDX_RXFILTER] = cmd_rxfilter,
    [HOSTLINK_CMD_IDX_KS_KM_KIMB_CWTONE_CWVOL_CWTRAIN] =
        cmd_ks_km_kimb_cwtone_cwvol_cwtrain,
    [HOSTLINK_CMD_IDX_QSKTIME_KR] = cmd_qsktime_kr,
};

/*
 * A command block seeds the host configuration once both VFO frequencies are
 * present (not the 0xFFFFFFFF "never written" sentinel) and within the tuning
 * range. A partial or garbage block must not bring the radio up.
 */
static bool host_block_configured(const uint32_t *block) {
    uint32_t freq_a = block[HOSTLINK_CMD_IDX_VFOA_FREQ];
    uint32_t freq_b = block[HOSTLINK_CMD_IDX_VFOB_FREQ];
    uint32_t max_hz = 1000000000u; // 1 GHz

    return freq_a != HOSTLINK_CMD_WORD_UNSET && freq_a <= max_hz &&
           freq_b != HOSTLINK_CMD_WORD_UNSET && freq_b <= max_hz;
}

static void hostlink_task(void) {
    if (s_cmd_dirty) {
        uint32_t basepri = Bsp_Irq_SaveLow();
        s_cmd_dirty      = false;
        uint32_t snapshot[HOSTLINK_CMD_WORDS];
        memcpy(snapshot, s_cmd_rx, sizeof(snapshot));
        Bsp_Irq_RestoreLow(basepri);

        App_RadioState_Begin();
        for (uint32_t i = 0; i < HOSTLINK_CMD_WORDS; ++i) {
            if (snapshot[i] != s_cmd_shadow[i]) {
                LOG_DEBUG("cmd[%u] 0x%08x -> 0x%08x", (unsigned)i,
                          (unsigned)s_cmd_shadow[i], (unsigned)snapshot[i]);
                s_cmd_shadow[i] = snapshot[i];
                if (s_cmd_handlers[i] != 0) {
                    s_cmd_handlers[i](snapshot[i]);
                }
            }
        }
        if (!App_RadioState_IsConfigured() &&
            host_block_configured(snapshot)) {
            App_RadioState_SetConfigured();
        }
        App_RadioState_Apply();
    }

    if (s_host_cmd_pending) {
        uint32_t basepri = Bsp_Irq_SaveLow();
        uint16_t cmd     = s_pending_host_cmd;
        s_host_cmd_pending = false;
        Bsp_Irq_RestoreLow(basepri);

        hostlink_run_host_cmd(cmd);
    }
}

/* --- init --------------------------------------------------------------- */

int App_HostLink_Init(void) {
    build_base_info();

    memset(s_cmd_rx, 0xFF, sizeof(s_cmd_rx));
    memset(s_cmd_shadow, 0xFF, sizeof(s_cmd_shadow));
    s_cmd_dirty        = false;
    s_pending_host_cmd = 0u;
    s_host_cmd_pending = false;

    if (Bsp_I2cSlave_Init(hostlink_write, hostlink_read) != 0) {
        LOG_ERROR("i2c slave init failed");
        return -1;
    }
    return App_Scheduler_AddEvent(hostlink_task, APP_SCHED_EV_HOSTLINK);
}
