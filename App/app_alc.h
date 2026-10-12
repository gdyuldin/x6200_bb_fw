#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * TX automatic level control. The output scale (k_mod times the per-band level
 * offset) is applied to the published TX samples before the FPGA staging ring;
 * the gain layers (g_slow, g_fast, fast_clamp, swr_clamp and the external ALC
 * backoff) run in the ADC DMA ISR and drive the Q15 gain the FPGA driver applies
 * when it emits a TX sample.
 *
 * Regulation is envelope-linear: the output is a scaled copy of the input, and
 * g_slow only corrects the residual transfer error. The per-band level offset
 * (selected by TX frequency, calibrated so the input-to-output transfer settles
 * at BOARD_TX_MAX_POWER_W on every band) makes K band-independent, and g_slow =
 * sqrt(BOARD_TX_MAX_POWER_W / K) pulls the measured average output to the
 * commanded estimate. It is independent of the setpoint and the input level and
 * never exceeds 1: a transfer above nominal is attenuated, a lower one is not
 * boosted (the output simply tracks the estimate). g_fast is a per-block peak
 * limiter at APP_ALC_FF_LIMIT_RATIO (1.20x) the setpoint; fast_clamp and
 * swr_clamp are absolute protection limits, and ext is the external ALC backoff
 * (only attenuation, commanded by the amplifier through the EXT ALC line). All
 * layers are absolute gain limits and never stack. The learned transfer K, the
 * slow gain, the input power average and the protection clamps are kept across
 * key-down and a temporary 0 W setpoint, so a CW/FM transmission starts at the
 * configured level instead of open-loop. K is also kept across a band change
 * (only the level offset is band-dependent); it starts from a conservative seed
 * (APP_ALC_K_SEED) until the first measurement, and its update uses one time
 * constant in both directions and is gated on the commanded output, with the
 * measured fwd clamped up to the detector floor so a suppressed output can pull
 * K down without over-correcting.
 */

/* Layer that produced the last total gain, for `AppAlcSnapshot.active_layer`. */
#define APP_ALC_LAYER_NONE       0u
#define APP_ALC_LAYER_SLOW       1u
#define APP_ALC_LAYER_FAST       2u
#define APP_ALC_LAYER_FAST_CLAMP 3u
#define APP_ALC_LAYER_SWR_CLAMP  4u
#define APP_ALC_LAYER_EXT        5u

/* SWR-scan carrier setpoint. This is the ALC target, not a calibrated watt:
 * the tune branch holds forward power near it while the GUI sweeps the VFO. */
#define APP_ALC_SWR_SCAN_POWER_W 1.0f

typedef struct {
    uint32_t limit_mw;
    float    k_mod;
    float    g_slow;
    float    g_fast;
    int32_t  fast_clamp_q15;
    int32_t  swr_clamp_q15;
    int32_t  gain_q15;
    uint8_t  level;
    float    iq_pow_pk;
    float    iq_pow_avg;
    float    gain_avg;
    float    fwd_avg_w;
    float    k_est;
    float    k_inst;
    bool     k_valid;
    float    p_pred;
    float    tx_ramp;
    uint32_t fast_attacks;
    uint32_t swr_attacks;
    uint8_t  active_layer;
    uint32_t slow_blocks;
    uint32_t fast_blocks;
} AppAlcSnapshot;

void    App_Alc_Init(void);
/* txpwr_w is the TX power setpoint in watts; 0 means 0 W. */
void    App_Alc_SetPowerLimit(float txpwr_w);
/* Runs in the ADC DMA ISR with the latest peak, PEP and reflected power, plus
 * the smoothed external ALC line code (a power-backoff command, only
 * attenuation). */
void    App_Alc_Update(float fwd_peak_w, float fwd_pep_w, float fwd_avg_w,
                       float ref_fast_w, float ref_avg_w, uint16_t ext_alc_raw);
/* Arm the RX -> TX soft-start ramp. Called from the TX branch of SetRadioMode. */
void    App_Alc_NotifyTxStart(void);

/*
 * ATU tuning mode. While active the setpoint is forced to APP_ATU_TUNE_POWER_W
 * (the user txpwr is saved and restored on exit), the SWR gain clamp is skipped
 * and the learned transfer K is frozen (the relay blank gaps would corrupt the
 * estimate). The slow trim and the direct-power fast clamp keep running so the
 * carrier holds the tune power.
 */
void    App_Alc_SetAtuActive(bool active);
/* SWR-scan carrier mode. Shares the same forced low-power tune state as the
 * ATU: the user txpwr is saved/restored, the SWR gain clamp is skipped and the
 * learned transfer K is frozen. The two modes are mutually exclusive. */
void    App_Alc_SetSwrScanActive(bool active);
/* Release the reflected-power clamp to unity after a successful tune: it may
 * have been wound down against the old mismatched load and would otherwise
 * recover at its slow release rate on the next transmission. */
void    App_Alc_ResetSwrClamp(void);
/* Report the TX-branch frequency: on a band change the per-band level offset is
 * selected (the transfer K is band-independent and kept). */
void    App_Alc_NotifyFrequency(uint32_t freq_hz);
/* Nudge the active band's calibrated level by delta_db (clamped to
 * APP_ALC_LEVEL_DB_MIN..MAX), apply it and return the new value; delta_db == 0
 * just reads the current value. Main-loop/console context, not ISR. */
float   App_Alc_AdjustLevelDb(float delta_db);
/* Restore the active band's factory level and return it. Main-loop/console
 * context, not ISR. */
float   App_Alc_ResetLevelDb(void);

/*
 * Per-band measured-power correction. The coupler/detector response droops with
 * frequency, so the raw detector reading under-reports the true power on the
 * high bands. The factor scales the measured forward and reflected power back
 * (identical tandem-match arms assumed). It is selected by TX frequency together
 * with the level offset.
 */
/* Linear power multiplier for the current band; 1.0 before the first frequency
 * report. ISR-safe (plain read of the applied factor). */
float   App_Alc_GetPowerCorr(void);
/* Nudge the active band's correction by delta_db (clamped to
 * APP_ALC_POWER_CORR_DB_MIN..MAX), apply it and return the new value; delta_db
 * == 0 just reads the current value. Main-loop/console context, not ISR. */
float   App_Alc_AdjustPowerCorrDb(float delta_db);
/* Restore the active band's factory correction and return it. Main-loop/console
 * context, not ISR. */
float   App_Alc_ResetPowerCorrDb(void);
/* Compute block IQ stats, update K/feed-forward, and return the output scale
 * (k_mod times the current band's level offset). Called from
 * App_Radio_TxPublish() in the I2S DMA ISR, before the samples are staged. */
float   App_Alc_GetOutputScale(const float *i, const float *q, uint32_t count);
/* Current total ALC gain as a percentage (0..100), for telemetry. */
uint8_t App_Alc_GetLevel(void);
/* Copy the internal state under an IRQ lock for diagnostics. */
void    App_Alc_GetSnapshot(AppAlcSnapshot *out);
