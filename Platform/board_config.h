#pragma once

/* Reported over the host protocol base-info (App/app_hostlink). */
#define BOARD_FW_VERSION "1.0.0"
#define BOARD_FW_DATE    "Nov 17 2026, by R2RFE"

/* RF ADC/DAC sampling clock: the clock generator OUT1 rate that feeds the FPGA
 * NCO and the I2S_CKIN. The FPGA tuning words, the Nyquist-zone/band selection
 * and the FFT sample-rate divisors are all derived from this value. */
#define BOARD_RF_SAMPLING_RATE_HZ    122880000u

/* Audio clock domain: the codec I2S rate and the FPGA bus tick come from the
 * same clock generator, so every audio/FPGA tick rate derives from this. */
#define BOARD_AUDIO_SAMPLE_RATE_HZ   48000u

#define BOARD_BUTTONS_PERIOD_MS      20u
#define BOARD_HOUSEKEEPING_PERIOD_MS 1000u

/* Boot active frequency pushed to the FPGA and the RF filters. */
#define BOARD_DEFAULT_FREQ_HZ        14270000u
/* Upper TX frequency limit; transmissions above it are refused. */
#define BOARD_TX_FREQ_MAX_HZ         54000000u
/* Relay pulse step for the LPF latching sequence. */
#define BOARD_RF_RELAY_STEP_MS       30u

/*
 * TX level reference. Modulators and the ALC work in a normalized domain with a
 * baseband peak of 1.0; App_Radio_TxPublish() applies the ALC output scale
 * (k_mod times the per-band level offset) once at the float -> Q1.31 FPGA
 * boundary. The per-band level table — calibrated so full-scale drive reaches
 * BOARD_TX_MAX_POWER_W at the minimum supply voltage — lives in App/app_alc.c.
 */
#define BOARD_TX_MAX_POWER_W         8.0f
/* Common smoothing for the TX input RMS and the measured average power. */
#define BOARD_TX_AVG_TAU_S           0.05f

/* Power button held low longer than this requests power-off. */
#define BOARD_POWER_KEY_OFF_MS       5000u
/* Settle pause between the safe-off steps and the final power cut. */
#define BOARD_POWER_OFF_SETTLE_MS    3000u

/*
 * Auto power-off tuning (host-link watchdog; total no-host window is roughly
 * the three below added together).
 */
#define BOARD_AUTO_OFF_IDLE_MS       3000u  /* no host frame -> arm */
#define BOARD_AUTO_OFF_GRACE_S       10u    /* pause after armed */
#define BOARD_AUTO_OFF_COUNTDOWN_S   180u   /* countdown before power-off */

/*
 * Power/temperature protection thresholds (App/app_protect). Every "trip /
 * release" pair is a hysteresis: the condition trips when it crosses TRIP and
 * releases when it comes back past RELEASE. Temperatures are millidegrees
 * Celsius, voltages mV, SOC tenths of a percent.
 */
#define BOARD_BATT_TEMP_HIGH_MC          45000
#define BOARD_BATT_TEMP_HIGH_RELEASE_MC  40000
#define BOARD_BATT_TEMP_LOW_MC           0
#define BOARD_BATT_TEMP_LOW_RELEASE_MC   2000
#define BOARD_BATT_TEMP_COLD_MC          (-30000)
#define BOARD_BATT_TEMP_COLD_RELEASE_MC  (-25000)
#define BOARD_CELL_OV_MV                 4250u
#define BOARD_CELL_OV_RELEASE_MV         4200u
#define BOARD_CELL_UV_MV                 3000u
#define BOARD_CELL_UV_RELEASE_MV         3100u
#define BOARD_PACK_LOW_MV                6800u
#define BOARD_PACK_LOW_RELEASE_MV        7000u
#define BOARD_SOC_LOW_D_PCT              50u
#define BOARD_SOC_LOW_RELEASE_D_PCT      60u
#define BOARD_PA_TEMP_HIGH_MC            80000
#define BOARD_PA_TEMP_HIGH_RELEASE_MC    75000
/*
 * Open-loop PA bias temperature compensation. The correction is zero at
 * BOARD_PA_BIAS_TEMP_REF_MC and only ever lowers bias, on both the drive and
 * the final channel. Each K is that channel's slope in tenths of mV/degC; the
 * DAC code conversion also uses DRV_ADC_MON_VREF_MV and the 12-bit full scale.
 * K values tuned on hardware. This is not ALC.
 */
#define BOARD_PA_BIAS_TEMP_REF_MC                   40000
#define BOARD_PA_BIAS_TEMP_K_FINAL_TENTHS_MV_PER_C  20
#define BOARD_PA_BIAS_TEMP_K_DRIVE_TENTHS_MV_PER_C  30
#define BOARD_VEXT_LOW_MV                10000u
#define BOARD_VEXT_LOW_RELEASE_MV        11500u

/* Protection tick divider: run the ADC/I2C sample burst every N 10 ms ticks. */
#define BOARD_PROTECT_SAMPLE_DIV         50u
/* Consecutive failed bursts before the cached data counts as unknown. */
#define BOARD_PROTECT_I2C_FAIL_MAX       3u

/*
 * Charge-current policy: precharge at low current when a cell is weak (reuses
 * the UV threshold pair BOARD_CELL_UV_MV / _RELEASE_MV through the s_cell_uv
 * latch), then CV taper by max per-cell voltage. The precharge current is the
 * bq24725 minimum step.
 */
#define BOARD_CHG_PRECHARGE_CURRENT_MA   128u
#define BOARD_CHG_TAPER1_CURRENT_MA      256u
#define BOARD_CHG_TAPER2_CURRENT_MA      128u
#define BOARD_CHG_TAPER1_CELL_MV         4150u
#define BOARD_CHG_TAPER2_CELL_MV         4200u
