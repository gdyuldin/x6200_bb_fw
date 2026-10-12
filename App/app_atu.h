#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * ATU (automatic antenna tuner) application. A non-blocking state machine runs
 * from the scheduler tick: it keys a tune carrier for each measurement, sweeps
 * the L/C/switch relay network through a coarse search and a local coordinate
 * descent, then publishes the result to the host. Relays are driven by
 * Devices/drv_atu_relay (raw I2C, bypassing the pca9535 shadow); the tune
 * carrier is blanked before every relay move and switched back on only after
 * the relay sequence latches, so the network never switches under RF. The
 * reflection coefficient gamma is measured by the fast window in
 * App/app_tx_monitor. The user txpwr and learned ALC transfer are saved/restored
 * by App/app_alc; the VSWR trip and the ALC SWR clamp are disabled while tuning.
 * The TX indication and telemetry TX flag stay driven by the radio mode, not by
 * the carrier.
 *
 * While tuning the radio frequency/filter commit is held (App_RadioState_Apply
 * keeps the staged edits pending); they are applied on the next host block after
 * tuning ends. The ATU is forced on (in the TX path) during tuning and
 * restored to the host-requested state afterwards. While the ATU is off, host
 * network updates are only stored and are applied when it is switched on; the
 * L/C/SW relays keep their last values when it is switched off.
 *
 * atu_params packing, used by telemetry (uint32_t atu_params at offset 0x210) and
 * host command slot 17: [7:0] = C code, [15:8] = L code, [16] = switch.
 */

#define APP_ATU_PARAMS_C_MASK   0x000000FFu
#define APP_ATU_PARAMS_L_SHIFT  8u
#define APP_ATU_PARAMS_L_MASK   0x0000FF00u
#define APP_ATU_PARAMS_SW_SHIFT 16u
#define APP_ATU_PARAMS_SW_MASK  0x00010000u

/* Tunables; all times and thresholds are provisional and tuned on hardware. */
#define APP_ATU_TICK_MS          1u
/* Silence before a relay pulse: covers the carrier envelope ramp and the
 * worst-case audio block the ISR may still be finishing. */
#define APP_ATU_BLANK_MS         3u
/* Settle from the carrier switching on, not from the relay move: it includes
 * the envelope ramp and the delay until the ADC measurement is valid. */
#define APP_ATU_SETTLE_MS        8u
#define APP_ATU_WINDOW_MS        2u
#define APP_ATU_FAST_SAMPLES     5u
#define APP_ATU_FAST_STEP_MS     3u
/* Fine probes keep extra margin over the carrier-on settle above. */
#define APP_ATU_FINE_SETTLE_MS   12u
#define APP_ATU_FINE_SAMPLES     3u
#define APP_ATU_FINE_PASSES      2u
#define APP_ATU_FINE_STEP        1u
#define APP_ATU_FINE_STEP_MAX    4u
#define APP_ATU_TUNE_POWER_W     (4.0f)

/* Thresholds stay in SWR, the unit they are tuned in. The search itself runs
 * on gamma = (SWR - 1) / (SWR + 1): gamma is linear and has no pole, so it
 * resolves the match region better. A delta in SWR is not invariant over the
 * range, so the step-like thresholds are converted around the target SWR. */
#define APP_ATU_TARGET           (1.5f)
#define APP_ATU_SUCCESS          (3.0f)
#define APP_ATU_FINE_PLATEAU     (0.1f)
#define APP_ATU_BRANCH_CLOSE     (1.0f)
#define APP_ATU_SAMPLE_STABLE_SWR (1.0f)

/* Coarse L-row early exit: drop the remaining L rows once the per-row best
 * stops improving, or at once when a row is much worse than the branch best.
 * The abort is only armed on the first switch branch. */
#define APP_ATU_COARSE_WORSE_SWR (0.1f)
#define APP_ATU_COARSE_ABORT_SWR (1.0f)
#define APP_ATU_COARSE_WORSE_ROWS 2u

#define APP_ATU_SWR_TO_GAMMA(swr) (((swr) - 1.0f) / ((swr) + 1.0f))
#define APP_ATU_SWR_STEP_TO_GAMMA(step, ref) \
    (APP_ATU_SWR_TO_GAMMA((ref) + (step)) - APP_ATU_SWR_TO_GAMMA(ref))
#define APP_ATU_GAMMA_TARGET \
    APP_ATU_SWR_TO_GAMMA(APP_ATU_TARGET)
#define APP_ATU_GAMMA_SUCCESS \
    APP_ATU_SWR_TO_GAMMA(APP_ATU_SUCCESS)
#define APP_ATU_GAMMA_PLATEAU \
    APP_ATU_SWR_STEP_TO_GAMMA(APP_ATU_FINE_PLATEAU, APP_ATU_TARGET)
#define APP_ATU_GAMMA_BRANCH_CLOSE \
    APP_ATU_SWR_STEP_TO_GAMMA(APP_ATU_BRANCH_CLOSE, APP_ATU_TARGET)
#define APP_ATU_GAMMA_SAMPLE_STABLE \
    APP_ATU_SWR_STEP_TO_GAMMA(APP_ATU_SAMPLE_STABLE_SWR, APP_ATU_TARGET)

int  App_Atu_Init(void);

/* Request a tune and start the sequence (from host cmd 0x8004). No-op while a
 * tune is already running. */
void App_Atu_RequestTune(void);

/* Host slot 17: set an explicit network (C, L codes 0..63, sw 0/1). Applied
 * directly when the ATU is on; while it is off the network is only stored and
 * applied on the next switch-on. Ignored while a tune is running. */
void App_Atu_ApplyNetwork(uint8_t c, uint8_t l, uint8_t sw);

/* Host cmd12 `atue`: switch the ATU in/out of the TX path. Switching on applies
 * the stored network; switching off leaves the L/C/SW relays as they are. Used
 * as the restore target after a tune. */
void App_Atu_SetOn(bool on);

/* True from RequestTune() until the sequence finishes or aborts. ISR-safe. */
bool App_Atu_IsTuning(void);

/* Last measured/published network, packed as APP_ATU_PARAMS_*. */
uint32_t App_Atu_GetParams(void);
