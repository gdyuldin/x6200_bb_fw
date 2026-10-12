#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sdr_common.h"
#include "sdr_cw.h"

/*
 * Radio state model fed by host commands (App/app_hostlink) and consumed by
 * hardware glue. Values are staged: Begin() copies the committed state into a
 * working copy (unless a staged set is still pending), the command handlers
 * mutate the copy via the setters, and Apply() diffs the copy against the
 * committed state and only touches the hardware where something really changed.
 * The staged edits are committed once they form a valid set; the active-VFO mode
 * and fftdec must map to usable resample rates, so a pair the DSP layer cannot
 * run is coerced (fftdec is lowered to what the mode supports) before commit.
 * Only an ATU tune holds the whole staged set pending across host blocks. The
 * committed state is what the hardware reflects; getters read it.
 *
 * Setters are only valid between Begin() and Apply(); calling one outside a
 * staged edit logs an error and does nothing. Begin/Apply run on the main loop,
 * never from an ISR.
 */

typedef enum {
    APP_RADIO_VFO_A = 0,
    APP_RADIO_VFO_B = 1,
    APP_RADIO_VFO_COUNT
} AppRadioVfo;

typedef enum {
    APP_RADIO_MIC_BUILTIN = 0,
    APP_RADIO_MIC_HANDLE,
    APP_RADIO_MIC_AUTO
} AppRadioMicSel;

int  App_RadioState_Init(void);
void App_RadioState_Begin(void);
void App_RadioState_Apply(void);

/*
 * Host-configuration latch. It is set through the staged edit like the rest of
 * the state, so it commits atomically with the accepted configuration. Until it
 * is set the radio stays off: no RF relays move, no demod audio and no TX. The
 * setter logs an error and does nothing outside a staged edit.
 */
void App_RadioState_SetConfigured(void);
bool App_RadioState_IsConfigured(void);

void App_RadioState_SetFreq(AppRadioVfo vfo, uint32_t freq_hz);
void App_RadioState_SetActiveVfo(AppRadioVfo vfo);
void App_RadioState_SetSplit(bool split);
/*
 * SWR scan mode from host `sple_atue_trx` bit 4. Staged like split. While set,
 * the TX branch follows the active VFO even when split is enabled, so the host
 * sweep and the transmitted carrier use the same frequency.
 */
void App_RadioState_SetSwrScan(bool on);
bool App_RadioState_GetSwrScan(void);
/*
 * Voice recording mode from host `sple_atue_trx` bit 3. Staged like split.
 * While set the radio keeps its RF path and stays in RX: the codec loops the
 * microphone to HPL and the host line-in to HPR/LOR instead of demodulating.
 */
void App_RadioState_SetVoiceRec(bool on);
bool App_RadioState_GetVoiceRec(void);
void App_RadioState_SetSpmode(bool spmode);void App_RadioState_SetMode(AppRadioVfo vfo, SdrModulation mode);
void App_RadioState_SetAgcMode(AppRadioVfo vfo, uint32_t value);
void App_RadioState_SetFftdec(uint8_t fftdec);
/* Spectrum zoom exponent from host word 29 bits 12..15. Clamped to the maximum
 * decimation-stage count; only the spectrum display consumes it. */
void App_RadioState_SetFftzoomcw(uint8_t n);

void App_RadioState_SetSquelch(bool enable, uint8_t threshold,
                               uint8_t threshold_fm);

/*
 * Desired tuner_on (ATU switched into the TX path) from host cmd12 `atue`. It has
 * no direct hardware action here: the ATU applies it (and forces it to 1 while
 * tuning). Staged in the edit like the rest of the state.
 */
void App_RadioState_SetTunerOn(bool on);
bool App_RadioState_GetTunerOn(void);


void    App_RadioState_SetMonitorLevel(uint8_t level);
float   App_RadioState_GetMonitorK(void);
uint8_t App_RadioState_GetMonitorLevel(void);


void  App_RadioState_SetCwTone(uint32_t cw_tone_hz);

/*
 * CW keyer settings from hostlink cmd33. Speed is clamped to 5..50 WPM;
 * cwvol is the raw 0..63 host value (0 = silent, 30 = 0 dB, 1 dB per step).
 */
void App_RadioState_SetCwKeySpeed(uint32_t wpm);
void App_RadioState_SetCwKeyMode(SdrCwKeyMode mode);
void App_RadioState_SetCwIambicMode(SdrCwIambicMode mode);
void App_RadioState_SetCwVol(uint8_t cwvol);
void App_RadioState_SetCwTrain(bool training);
/*
 * CW timing from hostlink cmd34: qsk_time in ms (0..1000) and the dah/dot
 * duration ratio in tenths (25..45 = 2.5..4.5). Out-of-range values are
 * clamped to those bounds.
 */
void App_RadioState_SetCwQskTimeAndRatio(uint32_t qsk_ms,
                                         uint32_t dash_ratio_x10);

/*
 * att/pre are per-VFO and pushed for the active VFO only; rfg and txpwr are
 * global. txpwr is the TX power setpoint in watts (0 = 0 W).
 */
void    App_RadioState_SetAtt(AppRadioVfo vfo, bool on);
void    App_RadioState_SetPre(AppRadioVfo vfo, bool on);
void    App_RadioState_SetRfg(uint8_t rfg);
void    App_RadioState_SetTxPwr(float txpwr_w);

/*
 * rxvol is the host-facing RX volume 0..100 (0.5 dB per step). Values up to
 * 70 attenuate the speaker/phones driver; above 70 the left DAC gets digital
 * boost for a quiet band. The setter stores the raw value and derives the HPR
 * driver gain, its mute bit and the DAC multipliers.
 * GetLeftDacScale/GetRightDacScale are mode-aware for the ISR: in normal RX
 * the left DAC carries HPR and host, the right DAC carries USB/LOR; in
 * voice_rec the left DAC carries the microphone (fixed level) and the right
 * DAC carries the host line-in to HPR/LOR.
 * GetHprGainDb/IsMuted feed the codec driver from thread context.
 */
void   App_RadioState_SetRxVol(uint8_t rxvol);
float  App_RadioState_GetLeftDacScale(void);
float  App_RadioState_GetRightDacScale(void);
int8_t App_RadioState_GetHprGainDb(void);
bool   App_RadioState_IsMuted(void);

/*
 * rxfilter boundaries in Hz: f_low is the HPF (lower) edge and f_high the LPF
 * (upper) edge. Values are stored as-is (no validation) and have no hardware
 * action yet. The hostlink layer unpacks the wire word and calls the setter.
 */
void     App_RadioState_SetRxFilter(int32_t low_hz, int32_t high_hz);
void     App_RadioState_GetRxFilter(int32_t *low_hz, int32_t *high_hz);
int32_t App_RadioState_GetRxFilterLowHz(void);
int32_t App_RadioState_GetRxFilterHighHz(void);

/*
 * RIT/XIT tuning offsets in Hz from hostlink words 42/43, clamped to +-2000.
 * The final branch frequencies already include them: RX = active VFO + RIT,
 * TX = branch VFO + XIT. The RF path and FPGA are configured from these final
 * values.
 */
void    App_RadioState_SetRit(int32_t hz);
void    App_RadioState_SetXit(int32_t hz);

/* Final branch frequencies with split and RIT/XIT applied. A non-positive
 * result is reported as 0 ("unset"); callers that transmit must validate it. */
uint32_t    App_RadioState_GetRxFreq(void);
uint32_t    App_RadioState_GetTxFreq(void);
AppRadioVfo App_RadioState_GetActiveVfo(void);
SdrModulation App_RadioState_GetMode(AppRadioVfo vfo);
SdrModulation App_RadioState_GetRxMode(void);
/* Effective TX modulation: the inactive VFO when split, else the active VFO. */
SdrModulation App_RadioState_GetTxMode(void);
uint8_t     App_RadioState_GetFftdec(void);
bool        App_RadioState_IsSplit(void);
// True - phones mode
bool        App_RadioState_GetSpmode(void);

/*
 * Microphone selection and raw codec gains from hostlink words 20/21.
 * micsel uses AppRadioMicSel; gains are raw bytes and are clamped by the codec
 * driver where applicable. Values are staged in a staged edit like the rest of
 * the state.
 */
void    App_RadioState_SetMicSel(uint8_t sel);
void    App_RadioState_SetMicGains(uint8_t ling, uint8_t loutg,
                                   uint8_t imicg, uint8_t hmicg);
uint8_t App_RadioState_GetMicSel(void);
uint8_t App_RadioState_GetLing(void);
uint8_t App_RadioState_GetLoutg(void);
uint8_t App_RadioState_GetImicg(void);
uint8_t App_RadioState_GetHmicg(void);
bool        App_RadioState_GetAtt(AppRadioVfo vfo);
bool        App_RadioState_GetPre(AppRadioVfo vfo);
uint8_t     App_RadioState_GetRfg(void);
float       App_RadioState_GetTxPwr(void);
