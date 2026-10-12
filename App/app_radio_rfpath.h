#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * RF front-end path control: preamp enable, attenuator, HMC472A gain ladder and
 * the analog band filters driven by the active frequency.
 *
 * preamp/att/gain sit on the preamp expander PCA9535 @0x20 (I2C1): preamp in
 * bit 14, attenuator in bit 13 and the HMC472A 6-bit ladder in bits [5:0]; the
 * BPF band code shares the same expander in bits [11:8]. The LPF (latching
 * relays) sits on the band-relay expander @0x21 in bits [15:6]. The expander
 * shadow is owned by Devices/drv_pca9535, so every setter is a masked
 * read-modify-write.
 *
 * The band-voltage analog output (PA3 PWM) is a third band-driven actuator and
 * is set from the same SetFreq() call.
 *
 * SetFreq() writes the BPF synchronously and only updates the LPF target; the
 * non-blocking relay pulse sequence runs from App_RadioRfPath_Task() so the main
 * loop is never held. A fast VFO sweep collapses intermediate relays and plays
 * only the final sequence. Index/code helpers are pure and exported for tests.
 *
 * rfg is the host-facing RF gain 0..100; GainCode() maps it to the HMC472A code
 * the hardware expects (higher rfg = less attenuation; the exact formula is in
 * the .c). SetAtt/SetPre take the boolean state of the active VFO; polarity is
 * 1 = on and is corrected here if the board proves active-low.
 */

#define APP_RADIO_RFPATH_RFG_MAX         100u
#define APP_RADIO_RFPATH_BPF_COUNT       11u
#define APP_RADIO_RFPATH_LPF_COUNT       9u
#define APP_RADIO_RFPATH_BAND_VOLT_COUNT 11u

int  App_RadioRfPath_Init(void);
void App_RadioRfPath_SetFreq(uint32_t freq_hz);
/* True when the LPF target would move for freq_hz; does not change the target.
 * Used to decide whether a TX branch change needs the FPGA mute. */
bool App_RadioRfPath_LpfWillChange(uint32_t freq_hz);
void App_RadioRfPath_SetRfg(uint8_t rfg);
void App_RadioRfPath_SetAtt(bool on);
void App_RadioRfPath_SetPre(bool on);
/* Net gain in dB of the RX front-end currently applied between the antenna and
 * the measured level (positive = gain), so a measured level can be referred
 * back to the antenna by subtracting it. */
float App_RadioRfPath_GetRxGainDb(void);

uint8_t  App_RadioRfPath_BpfIndex(uint32_t freq_hz);
uint16_t App_RadioRfPath_BpfCode(uint8_t index);
uint8_t  App_RadioRfPath_LpfIndex(uint32_t freq_hz);
uint16_t App_RadioRfPath_LpfCode(uint8_t index);
uint8_t  App_RadioRfPath_GainCode(uint8_t rfg);
/* Band-voltage output level in millivolts for freq_hz (PA3 PWM), keyed off the
 * amateur band. Pure, so the band table can be tested without hardware. */
uint16_t App_RadioRfPath_BandVoltMv(uint32_t freq_hz);
