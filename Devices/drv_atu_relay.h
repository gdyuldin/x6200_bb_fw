#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * ATU relay banks on the PCA9535 expanders, driven with non-blocking pulse
 * sequences so the main loop is never held.
 *
 * Ownership: app_radio_rfpath continues to own the LPF band relays in bits
 * [15:6] of expander 0x21 through the drv_pca9535 output shadow. The low bits
 * of 0x21 (the ATU switch and tuner_on, bits [2:0]) and all of 0x24 (the L and C
 * banks) belong to the ATU and are written raw, bypassing the shadow. A raw
 * write of the whole 0x21/0x24 word is safe for the LPF: its relays are
 * latching, so a zero only de-energizes the coils and never changes the closed
 * contact, and the shadow still reflects the last LPF code. Do NOT route these
 * writes through Drv_Pca9535_Write/WriteMasked, or the shadow (and thus the LPF
 * cache in app_radio_rfpath) would drift out of sync.
 *
 * The L/C expander (0x24) is configured as all-outputs here (drv_pca9535 only
 * configures 0x20/0x21); the hardware always has the ATU, so no degraded path.
 */

typedef enum {
    DRV_ATU_GROUP_L = 0,  /* sw1f bank, expander 0x24 */
    DRV_ATU_GROUP_C,      /* sw20 bank, expander 0x24 */
    DRV_ATU_GROUP_SWITCH, /* ATU C pre/post switch, expander 0x21 sel 0/1 */
    DRV_ATU_GROUP_ON,     /* tuner_on, expander 0x21 sel 0/1 */
    DRV_ATU_GROUP_COUNT
} DrvAtuGroup;

#define DRV_ATU_RELAY_XFER_TIMEOUT_MS 10u

/* Configure the 0x24 bank as outputs and poison the per-group cache so the
 * first Request always pulses. */
int Drv_AtuRelay_Init(void);

/* Ask for a group to select sel (L/C: 0..63 code, switch/tuner_on: 0/1). A no-op
 * when sel already equals the cached value (no pulse, no delays). A request
 * that arrives while a sequence is running is deferred until that sequence
 * finishes; the last request for a group wins. A running pulse is never
 * aborted, because an incomplete latching sequence leaves the relay state
 * undefined. */
void Drv_AtuRelay_Request(DrvAtuGroup g, uint8_t sel);

/* True while a pulse sequence is running or a deferred request is waiting. */
bool Drv_AtuRelay_Busy(void);

/* Advance the active sequence by one phase when its dwell has elapsed. Called
 * from the app_atu tick (main loop). */
void Drv_AtuRelay_Tick(void);
