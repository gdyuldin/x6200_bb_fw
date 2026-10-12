#pragma once

/*
 * IWDG1 servicing (reload only).
 *
 * Invariant (RM0468 §50.4): IWDG1 is started and configured by the bootloader
 * before the application jump (KR=0xCCCC start, 0x5555 unlock, PR=6 /256,
 * RLR=0xFFF, WINR=0xFFF). This module writes ONLY the reload key 0xAAAA
 * (RM0468 §50.4.1). Never write 0xCCCC (start), 0x5555 (unlock), PR, RLR or
 * WINR here. If self-start is ever required it must be an explicit, documented
 * deviation from this contract.
 *
 * Long/blocking-op rule: any loop that can block a meaningful fraction of the
 * IWDG timeout must call Bsp_Iwdg_Kick() at a bounded cadence inside its
 * loop — once per EEPROM page write (AT24C256C, 64-byte pages), once per
 * bounded flash write/erase chunk, once per redesign_filters chunk, and
 * optionally once per PMIC register-group write. Normal runtime is covered by
 * the 1 s housekeeping task (App_Housekeeping_Init).
 *
 * Reset-cause rule: an IWDG reset returns via the bootloader
 * (RCC_RSR.IWDG1RSTF) and intersects the RMVF-clear IAP hand-off. Never clear
 * RCC_RSR.RMVF here — that belongs solely to Bsp_Boot_EnterIap().
 */
void Bsp_Iwdg_Kick(void);
