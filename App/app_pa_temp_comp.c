#include "app_pa_temp_comp.h"

#include <stddef.h>
#include <stdint.h>

#include "board_config.h"
#include "bsp_irq.h"
#include "drv_adc_mon.h"
#include "drv_pa_bias.h"

#define APP_PA_TEMP_COMP_READ_DIV 2u
#define APP_PA_TEMP_COMP_APPLY_DIV 4u
#define APP_PA_TEMP_COMP_EMA_DIV 8
#define APP_PA_TEMP_COMP_STEP_Q8 (2 * 256)
#define APP_PA_TEMP_COMP_DAC_FULL_SCALE 4095

static int32_t  s_temp_ema_mc;
static bool     s_seeded;
static bool     s_temp_valid;
static int32_t  s_target_final_q8;
static int32_t  s_applied_final_q8;
static int32_t  s_written_final_codes;
static int32_t  s_target_drive_q8;
static int32_t  s_applied_drive_q8;
static int32_t  s_written_drive_codes;
static uint32_t s_tick;

static int32_t corr_from_temp(int32_t temp_mc, int32_t k_tenths) {
    int64_t delta = (int64_t)temp_mc - BOARD_PA_BIAS_TEMP_REF_MC;
    int64_t product =
        delta * APP_PA_TEMP_COMP_DAC_FULL_SCALE * k_tenths * 256;
    int64_t corr_q8 = -(product / (10000 * (int64_t)DRV_ADC_MON_VREF_MV));

    if (corr_q8 > 0) {
        corr_q8 = 0;
    }
    return (int32_t)corr_q8;
}

static int32_t slew(int32_t applied, int32_t target) {
    if (applied > target) {
        int32_t step = applied - target;
        applied -= (step > APP_PA_TEMP_COMP_STEP_Q8)
                       ? APP_PA_TEMP_COMP_STEP_Q8
                       : step;
    } else if (applied < target) {
        int32_t step = target - applied;
        applied += (step > APP_PA_TEMP_COMP_STEP_Q8)
                       ? APP_PA_TEMP_COMP_STEP_Q8
                       : step;
    }
    return applied;
}

static void write_offsets(void) {
    int32_t drive_codes = s_applied_drive_q8 >> 8;
    int32_t final_codes = s_applied_final_q8 >> 8;

    if (drive_codes == s_written_drive_codes &&
        final_codes == s_written_final_codes) {
        return;
    }
    s_written_drive_codes = drive_codes;
    s_written_final_codes = final_codes;
    (void)Drv_PaBias_SetOffset((int16_t)drive_codes, (int16_t)final_codes);
}

static void seed_from(int32_t temp_mc) {
    s_temp_ema_mc = temp_mc;
    s_seeded = true;
    s_target_final_q8 = corr_from_temp(
        temp_mc, BOARD_PA_BIAS_TEMP_K_FINAL_TENTHS_MV_PER_C);
    s_applied_final_q8 = s_target_final_q8;
    s_target_drive_q8 = corr_from_temp(
        temp_mc, BOARD_PA_BIAS_TEMP_K_DRIVE_TENTHS_MV_PER_C);
    s_applied_drive_q8 = s_target_drive_q8;
    s_written_final_codes = 0;
    s_written_drive_codes = 0;
    write_offsets();
}

static void update_target(int32_t temp_mc) {
    s_temp_ema_mc += (temp_mc - s_temp_ema_mc) / APP_PA_TEMP_COMP_EMA_DIV;
    s_target_final_q8 = corr_from_temp(
        s_temp_ema_mc, BOARD_PA_BIAS_TEMP_K_FINAL_TENTHS_MV_PER_C);
    s_target_drive_q8 = corr_from_temp(
        s_temp_ema_mc, BOARD_PA_BIAS_TEMP_K_DRIVE_TENTHS_MV_PER_C);
}

int App_PaTempComp_Init(void) {
    int32_t temp_mc;

    s_temp_ema_mc = 0;
    s_seeded = false;
    s_temp_valid = false;
    s_target_final_q8 = 0;
    s_applied_final_q8 = 0;
    s_written_final_codes = 0;
    s_target_drive_q8 = 0;
    s_applied_drive_q8 = 0;
    s_written_drive_codes = 0;
    s_tick = 0u;

    if (Drv_AdcMon_ReadPaTemp(&temp_mc) == 0) {
        s_temp_valid = true;
        seed_from(temp_mc);
    }
    return 0;
}

void App_PaTempComp_Tick10ms(void) {
    s_tick++;

    if ((s_tick % APP_PA_TEMP_COMP_READ_DIV) == 0u) {
        int32_t temp_mc;

        if (Drv_AdcMon_ReadPaTemp(&temp_mc) == 0) {
            if (!s_seeded) {
                seed_from(temp_mc);
            } else {
                update_target(temp_mc);
            }
            s_temp_valid = true;
        } else {
            s_temp_valid = false;
        }
    }

    if ((s_tick % APP_PA_TEMP_COMP_APPLY_DIV) != 0u || !s_temp_valid) {
        return;
    }

    s_applied_final_q8 = slew(s_applied_final_q8, s_target_final_q8);
    s_applied_drive_q8 = slew(s_applied_drive_q8, s_target_drive_q8);
    write_offsets();
}

void App_PaTempComp_GetStatus(AppPaTempCompStatus *out) {
    uint32_t irq;

    if (out == 0) {
        return;
    }
    irq = Bsp_Irq_SaveLow();
    out->ema_mc = s_temp_ema_mc;
    out->target_final_q8 = s_target_final_q8;
    out->applied_final_q8 = s_applied_final_q8;
    out->target_drive_q8 = s_target_drive_q8;
    out->applied_drive_q8 = s_applied_drive_q8;
    out->valid = s_temp_valid;
    out->seeded = s_seeded;
    Bsp_Irq_RestoreLow(irq);
}
