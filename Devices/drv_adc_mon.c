#include "drv_adc_mon.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "bsp_irq.h"
#include "bsp_prof.h"
#include "bsp_time.h"

/*
 * Continuous conversion keeps ADSTART set and the DMA circular, so a scan is
 * completed every ~156 us without software triggers. The DMA ISR copies the
 * finished scan out of the circular buffer because the next scan starts
 * overwriting it immediately. The NTC conversion goes raw -> node voltage
 * (Vref = 3.3 V) -> Rntc = Rpu*V/(Vpu - V) from the +5 V pull-up divider, then
 * 1/T = 1/T0 + ln(R/R25)/B.
 */

#define DRV_ADC_MON_RAW_FULL_SCALE 4096u
#define DRV_ADC_MON_NTC_T0_K 298.15f
#define DRV_ADC_MON_KELVIN_TO_C 273.15f

static uint16_t s_raw[DRV_ADC_MON_CH_COUNT]
    __attribute__((section(".dma_d2"), aligned(32)));
static volatile uint16_t s_scan[DRV_ADC_MON_CH_COUNT];

static volatile uint32_t     s_scan_count;
static volatile uint32_t     s_overrun_count;
static volatile uint32_t     s_dma_error_count;
static volatile uint32_t     s_max_gap_cycles;
static uint32_t              s_last_scan_cycles;
static bool                  s_have_last_scan;
static DrvAdcMonScanCallback s_scan_callback;

int Drv_AdcMon_Init(void) {
    uint32_t t0;

    if (HAL_ADCEx_Calibration_Start(&hadc1, LL_ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        return -1;
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_raw, DRV_ADC_MON_CH_COUNT) !=
        HAL_OK) {
        return -1;
    }
    /* The ADC IRQ is disabled in CubeMX, so an enabled overrun interrupt would
     * leave a permanently pending NVIC request. The scan ISR polls OVR. */
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_OVR);
    /* Continuous mode starts converting immediately; wait for the first scan
     * so the read API never returns an uninitialised copy. */
    t0 = Bsp_Time_GetMs();
    while (s_scan_count == 0u) {
        if ((Bsp_Time_GetMs() - t0) > 10u) {
            return -1;
        }
    }
    return 0;
}

int Drv_AdcMon_Sample(void) {
    uint32_t t0    = Bsp_Time_GetMs();
    uint32_t start = s_scan_count;

    while (s_scan_count == start) {
        if ((Bsp_Time_GetMs() - t0) > 10u) {
            return -1;
        }
    }
    return 0;
}

/* Runs in the DMA1 Stream7 ISR. The copy must be short: the next continuous
 * scan starts overwriting s_raw as soon as the sequence completes. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    uint32_t now;
    size_t   i;

    if (hadc->Instance != ADC1) {
        return;
    }
    BSP_PROF_ISR_BEGIN(BSP_PROF_ISR_ADC_MON, prof_start);

    for (i = 0; i < DRV_ADC_MON_CH_COUNT; i++) {
        s_scan[i] = s_raw[i];
    }

    now = Bsp_Time_GetCycles();
    if (s_have_last_scan) {
        uint32_t gap = now - s_last_scan_cycles;
        if (gap > s_max_gap_cycles) {
            s_max_gap_cycles = gap;
        }
    } else {
        s_have_last_scan = true;
    }
    s_last_scan_cycles = now;
    s_scan_count++;

    if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_OVR)) {
        s_overrun_count++;
    }
    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

    if (s_scan_callback != 0) {
        s_scan_callback(s_scan, DRV_ADC_MON_CH_COUNT);
    }

    BSP_PROF_ISR_END(BSP_PROF_ISR_ADC_MON, prof_start);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        s_dma_error_count++;
    }
}

int Drv_AdcMon_SetScanCallback(DrvAdcMonScanCallback cb) {
    uint32_t irq = Bsp_Irq_SaveLow();

    s_scan_callback = cb;
    Bsp_Irq_RestoreLow(irq);
    return 0;
}

void Drv_AdcMon_TakeStats(DrvAdcMonStats *out) {
    uint32_t irq;

    if (out == 0) {
        return;
    }
    irq = Bsp_Irq_SaveLow();
    /* Also catch an overrun when the DMA completion callback did not run (for
     * example after a DMA error), so the health log sees the real cause. */
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_OVR)) {
        s_overrun_count++;
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    }
    out->scans = s_scan_count;
    out->overruns = s_overrun_count;
    out->dma_errors = s_dma_error_count;
    out->max_gap_cycles = s_max_gap_cycles;
    s_max_gap_cycles = 0u;
    Bsp_Irq_RestoreLow(irq);
}

int Drv_AdcMon_ReadRaw(DrvAdcMonChannel ch, uint16_t *raw) {
    if (raw == 0 || (unsigned)ch >= (unsigned)DRV_ADC_MON_CH_COUNT) {
        return -1;
    }
    *raw = s_scan[ch];
    return 0;
}

int Drv_AdcMon_ReadVextMv(uint16_t *mv) {
    if (mv == 0) {
        return -1;
    }
    *mv = (uint16_t)(((uint32_t)s_scan[DRV_ADC_MON_VEXT] *
                      DRV_ADC_MON_VEXT_FULL_SCALE_MV) >>
                     12);
    return 0;
}

int Drv_AdcMon_PaRawFromTempMc(int32_t temp_mc, uint16_t *raw) {
    float t_k;
    float r_ntc;
    float v_node_mv;
    float code;

    if (raw == 0) {
        return -1;
    }
    t_k = (float)temp_mc / 1000.0f + DRV_ADC_MON_KELVIN_TO_C;
    /* R = R25 * exp(B * (1/T - 1/T0)); node = Vpu * R / (Rpu + R). */
    r_ntc = (float)DRV_ADC_MON_NTC_R25_OHM *
            expf((float)DRV_ADC_MON_NTC_BETA_K *
                 (1.0f / t_k - 1.0f / DRV_ADC_MON_NTC_T0_K));
    v_node_mv = (float)DRV_ADC_MON_NTC_V_PULLUP_MV * r_ntc /
                ((float)DRV_ADC_MON_NTC_R_PULLUP_OHM + r_ntc);
    code = v_node_mv * (float)DRV_ADC_MON_RAW_FULL_SCALE /
           (float)DRV_ADC_MON_VREF_MV;
    if (code < 1.0f || code >= (float)(DRV_ADC_MON_RAW_FULL_SCALE - 1u)) {
        return -1;
    }
    *raw = (uint16_t)(code + 0.5f);
    return 0;
}

int Drv_AdcMon_ReadPaTemp(int32_t *temp_mc) {
    uint32_t raw = s_scan[DRV_ADC_MON_PA_NTC];
    float v_node_mv;
    float r_ntc;
    float t_c;

    if (temp_mc == 0) {
        return -1;
    }
    /* raw 0 = NTC shorted to GND; top of range = NTC open or Vnode at/over
     * Vref (the 5 V divider leaves the ADC span below about -30 C). */
    if (raw == 0u || raw >= (DRV_ADC_MON_RAW_FULL_SCALE - 1u)) {
        return -1;
    }
    v_node_mv = (float)raw * (float)DRV_ADC_MON_VREF_MV /
                (float)DRV_ADC_MON_RAW_FULL_SCALE;
    r_ntc = (float)DRV_ADC_MON_NTC_R_PULLUP_OHM * v_node_mv /
            ((float)DRV_ADC_MON_NTC_V_PULLUP_MV - v_node_mv);
    t_c = 1.0f / (1.0f / DRV_ADC_MON_NTC_T0_K +
                  logf(r_ntc / (float)DRV_ADC_MON_NTC_R25_OHM) /
                      (float)DRV_ADC_MON_NTC_BETA_K) -
          DRV_ADC_MON_KELVIN_TO_C;
    *temp_mc = (int32_t)(t_c * 1000.0f);
    return 0;
}
