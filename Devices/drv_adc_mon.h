#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * ADC1 monitoring scan: 6 channels in CubeMX rank order (IN1..IN5, IN8),
 * DMA1_Stream7 circular into a D2 buffer. Drv_AdcMon_Init() calibrates the
 * ADC and starts continuous conversions; the DMA completion ISR copies each
 * finished scan out of the DMA buffer. Drv_AdcMon_Sample() waits for the next
 * completed scan as a liveness check before the caller reads the latest copy.
 *
 * Conversions implemented here: external supply voltage (IN1, scaled to its
 * full-scale value below) and the PA thermistor (IN5, DRV_ADC_MON_NTC_* model).
 * The RF detector channels stay raw: their power/SWR conversion belongs to the
 * TX path.
 *
 * Drv_AdcMon_TakeStats() reports scan/error counters for DMA health logging.
 * A scan callback may be registered to consume the completed scan in the DMA
 * ISR context; keep it short.
 *
 * Returns 0 on success or a negative error code.
 */

/*
 * Nominal time between completed scans. On STM32H72x/H73x the ADC analog block
 * runs at fadc_ker_ck = adc_ker_ck / 2 (RM0468 "ADC clocks"), so the 96 MHz
 * PLL2 output with PRESC=/32 gives 1.5 MHz, not 3 MHz. A 12-bit conversion
 * takes 6.5 ADC clocks (RM0468 "Timing": Tconv = Tsample + Tsar = 32.5 + 6.5),
 * so one 6-channel scan is 6 x 39 = 234 clocks = 156 us.
 */
#define DRV_ADC_MON_SCAN_PERIOD_US 156u
typedef enum {
    DRV_ADC_MON_VEXT = 0,  /* IN1, PA1_C — external supply */
    DRV_ADC_MON_FWD,       /* IN2, PF11  — forward detector (raw) */
    DRV_ADC_MON_REFL,      /* IN3, PA6   — reflected detector (raw) */
    DRV_ADC_MON_SPARE,     /* IN4, PC4   — unused (raw) */
    DRV_ADC_MON_PA_NTC,    /* IN5, PB1   — PA thermistor */
    DRV_ADC_MON_EXT_ALC,   /* IN8, PC5   — external PA gain control (EXT ALC) */
    DRV_ADC_MON_CH_COUNT
} DrvAdcMonChannel;

/* External supply voltage at ADC1 IN1 full scale. */
#define DRV_ADC_MON_VEXT_FULL_SCALE_MV 19800u

/* ADC1 reference, nominal board 3.3 V rail. */
#define DRV_ADC_MON_VREF_MV 3300u

/*
 * PA thermistor model (ADC1 IN5): NTC to GND, pull-up to the rail below, node
 * straight into the ADC. The pull-up rail differs from Vref, so the reading is
 * not ratiometric and the conversion needs both rails. The electrical parameters
 * are in the DRV_ADC_MON_NTC_* defines.
 */
#define DRV_ADC_MON_NTC_R_PULLUP_OHM 100000u
#define DRV_ADC_MON_NTC_V_PULLUP_MV  5000u
#define DRV_ADC_MON_NTC_R25_OHM      10000u
#define DRV_ADC_MON_NTC_BETA_K       3950u

typedef struct {
    uint32_t scans;          /* completed scans since init */
    uint32_t overruns;       /* ADC overrun events seen by the scan ISR */
    uint32_t dma_errors;     /* HAL ADC/DMA error callbacks */
    uint32_t max_gap_cycles; /* longest ISR interval since the previous call */
} DrvAdcMonStats;

/* Called from the DMA ISR after each completed scan. Keep it short. */
typedef void (*DrvAdcMonScanCallback)(const volatile uint16_t *raw, size_t count);

int Drv_AdcMon_Init(void);
int Drv_AdcMon_Sample(void);
int Drv_AdcMon_ReadRaw(DrvAdcMonChannel ch, uint16_t *raw);
int Drv_AdcMon_ReadVextMv(uint16_t *mv);
int Drv_AdcMon_ReadPaTemp(int32_t *temp_mc);

int  Drv_AdcMon_SetScanCallback(DrvAdcMonScanCallback cb);
void Drv_AdcMon_TakeStats(DrvAdcMonStats *out);

/*
 * Inverse of the PA NTC model: the raw ADC1 code whose divider node equals the
 * given temperature. A protection threshold is turned into a raw-code
 * threshold once, so the hot path compares raw codes instead of running the
 * log conversion every sample. Returns 0 on success or -1 when the
 * temperature maps outside the ADC span.
 */
int Drv_AdcMon_PaRawFromTempMc(int32_t temp_mc, uint16_t *raw);
