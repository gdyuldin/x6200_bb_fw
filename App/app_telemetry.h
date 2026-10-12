#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Host telemetry flow packet (telemetry_flow_t, 552 B) sent on USART2
 * (BSP_UART_AUX) after APP_SCHED_EV_SPECTRUM_READY. The layout is the host
 * packet: magic, 512 spectrum bins, a flag word and the power/S-meter
 * scalars. Byte offsets and sizes are fixed by the host protocol; field names
 * are local only. The trailing crc is CRC-32/MPEG-2 over 0x000..0x223.
 */

#define TELEMETRY_FLOW_MAGIC     0xAA5555AAu
#define TELEMETRY_FLOW_FFT_BINS  512u
#define TELEMETRY_FLOW_SIZE      552u
/* Bytes covered by the trailing crc: 0x000..0x223. */
#define TELEMETRY_FLOW_CRC_LEN   548u
/* Worst-case SLIP frame: 0xC0 + 2x552 escaped bytes + 0xC0. */
#define APP_TELEMETRY_FRAME_MAX  1106u

/* flag word (offset 0x204). */
#define TELEMETRY_FLOW_FLAG_RESYNC            (1u << 0)
#define TELEMETRY_FLOW_FLAG_TX                (1u << 1)
#define TELEMETRY_FLOW_FLAG_ATU_STATUS        (1u << 2)
#define TELEMETRY_FLOW_FLAG_VEXT              (1u << 3)
#define TELEMETRY_FLOW_FLAG_CHARGING          (1u << 4)
#define TELEMETRY_FLOW_FLAG_BATTERY_NOT_FOUND (1u << 5)
#define TELEMETRY_FLOW_FLAG_BATTERY_HIGH_TEMP (1u << 6)
#define TELEMETRY_FLOW_FLAG_PA_HIGH_TEMP      (1u << 7)
#define TELEMETRY_FLOW_FLAG_POWER_KEY         (1u << 11)
#define TELEMETRY_FLOW_FLAG_SQL_MUTE          (1u << 12)
#define TELEMETRY_FLOW_FLAG_SQL_FM_MUTE       (1u << 13)
#define TELEMETRY_FLOW_FLAG_DC_IN_TOO_LOW     (1u << 14)
#define TELEMETRY_FLOW_FLAG_AUTO_OFF_ACTIVE   (1u << 15)
#define TELEMETRY_FLOW_FLAG_BATTERY_LOW_TEMP  (1u << 16)

typedef struct {
    uint32_t magic;
    uint8_t  samples[TELEMETRY_FLOW_FFT_BINS];
    uint32_t flag;
    int8_t   dbm;
    uint8_t  tx_power;
    uint8_t  vswr;
    uint8_t  alc_level;
    uint8_t  vext;              /* 0.1 V */
    uint8_t  vbat;              /* 0.1 V */
    int8_t   batcap;            /* % */
    int8_t   bat_temp;          /* degC */
    uint32_t atu_params;        /* APP_ATU_PARAMS_* packed L/C/switch */
    uint16_t shutdown_count;    /* seconds to auto-off */
    uint16_t reserved_3[5];
    uint32_t hkey;
    uint32_t crc;
} telemetry_flow_t;

_Static_assert(sizeof(telemetry_flow_t) == 0x228, "telemetry_flow_t size");
_Static_assert(offsetof(telemetry_flow_t, magic) == 0x000, "magic offset");
_Static_assert(offsetof(telemetry_flow_t, samples) == 0x004, "samples offset");
_Static_assert(offsetof(telemetry_flow_t, flag) == 0x204, "flag offset");
_Static_assert(offsetof(telemetry_flow_t, dbm) == 0x208, "dbm offset");
_Static_assert(offsetof(telemetry_flow_t, tx_power) == 0x209, "tx_power offset");
_Static_assert(offsetof(telemetry_flow_t, vswr) == 0x20a, "vswr offset");
_Static_assert(offsetof(telemetry_flow_t, alc_level) == 0x20b, "alc_level offset");
_Static_assert(offsetof(telemetry_flow_t, vext) == 0x20c, "vext offset");
_Static_assert(offsetof(telemetry_flow_t, vbat) == 0x20d, "vbat offset");
_Static_assert(offsetof(telemetry_flow_t, batcap) == 0x20e, "batcap offset");
_Static_assert(offsetof(telemetry_flow_t, bat_temp) == 0x20f, "bat_temp offset");
_Static_assert(offsetof(telemetry_flow_t, atu_params) == 0x210, "atu_params offset");
_Static_assert(offsetof(telemetry_flow_t, shutdown_count) == 0x214, "shutdown_count offset");
_Static_assert(offsetof(telemetry_flow_t, reserved_3) == 0x216, "reserved_3 offset");
_Static_assert(offsetof(telemetry_flow_t, hkey) == 0x220, "hkey offset");
_Static_assert(offsetof(telemetry_flow_t, crc) == 0x224, "crc offset");

typedef enum {
    APP_TELEMETRY_FRAMING_SLIP = 0,
    APP_TELEMETRY_FRAMING_RAW,
    APP_TELEMETRY_FRAMING_COBS
} AppTelemetryFraming;

int App_Telemetry_Init(void);

void App_Telemetry_SetTx(bool tx);

/* Select the wire framing. COBS is a stub and returns -ENOTSUP; the change is
 * ignored in that case. Takes effect on the next packet. */
int App_Telemetry_SetFraming(AppTelemetryFraming framing);
