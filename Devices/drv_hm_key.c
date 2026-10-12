#include "drv_hm_key.h"

#include "bsp_hm_key.h"
#include "bsp_log.h"
#include "bsp_time.h"
#include "drv_hm_key_decode.h"

#define LOG_TAG "drv.hm_key"

static void edge_handler(uint32_t period, uint32_t high, void *ctx) {
    (void)ctx;
    (void)Drv_HmKeyDecode_Edge(period, high, Bsp_Time_GetMs());
}

uint8_t Drv_HmKey_GetCode(void) {
    return Drv_HmKeyDecode_Get(Bsp_Time_GetMs());
}

int Drv_HmKey_Init(void) {
    Drv_HmKeyDecode_Reset();
    if (Bsp_HmKey_Start(edge_handler, 0) != 0) {
        LOG_ERROR("capture start failed");
        return -1;
    }
    LOG_INFO("receiver ready");
    return 0;
}