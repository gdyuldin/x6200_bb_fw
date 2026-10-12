#include "app_fpga.h"

#include "app_scheduler.h"
#include "bsp_log.h"
#include "drv_fpga.h"

#define LOG_TAG "app.fpga"

/*
 * Arm a 0x02 read-status periodically. The driver sends it on the next PE10
 * tick and keeps the answer in RAM without logging.
 */
static void fpga_status_task(void) {
    Drv_Fpga_RequestStatus();
}

int App_Fpga_Init(const DrvFpgaCalib *calib) {
    DrvFpgaCalib new_calib = *calib;
    /* A better FPGA calibration to max bit per volt */
    new_calib.tx_cic_gain = 8;
    new_calib.dac_gain    = 0;
    int rc = Drv_Fpga_Init(&new_calib);

    if (rc != 0) {
        LOG_WARN("fpga init failed (%d), continuing", rc);
    }
    if (App_Scheduler_Add(fpga_status_task, 100u) != 0) {
        LOG_WARN("fpga status task not scheduled");
    }
    return rc;
}

void App_Fpga_SetRxFrequency(uint32_t freq_hz) {
    Drv_Fpga_SetRxFrequency(freq_hz);
}

void App_Fpga_SetTxFrequency(uint32_t freq_hz) {
    Drv_Fpga_SetTxFrequency(freq_hz);
}

void App_Fpga_TxMute(uint32_t ms) {
    Drv_Fpga_TxMute(ms);
}

void App_Fpga_SetSampleRate(uint8_t fftdec) {
    Drv_Fpga_SetSampleRate(fftdec);
}
