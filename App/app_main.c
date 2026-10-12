#include "app_main.h"

#include "app_alc.h"
#include "app_cal.h"
#include "app_event.h"
#include "app_console.h"
#include "app_fpga.h"
#include "app_hostcmd.h"
#include "app_hostlink.h"
#include "app_hostlink_ext.h"
#include "app_housekeeping.h"
#include "app_nvram.h"
#include "app_pa_temp_comp.h"
#include "app_periph.h"
#include "app_power.h"
#include "app_prof.h"
#include "app_protect.h"
#include "app_ptt.h"
#include "app_atu.h"
#include "app_radio.h"
#include "app_radio_rfpath.h"
#include "app_radio_state.h"
#include "app_scheduler.h"
#include "app_telemetry.h"
#include "app_tx_monitor.h"
#include "bsp_init.h"
#include "bsp_log.h"
#include "sdr_spectrum.h"

#include "bsp_gpio.h"
#include "drv_clockgen.h"
#include "drv_hm_key.h"
#include "drv_pa_bias.h"
#include "main.h"

#define LOG_TAG "app.main"


static void app_init(void) {
    int rc;

    Bsp_Init();
    Bsp_Log_Init();
    LOG_INFO("x6200_bb_fw boot");
    App_Power_Init();
    /* 5P49V6965 clock generator: OUT1 = 122.88 MHz to the FPGA. Runs after the
     * PMIC rails and before the FPGA/NVRAM/DSP consumers. */
    rc = Drv_ClockGen_Init();
    if (rc == 0) {
        LOG_INFO("clock gen ok");
    } else {
        LOG_WARN("clock gen init failed (%d), continuing", rc);
    }
    App_Nvram_Init();
    /* Latch the persisted charger-enable preference before the protection tick
     * starts; a stored 0 keeps the charger inhibited from the first burst. */
    App_Protect_SetChargeEnabled(App_Nvram_ChargeEnabled());
    App_Fpga_Init(App_Nvram_FpgaCalib());
    Sdr_Spectrum_Init();
    App_Alc_Init();
    /* Event queues must exist before the radio can publish mode transitions. */
    if (App_Event_Init() != 0) {
        LOG_ERROR("event init failed, continuing without dispatcher");
    }
    /* PTT arbiter before the radio: the CW keyer (started with the codec) pushes
     * its TX request into it. */
    if (App_Ptt_Init() != 0) {
        LOG_ERROR("ptt init failed, continuing without ptt arbiter");
    }
    App_Radio_Init();
    App_RadioState_Init();
    App_TxMonitor_Init();
    App_Periph_Init();
    (void)Drv_PaBias_Init(App_Periph_HwRev(), App_Nvram_Calib()->bias_drive,
                          App_Nvram_Calib()->bias_final);
    if (App_PaTempComp_Init() != 0) {
        LOG_ERROR("pa temp comp init failed, continuing without compensation");
    }
    App_RadioRfPath_Init();
    App_Atu_Init();
    /* Extended host page before the transport: the write hook routes into it. */
    App_HostLinkExt_Init();
    App_HostLink_Init();
    App_Protect_Init();
    if (Drv_HmKey_Init() != 0) {
        LOG_ERROR("hand-mic key init failed, continuing without hkey");
    }
    App_Telemetry_Init();
    App_HostCmd_Init();
    rc = App_CalBias_Init();
    if (rc == 0) {
        rc = App_CalAlc_Init();
    }
    if (rc == 0) {
        rc = App_CalComp_Init();
    }
    App_Prof_Init();
    App_Console_Init();
    App_Housekeeping_Init();

    // Check PG13 for unlock FPGA
    // Bsp_Gpio_Write((BspPin){GPIOG, GPIO_PIN_13}, GPIO_PIN_RESET);
}

void App_Main(void) {
    app_init();
    App_Scheduler_Run();
}
