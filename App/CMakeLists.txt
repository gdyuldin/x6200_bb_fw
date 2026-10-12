add_library(App STATIC
    app_main.c
    app_event.c
    app_alc.c
    app_housekeeping.c
    app_protect.c
    app_ptt.c
    app_prof.c
    app_radio.c
    app_mic_pre.c
    app_radio_state.c
    app_radio_rfpath.c
    app_atu.c
    app_periph.c
    app_power.c
    app_nvram.c
    app_pa_temp_comp.c
    app_telemetry.c
    app_tx_monitor.c
    app_hostcmd.c
    app_hostlink.c
    app_hostlink_ext.c
    app_console.c
    app_cal_bias.c
    app_cal_alc.c
    app_cal_comp.c
    app_scheduler.c
    app_fpga.c
)

target_include_directories(App PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(App PUBLIC
    Devices
    Platform
)

add_subdirectory(SDR)
