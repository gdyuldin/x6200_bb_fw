add_library(Devices STATIC
    drv_pca9535.c
    drv_atu_relay.c
    drv_adc_mon.c
    drv_charger.c
    drv_clockgen.c
    drv_pa_bias.c
    drv_codec.c
    drv_eeprom.c
    drv_gauge.c
    drv_hm_key.c
    drv_hm_key_decode.c
    drv_nvram.c
    drv_pmic.c
    drv_fpga.c
)

target_include_directories(Devices PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Devices PUBLIC
    Platform
    m
)

# drv_fpga runs from ITCM (placed by *drv_fpga.* in the linker script), which
# relies on the object keeping its name. LTO merges objects into *.ltrans*.o,
# so this file must stay a plain object.
set_source_files_properties(drv_fpga.c PROPERTIES COMPILE_OPTIONS -fno-lto)
