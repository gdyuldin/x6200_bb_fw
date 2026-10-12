add_library(Platform STATIC
    bsp_init.c
    bsp_boot.c
    bsp_band_volt.c
    bsp_board.c
    bsp_cordic.c
    bsp_crc.c
    bsp_exti.c
    bsp_gpio.c
    bsp_hm_key.c
    bsp_i2c_bus.c
    bsp_i2c_slave.c
    bsp_iwdg.c
    bsp_log.c
    bsp_pendsv.c
    bsp_prof.c
    bsp_uart.c
    bsp_time.c
)

target_include_directories(Platform PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Platform PUBLIC
    stm32cubemx
)
