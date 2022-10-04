cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(picostep_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
add_compile_definitions(STARFISH=1)
# add_compile_definitions(JELLYFISH=1)

add_executable(picostep
  main.c
  linear_axis.c
  rotational_axis.c
  i2c_commands.c
  gpio_commands.c
  drivers/tmc2209.c
  drivers/tmc2209_helper.c
  drivers/tmc_uart.c
  drivers/neopixel.c
  drivers/xgzp6857d.c
  littleg/littleg.c
)

pico_generate_pio_header(picostep ${CMAKE_CURRENT_LIST_DIR}/drivers/neopixel.pio)

target_include_directories(picostep PUBLIC ${CMAKE_CURRENT_LIST_DIR})

pico_enable_stdio_uart(picostep 0)
pico_enable_stdio_usb(picostep 1)
pico_add_extra_outputs(picostep)

target_link_libraries(picostep pico_stdlib hardware_pio hardware_dma hardware_irq hardware_i2c)
