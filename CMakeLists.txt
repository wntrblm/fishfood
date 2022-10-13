cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(picostep_project C CXX ASM)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)
add_compile_definitions(USBD_VID=0xCAFE)
add_compile_definitions(USBD_MANUFACTURER="Winterbloom")

list(APPEND PICOSTEP_SOURCES
  src/main.c
  src/linear_axis.c
  src/rotational_axis.c
  src/machine.c
  src/i2c_commands.c
  src/gpio_commands.c
  src/stepper.c
  src/drivers/tmc2209.c
  src/drivers/tmc2209_helper.c
  src/drivers/tmc_uart.c
  src/drivers/neopixel.c
  src/drivers/xgzp6857d.c
  src/littleg/littleg.c)

function(add_board_build board_name)
  add_executable(${board_name} ${PICOSTEP_SOURCES})

  pico_generate_pio_header(${board_name} ${CMAKE_CURRENT_LIST_DIR}/src/drivers/neopixel.pio)
  pico_enable_stdio_uart(${board_name} 0)
  pico_enable_stdio_usb(${board_name} 1)
  pico_add_extra_outputs(${board_name})

  target_include_directories(${board_name} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)
  target_link_libraries(${board_name} pico_stdlib hardware_pio hardware_dma hardware_irq hardware_i2c)
  target_compile_definitions(${board_name} PUBLIC PICOSTEP_BOARD="${board_name}" ${ARGN})
endfunction()

add_board_build(starfish STARFISH=1 USBD_PID=0xAA68 USBD_PRODUCT="Starfish")
add_board_build(jellyfish JELLYFISH=1 USBD_PID=0xAA69 USBD_PRODUCT="Jellyfish")
