cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(picostep_project C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64)

add_executable(picostep
  main.c
  tmc2209.c
  tmc_config.c
)

target_include_directories(picostep PUBLIC ./)

pico_enable_stdio_usb(picostep 1)
pico_add_extra_outputs(picostep)

target_link_libraries(picostep pico_stdlib)
