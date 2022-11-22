/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include <stddef.h>
#include <stdint.h>

struct M42PinTableEntry {
    uint8_t pin;
    const char* name;
};

#ifdef JELLYFISH
#define PIN_CAM_LED 0
#define PIN_AUX_OUT 1

#define PIN_IN_1 2
#define PIN_IN_2 3

#define PIN_BLTOUCH_CONTROL 4
#define PIN_BLTOUCH_STATUS 5

#define PIN_I2C_SDA 10
#define PIN_I2C_SCL 11

#define PIN_AUX_LED 13
#define PIN_ACT_LED 15

#define PIN_UART_TX 16
#define PIN_UART_RX 17

#define PIN_M0_DIR 18
#define PIN_M0_STEP 19
#define PIN_M0_DIAG 20
#define PIN_M0_EN 21

#define PIN_M1_DIR 22
#define PIN_M1_STEP 23
#define PIN_M1_DIAG 24
#define PIN_M1_EN 25

#define PIN_M2_DIR 26
#define PIN_M2_STEP 27
#define PIN_M2_DIAG 28
#define PIN_M2_EN 29

static const struct M42PinTableEntry M42_PIN_TABLE[] = {
    {.pin = PIN_AUX_LED, .name = "Aux LED"},
    {.pin = PIN_ACT_LED, .name = "Act LED"},
    {.pin = PIN_AUX_OUT, .name = "Aux out"},
    {.pin = PIN_CAM_LED, .name = "Cam led"},
    {.pin = PIN_IN_1, .name = "In 1"},
    {.pin = PIN_IN_2, .name = "in 2"},
    {.pin = PIN_BLTOUCH_CONTROL, .name = "BLTouch control"},
    {.pin = PIN_BLTOUCH_STATUS, .name = "BLTouch status"},
};
#endif

#ifdef STARFISH
#define PIN_PUMP_A 0
#define PIN_PUMP_B 1
#define PIN_VALVE_A 2
#define PIN_VALVE_B 3
#define PIN_AUX_LED 4
#define PIN_ACT_LED 5

#define PIN_I2C_SDA 6
#define PIN_I2C_SCL 7

#define PIN_RS485_TX 8
#define PIN_RS485_RX 9
#define PIN_RS485_IN_EN 10
#define PIN_RS485_OUT_EN 11

#define PIN_IN_1 12
#define PIN_IN_2 13

#define PIN_AUX_OUT 14
#define PIN_CAM_LED 15

#define PIN_UART_TX 16
#define PIN_UART_RX 17

#define PIN_M2_DIR 18
#define PIN_M2_STEP 19
#define PIN_M2_DIAG 20
#define PIN_M2_EN 21

#define PIN_M1_DIR 22
#define PIN_M1_STEP 23
#define PIN_M1_DIAG 24
#define PIN_M1_EN 25

#define PIN_M0_DIR 26
#define PIN_M0_STEP 27
#define PIN_M0_DIAG 28
#define PIN_M0_EN 29

static const struct M42PinTableEntry M42_PIN_TABLE[] = {
    {.pin = PIN_PUMP_A, .name = "Pump A"},
    {.pin = PIN_PUMP_B, .name = "Pump B"},
    {.pin = PIN_VALVE_A, .name = "Valve A"},
    {.pin = PIN_VALVE_B, .name = "Valve B"},
    {.pin = PIN_AUX_LED, .name = "Aux LED"},
    {.pin = PIN_ACT_LED, .name = "Act LED"},
    {.pin = PIN_AUX_OUT, .name = "Aux out"},
    {.pin = PIN_CAM_LED, .name = "Cam led"},
    {.pin = PIN_IN_1, .name = "In 1"},
    {.pin = PIN_IN_2, .name = "in 2"},
};

#endif

static const size_t M42_PIN_TABLE_LEN = sizeof(M42_PIN_TABLE) / sizeof(M42_PIN_TABLE[0]);
