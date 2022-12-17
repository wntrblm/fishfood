/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "rs485.h"
#include "config/pins.h"
#include "config/serial.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/time.h"
#include <stdio.h>

#ifdef HAS_RS485

void rs485_init() {
    uart_init(RS485_UART_INST, RS485_BAUD);
    uart_set_fifo_enabled(RS485_UART_INST, false);
    gpio_set_function(PIN_RS485_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_RS485_RX, GPIO_FUNC_UART);
    gpio_init(PIN_RS485_IN_EN);
    gpio_set_dir(PIN_RS485_IN_EN, GPIO_OUT);
    gpio_put(PIN_RS485_IN_EN, 1);
    gpio_init(PIN_RS485_OUT_EN);
    gpio_set_dir(PIN_RS485_OUT_EN, GPIO_OUT);
    gpio_put(PIN_RS485_OUT_EN, 0);
}

void rs485_write(const uint8_t* write_buf, size_t write_len) {
    gpio_put(PIN_RS485_OUT_EN, 1);
    gpio_put(PIN_RS485_IN_EN, 1);
    uart_write_blocking(RS485_UART_INST, write_buf, write_len);
    uart_tx_wait_blocking(RS485_UART_INST);
    gpio_put(PIN_RS485_OUT_EN, 0);
    gpio_put(PIN_RS485_IN_EN, 0);
}

void rs485_read(uint8_t* read_buf, size_t read_len) {
    gpio_put(PIN_RS485_OUT_EN, 0);
    gpio_put(PIN_RS485_IN_EN, 0);
    uart_read_blocking(RS485_UART_INST, read_buf, read_len);
    gpio_put(PIN_RS485_OUT_EN, 0);
    gpio_put(PIN_RS485_IN_EN, 1);
}

#endif
