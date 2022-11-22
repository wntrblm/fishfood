/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "tmc2209_registers.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct TMC2209;

typedef void (*TMC2209_uart_send_receive_func)(
    struct TMC2209* tmc, uint8_t* send_buf, size_t send_len, uint8_t* receive_buf, size_t receive_len);

struct TMC2209 {
    void* uart;
    uint8_t uart_address;
    /* Must be implemented by the calling program. */
    TMC2209_uart_send_receive_func uart_send_receive;
};

enum TMC2209_read_result {
    TMC_READ_OK = 0,
    TMC_READ_BAD_SYNC = -1,
    TMC_READ_BAD_ADDRESS = -2,
    TMC_READ_BAD_REGISTER = -3,
    TMC_READ_BAD_CRC = -4,
};

void TMC2209_init(
    struct TMC2209* tmc, void* uart, uint8_t uart_address, TMC2209_uart_send_receive_func uart_send_receive);

enum TMC2209_read_result TMC2209_read(struct TMC2209* tmc, uint8_t register_addr, uint32_t* out);

void TMC2209_write(struct TMC2209* tmc, uint8_t register_addr, uint32_t value);

uint8_t TMC2209_CRC8(uint8_t* data, size_t len);
