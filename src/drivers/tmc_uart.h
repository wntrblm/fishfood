/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "tmc2209.h"
#include <stdint.h>

void tmc_uart_read_write(
    struct TMC2209* tmc, uint8_t* send_buf, size_t send_len, uint8_t* receive_buf, size_t receive_len);
