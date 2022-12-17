/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include <stddef.h>
#include <stdint.h>

void rs485_init();
void rs485_write(const uint8_t* write_buf, size_t write_len);
void rs485_read(uint8_t* read_buf, size_t read_len);
