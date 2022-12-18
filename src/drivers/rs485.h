/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include <stddef.h>
#include <stdint.h>

void rs485_init();
void rs485_write(const uint8_t* write_buf, size_t write_len);
void rs485_switch_to_write();
void rs485_switch_to_read();

#define RS485_READ_EMPTY -1
int rs485_read();
