/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "lib/graviton/src/graviton.h"
#include "pico/time.h"

struct GravitonIODriver {
    struct GravitonIO io;
    struct GravitonIODriverContext {
        uint32_t first_byte_timeout_ms;
        uint32_t total_timeout_ms;
        absolute_time_t first_byte_deadline;
        absolute_time_t total_deadline;
        bool seen_first_byte;
    } context;
};

void GravitonIODriver_init(
    struct GravitonIODriver* io_driver, uint32_t first_byte_timeout_ms, uint32_t total_timeout_ms);
