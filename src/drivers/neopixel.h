/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "hardware/pio.h"
#include <stddef.h>
#include <stdint.h>

void Neopixel_init(uint32_t pin);
void Neopixel_write(uint8_t* pixels, size_t count);
void Neopixel_set(uint8_t* pixels, size_t n, uint8_t r, uint8_t g, uint8_t b);

inline void Neopixel_set_all(uint8_t* pixels, size_t count, uint8_t r, uint8_t g, uint8_t b) {
    for (size_t x = 0; x < count; x++) { Neopixel_set(pixels, x, r, g, b); }
}
