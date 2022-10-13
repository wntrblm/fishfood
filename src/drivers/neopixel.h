#pragma once

#include <stdint.h>
#include <stddef.h>
#include "hardware/pio.h"

void Neopixel_init(uint32_t pin);
void Neopixel_write(uint8_t* pixels, size_t count);
void Neopixel_set(uint8_t* pixels, size_t n, uint8_t r, uint8_t g, uint8_t b);

inline void Neopixel_set_all(uint8_t* pixels, size_t count, uint8_t r, uint8_t g, uint8_t b) {
    for(size_t x = 0; x < count; x++) {
        Neopixel_set(pixels, x, r, g, b);
    }
}
