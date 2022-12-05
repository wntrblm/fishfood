/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "neopixel.h"
#include "neopixel.pio.h"
#include "pico/time.h"

#define NEOPIXEL_FREQ_HZ 800000
#define NEOPIXEL_RESET_TIME_US 400

static uint32_t pio_sm = 0;
static int64_t last_write = 0;

void Neopixel_init(uint32_t pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    uint32_t pio_offset = pio_add_program(pio0, &neopixel_program);
    pio_sm = pio_claim_unused_sm(pio0, true);
    neopixel_program_init(pio0, pio_sm, pio_offset, pin, NEOPIXEL_FREQ_HZ);
}

void Neopixel_write(uint8_t* pixels, size_t count) {
    absolute_time_t t;
    update_us_since_boot(&t, last_write + NEOPIXEL_RESET_TIME_US);
    busy_wait_until(t);
    for (size_t x = 0; x < count * 3; x++) {
        uint32_t out = (pixels[x] << 24) | (pixels[x + 1] << 16) | (pixels[x + 2] << 8);
        pio_sm_put_blocking(pio0, pio_sm, out);
    }
    last_write = time_us_64();
}

void Neopixel_set(uint8_t* pixels, size_t n, uint8_t r, uint8_t g, uint8_t b) {
    size_t offset = n * 3;
    pixels[offset] = g;
    pixels[offset + 1] = r;
    pixels[offset + 2] = b;
}

void Neopixel_get(uint8_t* pixels, size_t n, uint8_t* r, uint8_t* g, uint8_t* b) {
    size_t offset = n * 3;
    *g = pixels[offset];
    *r = pixels[offset + 1];
    *b = pixels[offset + 2];
}
