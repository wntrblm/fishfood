#pragma once

#include <stdbool.h>
#include <stdint.h>

#define GPIO_OUT 1
#define GPIO_IN 0

static inline void gpio_init(uint32_t gpio) {}

static inline void gpio_set_dir(uint32_t gpio, bool out) {}

static inline bool gpio_get(uint32_t gpio) { return false; }
static inline void gpio_put(uint32_t gpio, bool value) {}

static inline void gpio_pull_up(uint32_t gpio) {}
static inline void gpio_pull_down(uint32_t gpio) {}
