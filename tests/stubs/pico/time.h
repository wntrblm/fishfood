#pragma once

#include <stdint.h>

typedef uint64_t absolute_time_t;

static inline uint64_t time_us_64() { return 0; }

static inline uint64_t to_us_since_boot(absolute_time_t t) { return t; }
static inline void update_us_since_boot(absolute_time_t* t, uint64_t us_since_boot) { *t = us_since_boot; }

static inline absolute_time_t get_absolute_time(void) {
    absolute_time_t t;
    update_us_since_boot(&t, time_us_64());
    return t;
}

static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return (int64_t)(to_us_since_boot(to) - to_us_since_boot(from));
}

static inline absolute_time_t delayed_by_us(const absolute_time_t t, uint64_t us) {
    absolute_time_t t2;
    uint64_t base = to_us_since_boot(t);
    uint64_t delayed = base + us;
    if ((int64_t)delayed < 0) {
        delayed = INT64_MAX;
    }
    update_us_since_boot(&t2, delayed);
    return t2;
}

static inline absolute_time_t make_timeout_time_us(uint64_t us) { return delayed_by_us(get_absolute_time(), us); }

static inline void sleep_us(uint64_t us) {}
