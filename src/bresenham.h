#pragma once

#include <stdbool.h>
#include <stdint.h>

struct Bresenham {
    // start point
    int32_t x0;
    int32_t y0;
    // end point
    int32_t x1;
    int32_t y1;

    // distance on each axis
    int32_t _dx;
    int32_t _dy;

    // error accumulator
    int32_t _diff;
    int32_t _y;
};

// This assumes X is always the major axis (the one moving the most) and X
// is always positive.
static inline void Bresenham_init(struct Bresenham* b, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    b->x0 = x0;
    b->y0 = y0;
    b->x1 = x1;
    b->y1 = y1;

    b->_dx = x1 - x0;
    b->_dy = y1 - y0;

    if (b->_dy < 0) {
        b->_dy = -(b->_dy);
    }

    b->_diff = 2 * (b->_dy - b->_dx);
    b->_y = y0;
};

static inline bool Bresenham_step(struct Bresenham* b) {
    bool stepped = false;

    if (b->_diff > 0) {
        stepped = true;
        b->_y += 1;
        b->_diff -= 2 * b->_dx;
    }
    b->_diff += 2 * b->_dy;

    return stepped;
}
