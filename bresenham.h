#pragma once


struct Bresenham {
    // start point
    int x0;
    int y0;
    // end point
    int x1;
    int y1;

    // distance on each axis
    int _dx;
    int _dy;

    // error accumulator
    int _diff;
    int _y;
};

// This assumes X is always the major axis (the one moving the most) and X
// is always positive.
void Bresenham_init(struct Bresenham* b, int x0, int y0, int x1, int y1) {
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

bool Bresenham_step(struct Bresenham* b) {
    bool stepped = false;

    if(b->_diff > 0) {
        stepped = true;
        b->_y += 1;
        b->_diff -= 2 * b->_dx;
    }
    b->_diff += 2 * b->_dy;

    return stepped;
}
