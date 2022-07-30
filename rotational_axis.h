#pragma once

#include "pico/time.h"
#include "drivers/tmc2209.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct RotationalAxis {
    struct TMC2209* tmc;
    uint32_t pin_enn;
    uint32_t pin_dir;
    uint32_t pin_step;

    int32_t actual_steps;
    float actual_deg;

    // internal state
    repeating_timer_t _step_timer;
    int32_t _delta_steps;
    bool _step_edge;
};

void RotationalAxis_init(struct RotationalAxis* m, struct TMC2209* tmc, uint32_t pin_enn, uint32_t pin_dir, uint32_t pin_step);
bool RotationalAxis_setup(struct RotationalAxis* m);
void RotationalAxis_move_to(volatile struct RotationalAxis* m, float dest_deg);
void RotationalAxis_is_moving(volatile struct RotationalAxis* m);
void RotationalAxis_stop(volatile struct RotationalAxis* m);
