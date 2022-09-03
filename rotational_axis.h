#pragma once

#include "drivers/tmc2209.h"
#include "pico/time.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct RotationalAxis {
    struct TMC2209* tmc;
    uint32_t pin_enn;
    uint32_t pin_dir;
    uint32_t pin_step;

    float steps_per_deg;

    int32_t actual_steps;

    // internal state
    int64_t _step_interval;
    absolute_time_t _next_step_at;

    int32_t _delta_steps;
    bool _step_edge;
};

void RotationalAxis_init(
    struct RotationalAxis* m,
    struct TMC2209* tmc,
    uint32_t pin_enn,
    uint32_t pin_dir,
    uint32_t pin_step,
    float steps_per_deg);
bool RotationalAxis_setup(struct RotationalAxis* m);
void RotationalAxis_start_move(volatile struct RotationalAxis* m, float dest_deg);
void RotationalAxis_wait_for_move(volatile struct RotationalAxis* m);
void RotationalAxis_step(volatile struct RotationalAxis* m);
inline bool RotationalAxis_is_moving(volatile struct RotationalAxis* m) { return m->_delta_steps != 0; }
void RotationalAxis_stop(volatile struct RotationalAxis* m);
float RotationalAxis_get_position_deg(volatile struct RotationalAxis* m);
