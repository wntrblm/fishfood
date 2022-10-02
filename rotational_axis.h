#pragma once

#include "pico/time.h"
#include "stepper.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct RotationalAxis {
    char name;

    struct Stepper* stepper;

    float steps_per_deg;

    // internal state
    int32_t _delta_steps;
    int64_t _step_interval;
    absolute_time_t _next_step_at;
};

void RotationalAxis_init(struct RotationalAxis* m, char name, struct Stepper* stepper);
void RotationalAxis_start_move(volatile struct RotationalAxis* m, float dest_deg);
void RotationalAxis_wait_for_move(volatile struct RotationalAxis* m);
void RotationalAxis_step(volatile struct RotationalAxis* m);
inline bool RotationalAxis_is_moving(volatile struct RotationalAxis* m) { return m->_delta_steps != 0; }
void RotationalAxis_stop(volatile struct RotationalAxis* m);
float RotationalAxis_get_position_deg(volatile struct RotationalAxis* m);
