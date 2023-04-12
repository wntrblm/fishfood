#pragma once

#include "pico/time.h"
#include "stepper.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINEAR_AXIS_LUT_COUNT 512

struct LinearAxisMovement {
    // Direction of travel, +1 or -1.
    int8_t direction;
    // Total number of steps to spend accelerating.
    int32_t accel_step_count;
    // Total number of steps to spend decelerating.
    int32_t decel_step_count;
    // Total number of steps between accelerating and decelerating.
    int32_t coast_step_count;
    // Total number of steps that need to be taken.
    int32_t total_step_count;
    // Number of steps taken so far.
    int32_t steps_taken;
    // acceleration look-up table
    uint16_t lut[LINEAR_AXIS_LUT_COUNT];
};

struct LinearAxis {
    char name;
    struct Stepper* stepper;
    struct Stepper* stepper2;

    // Motion configuration. These members can be changed directly.

    float steps_per_mm;
    // Maximum velocity in mm/s
    float velocity_mm_s;
    // Constant acceleration in mm/s^2
    float acceleration_mm_s2;
    // Which direction to home, either -1 for backwards or +1 for forwards.
    int8_t homing_direction;
    // How far to try to move during homing.
    float homing_distance_mm;
    // How far to move back before re-homing.
    float homing_bounce_mm;
    // Homing velocity and acceleration
    float homing_velocity_mm_s;
    float homing_acceleration_mm_s2;
    // Homing sensitivity, used to set the TMC2209's stallguard threshold.
    // Higher = more sensitive.
    uint8_t homing_sensitivity;
    // Endstop GPIO, if using endstop
    uint8_t endstop;

    // internal stepping state

    // Note: it takes two calls to LinearAxis_step() to complete an actual motor
    // step. This is because the first call send the falling edge and the
    // second calls the rising edge.
    // Time between subsequent calls to LinearAxis_step()
    int64_t _step_interval;
    // Time when the LinearAxis_step() will actually step.
    absolute_time_t _next_step_at;

    // internal acceleration and velocity state for the current move.
    struct LinearAxisMovement _current_move;
};

void LinearAxis_init(struct LinearAxis* m, char name, struct Stepper* stepper);

inline void LinearAxis_setup_dual(struct LinearAxis* m, struct Stepper* stepper) { m->stepper2 = stepper; }

void LinearAxis_sensorless_home(struct LinearAxis* m);
void LinearAxis_endstop_home(struct LinearAxis* m);

struct LinearAxisMovement LinearAxis_calculate_move(struct LinearAxis* m, float dest_mm);

void LinearAxis_start_move(struct LinearAxis* m, struct LinearAxisMovement move);

void LinearAxis_wait_for_move(struct LinearAxis* m);

float LinearAxis_get_position_mm(struct LinearAxis* m);

void LinearAxis_set_position_mm(struct LinearAxis* m, float mm);

static inline void LinearAxis_reset_position(struct LinearAxis* m) {
    m->stepper->total_steps = 0;
    m->_current_move = (struct LinearAxisMovement){};
}

static inline bool LinearAxis_is_moving(struct LinearAxis* m) { return m->_current_move.total_step_count != 0; }

static inline void LinearAxis_stop(struct LinearAxis* m) { m->_current_move = (struct LinearAxisMovement){}; }

bool LinearAxis_timed_step(struct LinearAxis* m);

void LinearAxis_direct_step(struct LinearAxis* m);

void LinearAxis_lookup_step_interval(struct LinearAxis* m);

uint32_t LinearAxisMovement_calculate_lut_entry(struct LinearAxis* a, uint32_t steps);
