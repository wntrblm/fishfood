#pragma once

#include "drivers/tmc2209.h"
#include "pico/time.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct ZMotor {
    // hardware configuration
    struct TMC2209* tmc;
    uint32_t pin_enn;
    uint32_t pin_dir;
    uint32_t pin_step;
    uint32_t pin_diag;

    // Motion configuration. These members can be changed directly.

    // Maximum velocity in mm/s
    float velocity_mm_s;
    // Constant acceleration in mm/s^2
    float acceleration_mm_s2;
    // Homing sensitivity, used to set the TMC2209's stallguard threshold.
    // Higher = more sensitive.
    uint8_t homing_sensitivity;

    // The actual position of the motor measured in steps. This can be used
    // to derive the actual position in millimeters. (read only)
    int32_t actual_steps;

    // internal stepping state

    // Note: it takes two calls to ZMotor_step() to complete an actual motor
    // step. This is because the first call send the falling edge and the
    // second calls the rising edge.
    // Time between subsequent calls to ZMotor_step()
    int64_t _step_interval;
    // Time when the ZMotor_step() will actually step.
    absolute_time_t _next_step_at;

    // The state of the output pin, used to properly toggle the step output.
    bool _step_edge;
    // The direction the motor is going in (1 or -1).
    int8_t _dir;

    // internal acceleration and velocity state for the current move.

    // Total number of steps to spend accelerating.
    int32_t _accel_step_count;
    // Total number of steps to spend decelerating.
    int32_t _decel_step_count;
    // Total number of steps between accelerating and decelerating.
    int32_t _coast_step_count;
    // Total number of steps that need to be taken.
    int32_t _total_step_count;
    // Number of steps taken so far.
    int32_t _current_step_count;

    // Set whenever stallguard is triggered and causes the DIAG pin to rise.
    int8_t _crash_flag;
};

void ZMotor_init(
    struct ZMotor* m, struct TMC2209* tmc, uint32_t pin_enn, uint32_t pin_dir, uint32_t pin_step, uint32_t pin_diag);
bool ZMotor_setup(struct ZMotor* m);
void ZMotor_home(volatile struct ZMotor* m);
void ZMotor_start_move(volatile struct ZMotor* m, float dest_mm);
void ZMotor_wait_for_move(volatile struct ZMotor* m);
float ZMotor_get_position_mm(volatile struct ZMotor* m);
inline void ZMotor_reset_position(volatile struct ZMotor* m) {
    m->actual_steps = 0;
    m->_total_step_count = 0;
    m->_current_step_count = 0;
}
inline bool ZMotor_is_moving(volatile struct ZMotor* m) { return m->_total_step_count != 0; }
inline void ZMotor_stop(volatile struct ZMotor* m) {
    m->_total_step_count = 0;
    m->_current_step_count = 0;
}

void ZMotor_step(volatile struct ZMotor* m);
