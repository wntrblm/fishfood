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

    // motion configuration
    float velocity_mm_s;
    float acceleration_mm_s2;
    uint8_t homing_sensitivity;

    // reported state
    int32_t actual_steps;
    float actual_mm;

    // internal stepping state
    repeating_timer_t _step_timer;
    bool _step_edge;
    int8_t _dir;

    // internal acceleration and velocity state
    int32_t _accel_step_count;
    int32_t _decel_step_count;
    int32_t _coast_step_count;
    int32_t _total_step_count;
    int32_t _step_count;

    // internal homing state
    int8_t _crash_flag;
};

void ZMotor_init(
    struct ZMotor* m, struct TMC2209* tmc, uint32_t pin_enn, uint32_t pin_dir, uint32_t pin_step, uint32_t pin_diag);
bool ZMotor_setup(struct ZMotor* m);
void ZMotor_home(volatile struct ZMotor* m);
void ZMotor_move_to(volatile struct ZMotor* m, float dest_mm);
inline void ZMotor_reset_position(volatile struct ZMotor* m) {
    m->actual_steps = 0;
    m->actual_mm = 0;
    m->_total_step_count = 0;
    m->_step_count = 0;
}
inline bool ZMotor_is_moving(volatile struct ZMotor* m) { return m->_total_step_count > 0 ? true : false; }
inline void ZMotor_stop(volatile struct ZMotor* m) {
    m->_total_step_count = 0;
    m->_step_count = 0;
}
inline void ZMotor_set_velocity(volatile struct ZMotor* m, float v_mm_s) { m->velocity_mm_s = v_mm_s; }
inline void ZMotor_set_acceleration(volatile struct ZMotor* m, float a_mm_s2) { m->acceleration_mm_s2 = a_mm_s2; }
inline void ZMotor_set_homing_sensitivity(volatile struct ZMotor* m, uint8_t sense) { m->homing_sensitivity = sense; }
