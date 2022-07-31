#pragma once

#include "pico/time.h"
#include "drivers/tmc2209.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct ZMotor {
    struct TMC2209* tmc;
    uint32_t pin_enn;
    uint32_t pin_dir;
    uint32_t pin_step;
    uint32_t pin_diag;

    int32_t actual_steps;
    float actual_mm;

    int32_t max_steps;

    // internal state
    repeating_timer_t _step_timer;
    int32_t _delta_steps;
    int8_t _homing_state;
    int8_t _crash_flag;
    bool _step_edge;
};

void ZMotor_init(struct ZMotor* m, struct TMC2209* tmc, uint32_t pin_enn, uint32_t pin_dir, uint32_t pin_step, uint32_t pin_diag);
bool ZMotor_setup(struct ZMotor* m);
void ZMotor_home(volatile struct ZMotor* m);
void ZMotor_is_homed(volatile struct ZMotor* m);
void ZMotor_move_to(volatile struct ZMotor* m, float dest_mm);
void ZMotor_is_moving(volatile struct ZMotor* m);
void ZMotor_stop(volatile struct ZMotor* m);
void ZMotor_set_step_interval(volatile struct ZMotor* m, int64_t step_us);
void ZMotor_set_velocity(volatile struct ZMotor* m, float v_mm_s);
