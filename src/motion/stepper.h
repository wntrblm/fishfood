#pragma once

#include "drivers/tmc2209.h"
#include <stddef.h>
#include <stdint.h>

struct Stepper {
    // Configuration
    struct TMC2209* tmc;
    uint8_t pin_enn;
    uint8_t pin_dir;
    uint8_t pin_step;
    uint8_t pin_diag;
    bool reversed;
    float run_current;
    float hold_current;

    // State
    // 1 for forwards -1 for backwards.
    int8_t direction;
    int32_t total_steps;
};

void Stepper_init(
    struct Stepper* s,
    struct TMC2209* tmc,
    uint8_t pin_enn,
    uint8_t pin_dir,
    uint8_t pin_step,
    uint8_t pin_diag,
    bool reversed,
    float run_current,
    float hold_current);
bool Stepper_setup(struct Stepper* s);
void Stepper_disable(struct Stepper* s);
void Stepper_enable(struct Stepper* s);
void Stepper_set_current(struct Stepper* s, float run_current, float hold_current);
void Stepper_enable_stealthchop(struct Stepper* s);
void Stepper_disable_stealthchop(struct Stepper* s);
void Stepper_enable_stallguard(struct Stepper* s, uint8_t threshold);
void Stepper_disable_stallguard(struct Stepper* s);
bool Stepper_stalled(struct Stepper* s);
void Stepper_update_direction(struct Stepper* s);
void Stepper_step(struct Stepper* s);
void Stepper_step_two(struct Stepper* s1, struct Stepper* s2);
