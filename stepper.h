#pragma once

#include "drivers/tmc2209.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

struct Stepper {
    // Configuration
    struct TMC2209* tmc;
    uint8_t pin_enn;
    uint8_t pin_dir;
    uint8_t pin_step;
    uint8_t pin_diag;
    bool reversed;

    // State
    // 1 for forwards -1 for backwards.
    int8_t direction;
    int32_t total_steps;

    // internal state
    bool _step_edge;
};

inline static void Stepper_init(
    struct Stepper* s, struct TMC2209* tmc, uint8_t pin_enn, uint8_t pin_dir, uint8_t pin_step, uint8_t pin_diag, bool reversed) {
    s->tmc = tmc;
    s->pin_enn = pin_enn;
    s->pin_dir = pin_dir;
    s->pin_step = pin_step;
    s->pin_diag = pin_diag;
    s->reversed = reversed;
    s->direction = 1;

    s->total_steps = 0;
    s->_step_edge = false;
}

inline static bool Stepper_setup(struct Stepper* s) {
    gpio_init(s->pin_enn);
    gpio_set_dir(s->pin_enn, GPIO_OUT);
    gpio_put(s->pin_enn, true);

    gpio_init(s->pin_dir);
    gpio_set_dir(s->pin_dir, GPIO_OUT);
    gpio_put(s->pin_dir, s->direction > 0 ? !s->reversed : s->reversed);

    gpio_init(s->pin_step);
    gpio_set_dir(s->pin_step, GPIO_OUT);
    gpio_put(s->pin_step, false);

    gpio_init(s->pin_diag);
    gpio_set_dir(s->pin_diag, GPIO_IN);
    gpio_pull_down(s->pin_diag);

    if (!TMC2209_write_config(s->tmc, s->pin_enn)) {
        printf("Error configuring TMC2209!");
        return false;
    }

    return true;
}

inline static void Stepper_disable(struct Stepper* s) { gpio_put(s->pin_enn, 1); }
inline static void Stepper_enable(struct Stepper* s) { gpio_put(s->pin_enn, 0); }

inline static void Stepper_set_current(struct Stepper* s, float current, float hold_current) {
    TMC2209_set_current(s->tmc, current, hold_current);
}

inline static void Stepper_enable_stallguard(struct Stepper* s, uint8_t threshold) {
    TMC2209_write(s->tmc, TMC2209_SGTHRS, threshold);
}

inline static void Stepper_disable_stallguard(struct Stepper* s) {
    TMC2209_write(s->tmc, TMC2209_SGTHRS, 0);
}

inline static bool Stepper_stalled(struct Stepper* s) {
    // TODO: At some point this might make sense to move to a GPIO IRQ,
    // but it's annoying since the built-in picosdk only provides *one*
    // callback for *all* gpio events.
    return gpio_get(s->pin_diag);
}

// Note: must be called *twice* to do a complete step.
// Returns true once a complete step has been made.
inline static bool Stepper_step(struct Stepper* s) {
    gpio_put(s->pin_dir, s->direction > 0 ? !s->reversed : s->reversed);
    gpio_put(s->pin_step, s->_step_edge);
    s->_step_edge = !s->_step_edge;

    if (s->_step_edge == false) {
        s->total_steps += s->direction;
    } else {
        return false;
    }
}
