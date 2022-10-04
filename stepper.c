#include "stepper.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include <stdio.h>

/*
    Public functions
*/

void Stepper_init(
    struct Stepper* s,
    struct TMC2209* tmc,
    uint8_t pin_enn,
    uint8_t pin_dir,
    uint8_t pin_step,
    uint8_t pin_diag,
    bool reversed,
    float run_current,
    float hold_current) {
    s->tmc = tmc;
    s->pin_enn = pin_enn;
    s->pin_dir = pin_dir;
    s->pin_step = pin_step;
    s->pin_diag = pin_diag;
    s->reversed = reversed;
    s->direction = 1;
    s->run_current = run_current;
    s->hold_current = hold_current;

    s->total_steps = 0;
    s->_step_edge = false;
}

bool Stepper_setup(struct Stepper* s) {
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

    Stepper_set_current(s, s->run_current, s->hold_current);

    return true;
}

void Stepper_disable(struct Stepper* s) { gpio_put(s->pin_enn, 1); }
void Stepper_enable(struct Stepper* s) { gpio_put(s->pin_enn, 0); }

void Stepper_set_current(struct Stepper* s, float run_current, float hold_current) {
    TMC2209_set_current(s->tmc, run_current, hold_current);
    s->run_current = run_current;
    s->hold_current = hold_current;
}

void Stepper_enable_stallguard(struct Stepper* s, uint8_t threshold) {
    TMC2209_write(s->tmc, TMC2209_SGTHRS, threshold);
}

void Stepper_disable_stallguard(struct Stepper* s) { TMC2209_write(s->tmc, TMC2209_SGTHRS, 0); }

bool Stepper_stalled(struct Stepper* s) {
    // TODO: At some point this might make sense to move to a GPIO IRQ,
    // but it's annoying since the built-in picosdk only provides *one*
    // callback for *all* gpio events.
    return gpio_get(s->pin_diag);
}

// Note: must be called *twice* to do a complete step.
// Returns true once a complete step has been made.
bool Stepper_step(struct Stepper* s) {
    gpio_put(s->pin_dir, s->direction > 0 ? !s->reversed : s->reversed);
    gpio_put(s->pin_step, s->_step_edge);
    s->_step_edge = !s->_step_edge;

    if (s->_step_edge == false) {
        s->total_steps += s->direction;
        return true;
    } else {
        return false;
    }
}
