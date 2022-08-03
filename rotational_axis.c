#include "rotational_axis.h"
#include "config/motion.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <math.h>
#include <stdio.h>

/*
    Macros and constants
*/

/*
    Forward declarations
*/
static bool step_timer_callback(repeating_timer_t* rt);

/*
    Public methods
*/
void RotationalAxis_init(
    struct RotationalAxis* m, struct TMC2209* tmc, uint32_t pin_enn, uint32_t pin_dir, uint32_t pin_step) {
    m->tmc = tmc;
    m->pin_enn = pin_enn;
    m->pin_dir = pin_dir;
    m->pin_step = pin_step;
    m->actual_steps = 0;
    m->actual_deg = 0.0f;
    m->_delta_steps = 0;
    m->_step_edge = 0;
}

bool RotationalAxis_setup(struct RotationalAxis* m) {
    gpio_init(m->pin_enn);
    gpio_set_dir(m->pin_enn, GPIO_OUT);
    gpio_put(m->pin_enn, true);

    gpio_init(m->pin_dir);
    gpio_set_dir(m->pin_dir, GPIO_OUT);
    gpio_put(m->pin_dir, false);

    gpio_init(m->pin_step);
    gpio_set_dir(m->pin_step, GPIO_OUT);
    gpio_put(m->pin_step, false);

    if (!TMC2209_write_config(m->tmc, m->pin_enn)) {
        printf("Error configuring rotation motor TMC2209!");
        return false;
    }

    printf("Starting stepper timer...\n");
    add_repeating_timer_us(-100, step_timer_callback, (void*)(m), &(m->_step_timer));

    return true;
}

void RotationalAxis_move_to(volatile struct RotationalAxis* m, float dest_deg) {
    float delta_deg = dest_deg - m->actual_deg;
    float delta_steps = delta_deg * LR_STEPS_PER_DEG;
    m->_delta_steps = (int32_t)(roundf(delta_steps));
    printf("> Moving %0.2f deg (%i steps)\n", delta_deg, m->_delta_steps);
}

/*
    Private methods
*/

static bool step_timer_callback(repeating_timer_t* rt) {
    uint32_t irq_status = save_and_disable_interrupts();

    struct RotationalAxis* current_motor = (struct RotationalAxis*)(rt->user_data);

    if (current_motor->_delta_steps == 0) {
        restore_interrupts(irq_status);
        return true;
    }

    gpio_put(current_motor->pin_dir, current_motor->_delta_steps >= 0 ? 0 : 1);
    gpio_put(current_motor->pin_step, current_motor->_step_edge);
    current_motor->_step_edge = !current_motor->_step_edge;

    if (current_motor->_step_edge == false) {
        if (current_motor->_delta_steps > 0) {
            current_motor->_delta_steps--;
            current_motor->actual_steps++;
            current_motor->actual_deg += LR_DEGS_PER_STEP;
        } else {
            current_motor->_delta_steps++;
            current_motor->actual_steps--;
            current_motor->actual_deg -= LR_DEGS_PER_STEP;
        }
    }

    restore_interrupts(irq_status);
    return true;
}
