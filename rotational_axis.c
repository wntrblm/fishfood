#include "rotational_axis.h"
#include "config/motion.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <math.h>
#include <stdio.h>

/*
    Public methods
*/
void RotationalAxis_init(
    struct RotationalAxis* m,
    struct TMC2209* tmc,
    uint32_t pin_enn,
    uint32_t pin_dir,
    uint32_t pin_step,
    float steps_per_deg) {
    m->tmc = tmc;
    m->pin_enn = pin_enn;
    m->pin_dir = pin_dir;
    m->pin_step = pin_step;
    m->steps_per_deg = steps_per_deg;
    m->actual_steps = 0;
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
        printf("Error configuring rotation motor TMC2209!\n");
        return false;
    }

    return true;
}

void RotationalAxis_start_move(volatile struct RotationalAxis* m, float dest_deg) {
    float delta_deg = dest_deg - RotationalAxis_get_position_deg(m);
    float delta_steps = delta_deg * m->steps_per_deg;

    uint32_t irq_status = save_and_disable_interrupts();
    m->_delta_steps = (int32_t)(roundf(delta_steps));
    m->_step_interval = 1000;
    m->_next_step_at = make_timeout_time_us(m->_step_interval);
    restore_interrupts(irq_status);

    printf("> Moving %0.2f deg (%i steps)\n", delta_deg, m->_delta_steps);
}


void RotationalAxis_wait_for_move(volatile struct RotationalAxis* m) {
    while(RotationalAxis_is_moving(m)) {
        tight_loop_contents();
    }

    printf("> Move finished at %0.3f (%i steps).\n", RotationalAxis_get_position_deg(m), m->actual_steps);
}

float RotationalAxis_get_position_deg(volatile struct RotationalAxis* m) {
    return ((float)(m->actual_steps)) * (1.0f / m->steps_per_deg);
}

/*
    Private methods
*/

void RotationalAxis_step(volatile struct RotationalAxis* m) {
    if (m->_delta_steps == 0) {
        goto exit;
    }

    if(absolute_time_diff_us(m->_next_step_at, get_absolute_time()) > 0) {
        goto exit;
    }

    gpio_put(m->pin_dir, m->_delta_steps >= 0 ? 0 : 1);
    gpio_put(m->pin_step, m->_step_edge);
    m->_step_edge = !m->_step_edge;

    if (m->_step_edge == false) {
        if (m->_delta_steps > 0) {
            m->_delta_steps--;
            m->actual_steps++;
        } else {
            m->_delta_steps++;
            m->actual_steps--;
        }
    }

exit:
    m->_next_step_at = make_timeout_time_us(m->_step_interval);
}
