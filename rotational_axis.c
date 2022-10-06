#include "rotational_axis.h"
#include "config/motion.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/sync.h"
#include <math.h>
#include <stdio.h>

/*
    Public methods
*/
void RotationalAxis_init(struct RotationalAxis* m, char name, struct Stepper* stepper) {
    m->name = name;
    m->stepper = stepper;
    m->_delta_steps = 0;
}

void RotationalAxis_start_move(struct RotationalAxis* m, float dest_deg) {
    float delta_deg = dest_deg - RotationalAxis_get_position_deg(m);
    int32_t delta_steps = (int32_t)(roundf(delta_deg * m->steps_per_deg));
    float actual_delta_deg = delta_steps * (1.0f / m->steps_per_deg);

    m->stepper->direction = m->_delta_steps >= 0 ? +1 : -1;
    m->_delta_steps = delta_steps;
    m->_step_interval = 100;
    m->_next_step_at = make_timeout_time_us(m->_step_interval);

    printf("> Moving %c axis %0.2f deg (%i steps)\n", m->name, actual_delta_deg, m->_delta_steps);
}

void RotationalAxis_wait_for_move(struct RotationalAxis* m) {
    while (RotationalAxis_is_moving(m)) {
        RotationalAxis_step(m);
    }

    printf(
        "> %c axis moved to %0.3f (%i steps).\n", m->name, RotationalAxis_get_position_deg(m), m->stepper->total_steps);
}

float RotationalAxis_get_position_deg(struct RotationalAxis* m) {
    return ((float)(m->stepper->total_steps)) * (1.0f / m->steps_per_deg);
}

void RotationalAxis_step(struct RotationalAxis* m) {
    if (m->_delta_steps == 0) {
        return;
    }

    if (absolute_time_diff_us(get_absolute_time(), m->_next_step_at) > 0) {
        return;
    }

    Stepper_step(m->stepper);

    if (m->_delta_steps > 0) {
        m->_delta_steps--;
    } else {
        m->_delta_steps++;
    }

    m->_next_step_at = make_timeout_time_us(m->_step_interval);
}
