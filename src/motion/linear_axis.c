#include "linear_axis.h"
#include "config/motion.h"
#include "hardware/gpio.h"
#include "hardware/platform_defs.h"
#include "hardware/sync.h"
#include "report.h"
#include <math.h>
#include <stdlib.h>

/*
    Public methods
*/

void LinearAxis_init(struct LinearAxis* m, char name, struct Stepper* stepper) {
    m->name = name;
    m->stepper = stepper;

    m->velocity_mm_s = 100.0f;
    m->acceleration_mm_s2 = 1000.0f;
    m->homing_sensitivity = 100;
    m->endstop = 0;

    m->_current_move = (struct LinearAxisMovement){};
}

void stallguard_seek(struct LinearAxis* m, float dist_mm) {
    Stepper_enable_stealthchop(m->stepper);
    Stepper_disable_stallguard(m->stepper);

    LinearAxis_start_move(m, LinearAxis_calculate_move(m, dist_mm));

    bool check_for_stall = false;
    while (true) {
        LinearAxis_timed_step(m);

        // Once the axis is up to speed, enable stallguard and watch for stalls
        if (m->_current_move.steps_taken == m->_current_move.accel_step_count) {
            Stepper_enable_stallguard(m->stepper, m->homing_sensitivity);
            check_for_stall = true;
        }

        if (check_for_stall && Stepper_stalled(m->stepper)) {
            break;
        }
    }

    LinearAxis_stop(m);
    LinearAxis_reset_position(m);
    Stepper_disable_stallguard(m->stepper);
    Stepper_disable_stealthchop(m->stepper);
}

void LinearAxis_sensorless_home(struct LinearAxis* m) {
    // TODO/Note: If an axis has two motors, this only monitors StallGuard
    // on the first of the two motors. At some point it probably makes sense
    // to monitor both. In practice, this hasn't caused any issues so far.

    //
    // 1: Initial seek
    //
    report_debug_ln("homing %c axis with sensitivity at %u...", m->name, m->homing_sensitivity);

    float old_velocity = m->velocity_mm_s;
    float old_acceleration = m->acceleration_mm_s2;
    m->velocity_mm_s = m->homing_velocity_mm_s;
    m->acceleration_mm_s2 = m->homing_acceleration_mm_s2;
    m->stepper->total_steps = 0;

    stallguard_seek(m, m->homing_direction * m->homing_distance_mm);

    //
    // 2. Bounce
    //
    report_debug_ln("endstop found, bouncing...");

    LinearAxis_start_move(m, LinearAxis_calculate_move(m, -(m->homing_direction * m->homing_bounce_mm)));

    while (LinearAxis_is_moving(m)) { LinearAxis_timed_step(m); }

    //
    // 3. Re-seek
    //
    report_debug_ln("re-seeking...");

    m->velocity_mm_s = m->homing_velocity_mm_s;
    m->acceleration_mm_s2 = m->homing_acceleration_mm_s2;
    stallguard_seek(m, m->homing_direction * m->homing_bounce_mm * 2);

    m->velocity_mm_s = old_velocity;
    m->acceleration_mm_s2 = old_acceleration;
    report_result_ln("%c axis homed", m->name);
}

void endstop_seek(struct LinearAxis* m, float dist_mm) {
    LinearAxis_start_move(m, LinearAxis_calculate_move(m, dist_mm));

    while (gpio_get(m->endstop) != 1) { LinearAxis_timed_step(m); }

    LinearAxis_stop(m);
    LinearAxis_reset_position(m);
}

void LinearAxis_endstop_home(struct LinearAxis* m) {
    //
    // 1: Initial seek
    //
    report_info_ln("homing %c axis using endstop %u...", m->name, m->endstop);

    gpio_init(m->endstop);
    gpio_set_dir(m->endstop, GPIO_IN);
    gpio_pull_up(m->endstop);

    float old_velocity = m->velocity_mm_s;
    float old_acceleration = m->acceleration_mm_s2;
    m->velocity_mm_s = m->homing_velocity_mm_s;
    m->acceleration_mm_s2 = m->homing_acceleration_mm_s2;
    m->stepper->total_steps = 0;

    endstop_seek(m, m->homing_direction * m->homing_distance_mm);

    //
    // 2. Bounce
    //
    report_info_ln("endstop found, bouncing...");

    LinearAxis_start_move(m, LinearAxis_calculate_move(m, -(m->homing_direction * m->homing_bounce_mm)));

    while (LinearAxis_is_moving(m)) { LinearAxis_timed_step(m); }

    //
    // 3. Re-seek
    //
    report_info_ln("re-seeking...");

    m->velocity_mm_s = m->homing_velocity_mm_s / 5.0f;
    m->acceleration_mm_s2 = m->homing_acceleration_mm_s2 / 2.0f;
    endstop_seek(m, m->homing_direction * m->homing_bounce_mm * 2);

    m->velocity_mm_s = old_velocity;
    m->acceleration_mm_s2 = old_acceleration;
    report_result_ln("%c axis homed", m->name);
}

struct LinearAxisMovement LinearAxis_calculate_move(struct LinearAxis* m, float dest_mm) {
    // Calculate how far to move to bring the motor to the destination.
    // Do the calculation based on steps (integers) instead of mm (floats) to
    // ensure consistency.
    int32_t dest_steps = (int32_t)(lroundf(ceilf(dest_mm * m->steps_per_mm)));
    int32_t delta_steps = dest_steps - m->stepper->total_steps;
    int32_t dir = delta_steps < 0 ? -1 : 1;

    // Determine the number of steps needed to complete the move.
    int32_t total_step_count = abs(delta_steps);

    // Determine how long acceleration and deceleration will take and
    // how many steps will be spent in each of the three phases (accelerating,
    // coasting, decelerating).
    float accel_time_s = m->velocity_mm_s / m->acceleration_mm_s2;
    float accel_distance_mm = 0.5f * accel_time_s * m->velocity_mm_s;
    int32_t accel_step_count = (int32_t)(lroundf(accel_distance_mm * m->steps_per_mm));
    int32_t decel_step_count = accel_step_count;
    int32_t coast_step_count = total_step_count - accel_step_count - decel_step_count;

    // Check for the case where a move is too short to reach full velocity
    // and therefore has no coasting phase. In this case, the acceleration
    // and deceleration phases will each occupy one half of the total steps.
    if (coast_step_count <= 0) {
        accel_step_count = total_step_count / 2;
        // Note: use subtraction here instead of just setting it the same
        // as the acceleration step count. This accommodates odd amounts of
        // total steps and ensures that the correct amount of total steps
        // are taken. For example, if there are 11 total steps then
        // accel_step_count = 5 and decel_step_count = 6.
        decel_step_count = total_step_count - accel_step_count;
        coast_step_count = 0;
    }

    struct LinearAxisMovement movement = {
        .direction = dir,
        .accel_step_count = accel_step_count,
        .decel_step_count = decel_step_count,
        .coast_step_count = coast_step_count,
        .total_step_count = total_step_count,
        .steps_taken = 0,
    };

    // Generate the acceleration look-up table.
    for (size_t i = 0; i < LINEAR_AXIS_LUT_COUNT; i++) {
        int32_t steps = (float)(i) / (float)(LINEAR_AXIS_LUT_COUNT - 1) * (float)(accel_step_count);
        uint16_t step_time = (uint16_t)(LinearAxisMovement_calculate_lut_entry(m, steps));
        movement.lut[i] = step_time;
    }

    report_info_ln(
        "Calculated move: accel: %li steps, coast: %li steps, decel: %li steps.",
        movement.accel_step_count,
        movement.coast_step_count,
        movement.decel_step_count);

    return movement;
}

void LinearAxis_start_move(struct LinearAxis* m, struct LinearAxisMovement move) {
    m->stepper->direction = move.direction;
    Stepper_update_direction(m->stepper);

    if (m->stepper2 != NULL) {
        m->stepper2->direction = move.direction;
        Stepper_update_direction(m->stepper2);
    }

    m->_current_move = move;
    m->_step_interval = 100;
    m->_next_step_at = make_timeout_time_us(m->_step_interval);

    // Calculate the *actual* distance that the motor will move based on the
    // stepping resolution.
    float actual_delta_mm = move.direction * (float)(move.total_step_count) * (1.0f / m->steps_per_mm);
    report_info_ln(
        "moving %c axis %0.3f mm (%li steps)",
        m->name,
        (double)actual_delta_mm,
        move.direction * move.total_step_count);
}

void LinearAxis_wait_for_move(struct LinearAxis* m) {
    if (!LinearAxis_is_moving(m)) {
        return;
    }

    // absolute_time_t report_time = make_timeout_time_ms(1000);

    while (LinearAxis_is_moving(m)) {
        LinearAxis_timed_step(m);

        // Note: disabled because it actually causes a slight jerk in the motor movement due
        // to the amount of time it takes to send this over USB.
        // if (absolute_time_diff_us(get_absolute_time(), report_time) <= 0) {
        //     report_info_ln("moved %li/%li steps", m->_current_move.steps_taken, m->_current_move.total_step_count);
        //     report_time = make_timeout_time_ms(1000);
        // }
    }

    report_info_ln(
        "%c axis moved to %0.3f (%li steps)", m->name, (double)LinearAxis_get_position_mm(m), m->stepper->total_steps);
}

float LinearAxis_get_position_mm(struct LinearAxis* m) {
    return (float)(m->stepper->total_steps) * (1.0f / m->steps_per_mm);
}

void LinearAxis_set_position_mm(struct LinearAxis* m, float mm) {
    m->stepper->total_steps = (int32_t)(lroundf(ceilf(mm * m->steps_per_mm)));
}

/*
    Private methods
*/

void __not_in_flash_func(LinearAxis_direct_step)(struct LinearAxis* m) {
    // Are there any steps to perform?
    if (m->_current_move.total_step_count == 0) {
        return;
    }

    if (m->stepper2 != NULL) {
        Stepper_step_two(m->stepper, m->stepper2);
    } else {
        Stepper_step(m->stepper);
    }

    m->_current_move.steps_taken++;

    // Is the move finished?
    if (m->_current_move.steps_taken == m->_current_move.total_step_count) {
        m->_current_move = (struct LinearAxisMovement){};
    }
}

bool __not_in_flash_func(LinearAxis_timed_step)(struct LinearAxis* m) {
    // Is it time to step yet?
    if (absolute_time_diff_us(get_absolute_time(), m->_next_step_at) > 0) {
        return false;
    }

    LinearAxis_direct_step(m);
    LinearAxis_lookup_step_interval(m);
    m->_next_step_at = make_timeout_time_us(m->_step_interval);

    return true;
}

__attribute__((optimize(3))) void __not_in_flash_func(LinearAxis_lookup_step_interval)(struct LinearAxis* m) {
    size_t lut_index;

    // Acceleration phase
    if (m->_current_move.steps_taken <= m->_current_move.accel_step_count) {
        lut_index = m->_current_move.steps_taken * (LINEAR_AXIS_LUT_COUNT - 1) / m->_current_move.accel_step_count;
    }
    // Coast phase
    else if (m->_current_move.steps_taken <= m->_current_move.accel_step_count + m->_current_move.coast_step_count) {
        lut_index = (LINEAR_AXIS_LUT_COUNT - 1);
    }
    // Deceleration phase
    else {
        int32_t steps =
            m->_current_move.steps_taken - (m->_current_move.accel_step_count + m->_current_move.coast_step_count);
        lut_index =
            (LINEAR_AXIS_LUT_COUNT - 1) - (steps * (LINEAR_AXIS_LUT_COUNT - 1) / m->_current_move.accel_step_count);
    }

    int64_t step_time_us = m->_current_move.lut[MIN(lut_index, LINEAR_AXIS_LUT_COUNT - 1)];
    step_time_us = MIN(step_time_us, 5000);

    m->_step_interval = step_time_us;
}

__attribute__((optimize(3))) uint32_t
__not_in_flash_func(LinearAxisMovement_calculate_lut_entry)(struct LinearAxis* a, uint32_t steps) {
    // Calculate instantenous velocity at the current distance traveled.

    // At 0 steps velocity is technically zero, so just cheat and pretend we're
    // 1 step in.
    if (steps == 0) {
        steps = 1;
    }

    // distance mm = steps * 1 / steps/mm
    float distance = steps / a->steps_per_mm;
    float inst_velocity = sqrtf(2.0f * distance * a->acceleration_mm_s2);

    // Calculate the timer period from the velocity
    float s_per_step;
    if (inst_velocity > 0.0f) {
        float steps_per_s = inst_velocity / (1.0f / a->steps_per_mm);
        s_per_step = 1.0f / steps_per_s;
    } else {
        s_per_step = 0.005f;
    }

    uint64_t step_time_us = (uint64_t)(s_per_step * 1000000.0f);
    step_time_us = MIN(step_time_us, 5000);

    return (uint32_t)(step_time_us);
}
