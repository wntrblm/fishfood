#include "linear_axis.h"
#include "config/motion.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <math.h>
#include <stdio.h>

/*
    Macros and constants
*/
enum HomingState {
    HOMING_NONE = 0,
    HOMING_SEEKING = 1,
    HOMING_BOUNCING = 2,
    HOMING_RESEEKING = 3,
    HOMING_FINISHED = 4
};

/*
    Static variables
*/
volatile struct LinearAxis* current_motor;

/*
    Forward declarations
*/
static void setup_move(volatile struct LinearAxis* m, float dest_mm);
static bool step_timer_callback(repeating_timer_t* rt);
static void diag_pin_irq();
static void debug_stallguard(volatile struct LinearAxis* m);

/*
    Public methods
*/

void LinearAxis_init(
    struct LinearAxis* m,
    char name,
    struct TMC2209* tmc,
    uint32_t pin_enn,
    uint32_t pin_dir,
    uint32_t pin_step,
    uint32_t pin_diag) {
    m->name = name;

    m->tmc = tmc;
    m->pin_enn = pin_enn;
    m->pin_dir = pin_dir;
    m->pin_step = pin_step;
    m->pin_diag = pin_diag;

    m->actual_steps = 0;

    m->_step_edge = 0;
    m->_dir = 0;

    m->_accel_step_count = 0;
    m->_decel_step_count = 0;
    m->_coast_step_count = 0;
    m->_total_step_count = 0;
    m->_current_step_count = 0;

    m->_crash_flag = 0;

    m->velocity_mm_s = 100.0f;
    m->acceleration_mm_s2 = 1000.0f;
    m->homing_sensitivity = 100;
}

bool LinearAxis_setup(struct LinearAxis* m) {
    gpio_init(m->pin_enn);
    gpio_set_dir(m->pin_enn, GPIO_OUT);
    gpio_put(m->pin_enn, true);

    gpio_init(m->pin_dir);
    gpio_set_dir(m->pin_dir, GPIO_OUT);
    gpio_put(m->pin_dir, false);

    gpio_init(m->pin_step);
    gpio_set_dir(m->pin_step, GPIO_OUT);
    gpio_put(m->pin_step, false);

    gpio_init(m->pin_diag);
    gpio_set_dir(m->pin_diag, GPIO_IN);
    gpio_pull_down(m->pin_diag);

    if (!TMC2209_write_config(m->tmc, m->pin_enn)) {
        printf("Error configuring TMC2209 for %c axis!", m->name);
        return false;
    }

    return true;
}

void LinearAxis_home(volatile struct LinearAxis* m) {
    printf("> Homing %c axis...\n", m->name);
    printf("> Stallguard is %u\n", m->homing_sensitivity);

    float old_velocity = m->velocity_mm_s;
    float old_acceleration = m->acceleration_mm_s2;
    m->velocity_mm_s = m->homing_velocity_mm_s;
    m->acceleration_mm_s2 = m->homing_acceleration_mm_s2;

    printf("> Seeking endstop...\n");

    current_motor = m;
    m->actual_steps = 0;

    // TODO: At some point this needs to use a more sophisticated GPIO IRQ,
    // since the built-in picosdk only provides *one* callback for *all* gpio
    // events.
    gpio_set_irq_enabled_with_callback(m->pin_diag, GPIO_IRQ_EDGE_RISE, true, &diag_pin_irq);
    setup_move(m, m->homing_direction * m->homing_distance_mm);

    // Ignore stallguard output until it's up to speed
    while (m->_current_step_count < m->_accel_step_count) {
        tight_loop_contents();
    }

    TMC2209_write(m->tmc, TMC2209_SGTHRS, m->homing_sensitivity);
    m->_crash_flag = false;

    while (!m->_crash_flag) {
        tight_loop_contents();
        // sleep_ms(50);
        // debug_stallguard(m);
    }

    LinearAxis_stop(m);

    printf("> Endstop found, bouncing...\n");
    TMC2209_write(m->tmc, TMC2209_SGTHRS, 0);
    m->_crash_flag = false;
    LinearAxis_reset_position(m);
    setup_move(current_motor, -(m->homing_direction * m->homing_bounce_mm));

    while (LinearAxis_is_moving(m)) { tight_loop_contents(); }

    printf("> Re-seeking...\n");
    setup_move(m, m->homing_direction * m->homing_bounce_mm * 2);

    while (m->_current_step_count < m->_accel_step_count) {
        tight_loop_contents();
    }

    TMC2209_write(m->tmc, TMC2209_SGTHRS, m->homing_sensitivity);
    m->_crash_flag = false;

    while (!m->_crash_flag) {
        tight_loop_contents();
        // sleep_ms(50);
        // debug_stallguard(m);
    }

    LinearAxis_stop(m);

    printf("> Found! Saving home position...\n");
    LinearAxis_reset_position(m);

    printf("> Disabling stallguard...\n");
    TMC2209_write(m->tmc, TMC2209_SGTHRS, 0);

    m->velocity_mm_s = old_velocity;
    m->acceleration_mm_s2 = old_acceleration;
    printf("> %c axis homing complete!\n", m->name);
}

void LinearAxis_start_move(volatile struct LinearAxis* m, float dest_mm) { setup_move(m, dest_mm); }

void LinearAxis_wait_for_move(volatile struct LinearAxis* m) {
    absolute_time_t report_time = make_timeout_time_ms(1000);
    while (LinearAxis_is_moving(m)) {
        if(absolute_time_diff_us(get_absolute_time(), report_time) <= 0) {
            printf("> Still moving, report_at=%lld, step interval=%lld next step at=%lld, steps taken=%d\n", report_time, m->_step_interval, m->_next_step_at, m->_current_step_count);
            report_time = make_timeout_time_ms(1000);
        }
        tight_loop_contents();
    }

    printf("> %c axis moved to %0.3f (%i steps).\n", m->name, LinearAxis_get_position_mm(m), m->actual_steps);
}

float LinearAxis_get_position_mm(volatile struct LinearAxis* m) {
    return (float)(m->actual_steps) * (1.0f / m->steps_per_mm);
}

/*
    Private methods
*/

static void setup_move(volatile struct LinearAxis* m, float dest_mm) {
    // Calculate how far to move to bring the motor to the destination.
    float delta_mm = dest_mm - LinearAxis_get_position_mm(m);
    int32_t dir = delta_mm < 0 ? -1 : 1;

    // Determine the number of steps needed to complete the move.
    float delta_mm_abs = fabs(delta_mm);
    int32_t total_step_count = (int32_t)(lroundf(delta_mm_abs * m->steps_per_mm));

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

    // Calculate the *actual* distance that the motor will move based on the
    // stepping resolution.
    float actual_delta_mm = dir * total_step_count * (1.0f / m->steps_per_mm);
    printf("> Moving %c axis %0.3f mm (%i steps)\n", m->name, actual_delta_mm, dir * total_step_count);
    printf("> Velocity: %0.2f mm/2, acceleration: %0.2f mm/2^2\n", m->velocity_mm_s, m->acceleration_mm_s2);
    printf("> Steps per phase: %ld, %ld, %ld\n", accel_step_count, coast_step_count, decel_step_count);

    // Update motor parameters and kickoff step timer. Disable interrupts to prevent
    // any wackiness.
    uint32_t irq_status = save_and_disable_interrupts();
    m->_accel_step_count = accel_step_count;
    m->_decel_step_count = decel_step_count;
    m->_coast_step_count = coast_step_count;
    m->_dir = dir;
    m->_current_step_count = 0;
    m->_total_step_count = total_step_count;
    m->_step_interval = 1000;
    m->_next_step_at = make_timeout_time_us(m->_step_interval);
    restore_interrupts(irq_status);
}

static void diag_pin_irq(uint32_t pin, uint32_t events) {
    if (current_motor == NULL) {
        return;
    }
    current_motor->_crash_flag = true;
}

void LinearAxis_step(volatile struct LinearAxis* m) {
    // Are there any steps to perform?
    if (m->_total_step_count == 0) {
        return;
    }

    // Is it time to step yet?
    if (absolute_time_diff_us(get_absolute_time(), m->_next_step_at) > 0) {
        return;
    }

    gpio_put(m->pin_dir, m->_dir == 1 ? 0 : 1);
    gpio_put(m->pin_step, m->_step_edge);
    m->_step_edge = !m->_step_edge;

    // Steps happen on rising edges, so when the _step_edge is reset to false
    // that means a rising edge was just sent.
    if (m->_step_edge == false) {
        m->_current_step_count++;
        m->actual_steps += m->_dir;

        // Is the move finished?
        if (m->_current_step_count == m->_total_step_count) {
            m->_current_step_count = 0;
            m->_total_step_count = 0;
            return;
        }

        // Calculate instantenous velocity at the current
        // distance traveled.
        float distance = m->_current_step_count * (1.0f / m->steps_per_mm);
        float inst_velocity;

        // Acceleration phase
        if (m->_current_step_count < m->_accel_step_count) {
            inst_velocity = sqrtf(2.0f * distance * m->acceleration_mm_s2);
        }
        // Coast phase
        else if (m->_current_step_count < m->_accel_step_count + m->_coast_step_count) {
            inst_velocity = m->velocity_mm_s;
        }
        // Deceleration phase
        else {
            float total_distance = m->_total_step_count * (1.0f / m->steps_per_mm);
            inst_velocity = sqrtf(2.0f * (total_distance - distance) * m->acceleration_mm_s2);
        }

        // Calculate the timer period from the velocity
        float s_per_step;
        if (inst_velocity > 0.0f) {
            float steps_per_s = inst_velocity / (1.0f / m->steps_per_mm);
            s_per_step = 1.0f / steps_per_s;
        } else {
            s_per_step = 0.001f;
        }

        int64_t step_time_us = (int64_t)(s_per_step * 1000000.0f);
        step_time_us = step_time_us > 5000 ? 5000 : step_time_us;
        m->_step_interval = step_time_us;
    }

    // The step interval is halved because it takes *two* calls to this method
    // to output a single step- the first call pulls the STEP pin low and the
    // second pulls it high. Steps occur only on the rising edge of the STEP
    // pin.
    m->_next_step_at = make_timeout_time_us(m->_step_interval / 2);
}

static void debug_stallguard(volatile struct LinearAxis* m) {
    uint32_t sg_result;
    TMC2209_read(m->tmc, TMC2209_SG_RESULT, &sg_result);
    printf("> SG: %u\n", sg_result);
}
