#include "z_axis.h"
#include "config/general.h"
#include "config/tmc.h"
#include "drivers/tmc2209_helper.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <math.h>
#include <stdio.h>

/*
    Macros and constants
*/

/*
    Static variables
*/
volatile struct ZMotor* current_motor;

/*
    Forward declarations
*/
static bool step_timer_callback(repeating_timer_t* rt);
static void diag_pin_irq();

/*
    Public methods
*/

void ZMotor_init(
    struct ZMotor* m, struct TMC2209* tmc, uint32_t pin_enn, uint32_t pin_dir, uint32_t pin_step, uint32_t pin_diag) {
    m->tmc = tmc;
    m->pin_enn = pin_enn;
    m->pin_dir = pin_dir;
    m->pin_step = pin_step;
    m->pin_diag = pin_diag;
    m->actual_steps = 0;
    m->actual_mm = 0.0f;
    m->max_steps = 0;
    m->_delta_steps = 0;
    m->_homing_state = 0;
    m->_crash_flag = 0;
    m->_step_edge = 0;
}

bool ZMotor_setup(struct ZMotor* m) {
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
        printf("Error configuring Z motor TMC2209!");
        return false;
    }

    current_motor = m;

    printf("Configuring DIAG interrupt...\n");
    gpio_set_irq_enabled_with_callback(m->pin_diag, GPIO_IRQ_EDGE_RISE, true, &diag_pin_irq);

    printf("Starting stepper timer...\n");
    add_repeating_timer_us(1000, step_timer_callback, NULL, &(m->_step_timer));

    ZMotor_set_velocity(m, 0.0f);

    return true;
}

void ZMotor_home(volatile struct ZMotor* m) {
    printf("Homing Z... ");
    //TMC2209_write(m->tmc, TMC2209_SGTHRS, CONFIG_TMC_HOMING_STALL_THRESHOLD);
    m->_homing_state = 1;
    while (m->_homing_state == 1) {
        sleep_ms(50);
        uint32_t sg_result;
        TMC2209_read(m->tmc, TMC2209_SG_RESULT, &sg_result);
        printf("> SG: %u\n", sg_result);
    }
    printf("found stop, bouncing...");
    while (m->_homing_state != 0) {
        sleep_ms(50);
        uint32_t sg_result;
        TMC2209_read(m->tmc, TMC2209_SG_RESULT, &sg_result);
        printf("> SG: %u\n", sg_result);
    }
    printf("homed!\n", m->actual_mm, m->max_steps);
}

void ZMotor_move_to(volatile struct ZMotor* m, float dest_mm) {
    float delta_mm = dest_mm - m->actual_mm;
    float delta_steps = delta_mm * (float)(Z_STEPS_PER_MM);
    m->_delta_steps = (int32_t)(roundf(delta_steps));
    printf("> Moving Z %0.2f mm (%i steps)\n", delta_mm, m->_delta_steps);
    while(m->_delta_steps != 0) {
        sleep_ms(50);
        uint32_t sg_result;
        TMC2209_read(m->tmc, TMC2209_SG_RESULT, &sg_result);
        printf("> SG: %u\n", sg_result);
    }
    printf("> Move finished at %0.2f.\n", m->actual_mm);
}


void ZMotor_set_step_interval(volatile struct ZMotor* m, int64_t step_us) {
    m->_step_timer.delay_us = step_us;
}

void ZMotor_set_velocity(volatile struct ZMotor* m, float v_mm_s) {
    if(v_mm_s == 0.0f) {
        v_mm_s = Z_DEFAULT_VELOCITY_MM_PER_M  / 60.0f;
    }
    float steps_per_s = (v_mm_s / Z_MM_PER_STEP);
    float s_per_step = 1.0f / steps_per_s;
    int64_t us_per_step = (int64_t)(1000000.0f * s_per_step);
    printf("> steps/s: %0.4f, s/step: %0.6f, us/s: %li\n", steps_per_s, s_per_step, us_per_step);
    if(us_per_step < 50) {
        printf("> Can not set a velocity that would result in a step time of less than 50 microseconds.\n");
        us_per_step = 50;
    }
    ZMotor_set_step_interval(m, us_per_step);
}

/*
    Private methods
*/

static void diag_pin_irq(uint32_t pin, uint32_t events) {
    uint32_t irq_status = save_and_disable_interrupts();

    if (current_motor->_homing_state == 1) {
        current_motor->_delta_steps = 10 * Z_STEPS_PER_MM;
        current_motor->_homing_state = 2;
    } else if (current_motor->_homing_state == 3) {
        current_motor->actual_mm = 0.0f;
        current_motor->actual_steps = 0;
        current_motor->_delta_steps = 0;
        current_motor->_homing_state = 0;
    } else {
        current_motor->_delta_steps = 0;
        current_motor->_crash_flag = true;
    }

    restore_interrupts(irq_status);
}

static bool step_timer_callback(repeating_timer_t* rt) {
    uint32_t irq_status = save_and_disable_interrupts();

    if (current_motor->_homing_state == 1 || current_motor->_homing_state == 3) {
        gpio_put(current_motor->pin_dir, Z_HOMING_DIR >= 0 ? 1 : 0);
        gpio_put(current_motor->pin_step, current_motor->_step_edge);
        current_motor->_step_edge = !current_motor->_step_edge;
        restore_interrupts(irq_status);
        return true;
    }

    if (current_motor->_delta_steps == 0) {
        if(current_motor->_homing_state == 2) {
            current_motor->_homing_state = 3;
        }
        restore_interrupts(irq_status);
        return true;
    }

    gpio_put(current_motor->pin_dir, current_motor->_delta_steps >= 0 ? 1 : 0);
    gpio_put(current_motor->pin_step, current_motor->_step_edge);
    current_motor->_step_edge = !current_motor->_step_edge;

    if (current_motor->_step_edge == false) {
        if (current_motor->_delta_steps > 0) {
            current_motor->_delta_steps--;
            current_motor->actual_steps++;
            current_motor->actual_mm += Z_MM_PER_STEP;
        } else {
            current_motor->_delta_steps++;
            current_motor->actual_steps--;
            current_motor->actual_mm -= Z_MM_PER_STEP;
        }
    }

    restore_interrupts(irq_status);
    return true;
}
