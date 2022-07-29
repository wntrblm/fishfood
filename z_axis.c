#include "z_axis.h"
#include "config/general.h"
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
bool step_timer_callback(repeating_timer_t* rt);
void diag_pin_irq();

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
    gpio_set_dir(m->pin_enn, GPIO_OUT);
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
    gpio_add_raw_irq_handler(m->pin_diag, &diag_pin_irq);

    printf("Starting steppers...\n");
    add_repeating_timer_us(-150, step_timer_callback, NULL, &(m->_step_timer));

    return true;
}

void ZMotor_home(volatile struct ZMotor* m) {
    m->_homing_state = 1;
    printf("Homing Z... ");
    while (m->_homing_state == 1) {}
    printf("Z min set");
    while (m->_homing_state == 2) {}
    printf(", Z max set %0.2f mm, %i steps\n", m->actual_mm, m->max_steps);
}

void ZMotor_move_to(volatile struct ZMotor* m, float dest_mm) {
    float delta_mm = dest_mm - m->actual_mm;
    float delta_steps = delta_mm * (float)(Z_STEPS_PER_MM);
    m->_delta_steps = (int32_t)(roundf(delta_steps));
    printf("> Moving Z %0.2f mm (%i steps)\n", delta_mm, m->_delta_steps);
}

/*
    Private methods
*/

void diag_pin_irq() {
    if (!(gpio_get_irq_event_mask(current_motor->pin_diag) & GPIO_IRQ_EDGE_RISE)) {
        return;
    }
    gpio_acknowledge_irq(current_motor->pin_diag, GPIO_IRQ_EDGE_RISE);

    uint32_t irq_status = save_and_disable_interrupts();
    if (current_motor->_homing_state == 1) {
        current_motor->actual_mm = 0.0f;
        current_motor->actual_steps = 0;
        current_motor->_delta_steps = Z_INITIAL_MAX_STEPS;
        current_motor->_homing_state = 2;
    } else if (current_motor->_homing_state == 2) {
        current_motor->max_steps = current_motor->actual_steps;
        current_motor->_homing_state = 0;
        current_motor->_delta_steps = -(current_motor->actual_steps / 2);
    } else {
        current_motor->_delta_steps = 0;
        current_motor->_crash_flag = true;
    }
    restore_interrupts(irq_status);
}

bool step_timer_callback(repeating_timer_t* rt) {
    uint32_t irq_status = save_and_disable_interrupts();

    if (current_motor->_homing_state == 1) {
        gpio_put(current_motor->pin_dir, Z_HOMING_DIR >= 0 ? 1 : 0);
        gpio_put(current_motor->pin_step, current_motor->_step_edge);
        current_motor->_step_edge = !current_motor->_step_edge;
        restore_interrupts(irq_status);
        return true;
    }

    if (current_motor->_delta_steps == 0) {
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
