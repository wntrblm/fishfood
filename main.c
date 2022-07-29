#include "config/general.h"
#include "config/pins.h"
#include "drivers/tmc2209.h"
#include "drivers/tmc2209_helper.h"
#include "drivers/tmc_uart.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "littleg/littleg.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

static struct TMC2209 tmc_left;
static struct TMC2209 tmc_right;
static struct TMC2209 tmc_z;

static repeating_timer_t step_timer;
static repeating_timer_t tmc_timer;

struct MotorState {
    volatile int32_t actual_steps;
    volatile int32_t delta_steps;
    volatile float actual_mm;
    volatile int32_t max_steps;
    volatile int32_t homing_state;
    volatile bool crash_flag;
    volatile bool _step_edge;
};

static volatile struct MotorState z_state;

inline static void set_z_dir(int32_t delta) { gpio_put(PIN_M0_DIR, delta >= 0 ? 1 : 0); }

bool step_timer_callback(repeating_timer_t* rt) {
    uint32_t irq_status = save_and_disable_interrupts();

    if (z_state.homing_state == 1) {
        set_z_dir(Z_HOMING_DIR);
        gpio_put(PIN_M0_STEP, z_state._step_edge);
        z_state._step_edge = !z_state._step_edge;
        restore_interrupts(irq_status);
        return true;
    }

    if (z_state.delta_steps == 0) {
        restore_interrupts(irq_status);
        return true;
    }

    set_z_dir(z_state.delta_steps);
    gpio_put(PIN_M0_STEP, z_state._step_edge);
    z_state._step_edge = !z_state._step_edge;

    if (z_state._step_edge == false) {
        if (z_state.delta_steps > 0) {
            z_state.delta_steps--;
            z_state.actual_steps++;
            z_state.actual_mm += Z_MM_PER_STEP;
        } else {
            z_state.delta_steps++;
            z_state.actual_steps--;
            z_state.actual_mm -= Z_MM_PER_STEP;
        }
    }

    restore_interrupts(irq_status);
    return true;
}

void diag_rise_callback(uint gpio, uint32_t events) {
    uint32_t irq_status = save_and_disable_interrupts();
    if(z_state.homing_state == 1) {
        z_state.actual_mm = 0.0f;
        z_state.actual_steps = 0;
        z_state.delta_steps = Z_INITIAL_MAX_STEPS;
        z_state.homing_state = 2;
    } else if(z_state.homing_state == 2) {
        z_state.max_steps = z_state.actual_steps;
        z_state.homing_state = 0;
        z_state.delta_steps = -(z_state.actual_steps / 2);
    } else {
        z_state.delta_steps = 0;
        z_state.crash_flag = true;
    }
    restore_interrupts(irq_status);
}

int main() {
    stdio_init_all();

    gpio_init(PIN_M0_DIR);
    gpio_set_dir(PIN_M0_DIR, GPIO_OUT);
    gpio_init(PIN_M0_STEP);
    gpio_set_dir(PIN_M0_STEP, GPIO_OUT);
    gpio_init(PIN_M0_EN);
    gpio_set_dir(PIN_M0_EN, GPIO_OUT);
    gpio_init(PIN_M0_DIAG);
    gpio_set_dir(PIN_M0_DIAG, GPIO_IN);

    gpio_put(PIN_M0_DIR, 1);
    gpio_put(PIN_M0_STEP, 0);
    gpio_put(PIN_M0_EN, 1);

    // Wait for USB connection.
    while (!stdio_usb_connected()) {}

    printf("Starting UART...\n");
    uart_init(uart1, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    printf("Starting TMC drivers...\n");
    TMC2209_init(&tmc_left, uart1, 0, tmc_uart_read_write);
    if (!TMC2209_write_config(&tmc_left, PIN_M0_EN)) {
        printf("Error configuring tmc_left!");
        return 0;
    }

    printf("Configuring DIAG interrupt...\n");
    gpio_set_irq_enabled_with_callback(PIN_M0_DIAG, GPIO_IRQ_EDGE_RISE, true, &diag_rise_callback);

    printf("Starting steppers...\n");
    add_repeating_timer_us(-100, step_timer_callback, NULL, &step_timer);

    struct lilg_Command command = {};

    while (1) {
        int in_c = getchar();
        if (in_c == EOF) {
            break;
        }

        // Echo
        // TODO: Remove this
        putchar(in_c);
        if (in_c == '\r') {
            putchar('\n');
        }

        bool valid_command = lilg_parse(&command, (char)(in_c));

        if (valid_command) {
            if (command.G.set && command.G.real == 0) {
                float dest_mm = (float)(command.Z.real);
                float delta_mm = dest_mm - z_state.actual_mm;
                float delta_steps = delta_mm * (float)(Z_STEPS_PER_MM);
                z_state.delta_steps = (int32_t)(roundf(delta_steps));
                printf("> Moving Z %0.2f mm (%i steps)\n", delta_mm, z_state.delta_steps);
            }
            if (command.G.set && command.G.real == 28) {
                z_state.homing_state = 1;
                while(z_state.homing_state == 1) {}
                printf("Z min set\n");
                while(z_state.homing_state == 2) {}
                printf("Z max set %0.2f mm, %i steps\n", z_state.actual_mm, z_state.max_steps);
            }
            if (command.M.set) {
                printf("> Z: %0.2f mm, (%i steps), crashed? %u\n", z_state.actual_mm, z_state.actual_steps, z_state.crash_flag);
                z_state.crash_flag = false;
            }
            printf("ok\n");
        }
    }

    printf("Main loop exited due to end of file on stdin\n");
}
