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
#include "z_axis.h"
#include "rotational_axis.h"
#include <math.h>
#include <stdio.h>

static struct TMC2209 tmc_left;
static struct TMC2209 tmc_right;
static struct TMC2209 tmc_z;

static struct ZMotor z_motor;
static struct RotationalAxis l_motor;

int main() {
    stdio_init_all();

    TMC2209_init(&tmc_left, uart1, 0, tmc_uart_read_write);
    TMC2209_init(&tmc_z, uart1, 1, tmc_uart_read_write);

    RotationalAxis_init(&l_motor, &tmc_left, PIN_M0_EN, PIN_M0_DIR, PIN_M0_STEP);
    ZMotor_init(&z_motor, &tmc_z, PIN_M1_EN, PIN_M1_DIR, PIN_M1_STEP, PIN_M1_DIAG);

    // Wait for USB connection.
    while (!stdio_usb_connected()) {}

    printf("Starting UART...\n");
    uart_init(uart1, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    printf("Starting motors...\n");

    ZMotor_setup(&z_motor);
    RotationalAxis_setup(&l_motor);

    printf("Ready!\n");

    struct lilg_Command command = {};

    while (1) {
        int in_c = getchar();
        if (in_c == EOF) {
            break;
        }

        // Echo TODO: Remove this
        putchar(in_c);
        if (in_c == '\r') {
            putchar('\n');
        }

        bool valid_command = lilg_parse(&command, (char)(in_c));

        if (valid_command) {
            if (command.G.set && command.G.real == 0) {
                if(command.Z.set) {
                    float dest_mm = (float)(command.Z.real);
                    ZMotor_move_to(&z_motor, dest_mm);
                }
                if(command.fields[0].set) {
                    float dest_deg = (float)(command.fields[0].real);
                    RotationalAxis_move_to(&l_motor, dest_deg);
                }
            }
            if (command.G.set && command.G.real == 28) {
                ZMotor_home(&z_motor);
            }
            if (command.M.set) {
                printf("> Z: %0.2f mm, (%i steps), crashed? %u\n", z_motor.actual_mm, z_motor.actual_steps, z_motor._crash_flag);
                printf("> A: %0.2f deg, (%i steps)\n", l_motor.actual_deg, l_motor.actual_steps);
            }
            printf("ok\n");
        }
    }

    printf("Main loop exited due to end of file on stdin\n");
}
