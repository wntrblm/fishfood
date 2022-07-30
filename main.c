#include "config/general.h"
#include "config/pins.h"
#include "drivers/neopixel.h"
#include "drivers/tmc2209.h"
#include "drivers/tmc2209_helper.h"
#include "drivers/tmc_uart.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "littleg/littleg.h"
#include "pico/stdlib.h"
#include "rotational_axis.h"
#include "z_axis.h"
#include <math.h>
#include <stdio.h>

#define NUM_PIXELS 8
static uint8_t pixels[3 * NUM_PIXELS];

static struct TMC2209 tmc_left;
static struct TMC2209 tmc_right;
static struct TMC2209 tmc_z;

static struct ZMotor z_motor;
static struct RotationalAxis l_motor;
static struct RotationalAxis r_motor;

int main() {
    stdio_init_all();

    Neopixel_init(PIN_CAM_LED);
    Neopixel_set_all(pixels, NUM_PIXELS, 255, 0, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    TMC2209_init(&tmc_left, uart1, 1, tmc_uart_read_write);
    TMC2209_init(&tmc_z, uart1, 0, tmc_uart_read_write);
    TMC2209_init(&tmc_right, uart1, 2, tmc_uart_read_write);

    RotationalAxis_init(&l_motor, &tmc_left, PIN_M1_EN, PIN_M1_DIR, PIN_M1_STEP);
    RotationalAxis_init(&r_motor, &tmc_left, PIN_M2_EN, PIN_M2_DIR, PIN_M2_STEP);
    ZMotor_init(&z_motor, &tmc_z, PIN_M0_EN, PIN_M0_DIR, PIN_M0_STEP, PIN_M0_DIAG);

    // Wait for USB connection.
    Neopixel_set_all(pixels, NUM_PIXELS, 0, 255, 0);
    Neopixel_write(pixels, NUM_PIXELS);
    while (!stdio_usb_connected()) {}

    printf("Starting UART...\n");
    uart_init(uart1, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    printf("Starting motors...\n");
    ZMotor_setup(&z_motor);
    RotationalAxis_setup(&l_motor);
    RotationalAxis_setup(&r_motor);

    printf("Ready!\n");
    Neopixel_set_all(pixels, NUM_PIXELS, 0, 0, 255);
    Neopixel_write(pixels, NUM_PIXELS);

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
                if (command.fields['F' - 'A'].set) {
                    ZMotor_set_step_interval(&z_motor, command.fields['F' - 'A'].real);
                    printf("> set stepping time to %u us\n", command.fields['F' - 'A'].real);
                }
                if (command.Z.set) {
                    float dest_mm = lilg_Decimal_to_float(command.Z);
                    ZMotor_move_to(&z_motor, dest_mm);
                }
                if (LILG_FIELD(command, A).set) {
                    float dest_deg = lilg_Decimal_to_float(LILG_FIELD(command, A));
                    RotationalAxis_move_to(&l_motor, dest_deg);
                }
            }
            if (command.G.set && command.G.real == 28) {
                ZMotor_home(&z_motor);
            }
            if (command.M.set) {

                if (command.M.real == 114) {
                    // M114 get current position
                    // https://marlinfw.org/docs/gcode/M114.html
                    printf(
                        "Z:%0.2f A:%0.2f B:%0.2f Count Z:%i A:%i B:%i\n",
                        z_motor.actual_mm,
                        l_motor.actual_deg,
                        r_motor.actual_deg,
                        z_motor.actual_steps,
                        l_motor.actual_steps,
                        r_motor.actual_steps);

                } else if (command.M.real == 150) {
                    // M150 set RGB
                    // https://marlinfw.org/docs/gcode/M150.html
                    Neopixel_set_all(
                        pixels,
                        NUM_PIXELS,
                        LILG_FIELD(command, R).real,
                        LILG_FIELD(command, G).real,
                        LILG_FIELD(command, B).real);
                    Neopixel_write(pixels, NUM_PIXELS);

                } else if (command.M.real == 914) {
                    // M914 Set bump sensitivity
                    // https://marlinfw.org/docs/gcode/M914.html
                    TMC2209_write(z_motor.tmc, TMC2209_SGTHRS, command.Z.real);
                    printf("> Set stallguard threshold to %u\n", command.Z.real);
                }
            }
            printf("ok\n");
        }
    }

    printf("Main loop exited due to end of file on stdin\n");
}
