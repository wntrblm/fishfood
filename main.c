#include "config/motion.h"
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
#include "pico/bootrom.h"
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

static bool absolute_positioning = true;

int main() {
    stdio_init_all();

    gpio_init(PIN_ACT_LED);
    gpio_set_dir(PIN_ACT_LED, GPIO_OUT);
    gpio_put(PIN_ACT_LED, true);

    Neopixel_init(PIN_CAM_LED);
    Neopixel_set_all(pixels, NUM_PIXELS, 255, 0, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    TMC2209_init(&tmc_z, uart0, 1, tmc_uart_read_write);
    TMC2209_init(&tmc_left, uart0, 0, tmc_uart_read_write);
    TMC2209_init(&tmc_right, uart0, 3, tmc_uart_read_write);

    ZMotor_init(&z_motor, &tmc_z, PIN_M1_EN, PIN_M1_DIR, PIN_M1_STEP, PIN_M1_DIAG);
    RotationalAxis_init(&l_motor, &tmc_left, PIN_M0_EN, PIN_M0_DIR, PIN_M0_STEP);
    RotationalAxis_init(&r_motor, &tmc_right, PIN_M2_EN, PIN_M2_DIR, PIN_M2_STEP);

    Neopixel_set_all(pixels, NUM_PIXELS, 0, 255, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    // Wait for USB connection before continuing.
    while (!stdio_usb_connected()) {}

    printf("Starting UART...\n");
    uart_init(uart0, 115200);
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

        bool valid_command = lilg_parse(&command, (char)(in_c));

        if (valid_command) {
            if (command.first_field == 'G') {
                if (command.G.real == 0) {
                    if (LILG_FIELD(command, F).set) {
                        float mm_per_min = lilg_Decimal_to_float(LILG_FIELD(command, F));
                        z_motor.velocity_mm_s = mm_per_min / 60.0f;
                    }
                    if (command.Z.set) {
                        float dest_mm = lilg_Decimal_to_float(command.Z);
                        if (!absolute_positioning) {
                            dest_mm = ZMotor_get_position_mm(&z_motor) + dest_mm;
                        }
                        ZMotor_move_to(&z_motor, dest_mm);
                    }
                    if (LILG_FIELD(command, A).set) {
                        float dest_deg = lilg_Decimal_to_float(LILG_FIELD(command, A));
                        if (!absolute_positioning) {
                            dest_deg = l_motor.actual_deg + dest_deg;
                        }
                        RotationalAxis_move_to(&l_motor, dest_deg);
                    }
                    if (LILG_FIELD(command, B).set) {
                        float dest_deg = lilg_Decimal_to_float(LILG_FIELD(command, B));
                        if (!absolute_positioning) {
                            dest_deg = r_motor.actual_deg + dest_deg;
                        }
                        RotationalAxis_move_to(&r_motor, dest_deg);
                    }
                } else if (command.G.real == 28) {
                    // Home axes
                    // https://marlinfw.org/docs/gcode/G28.html
                    ZMotor_home(&z_motor);
                } else if (command.G.real == 90) {
                    // Absolute positioning
                    // https://marlinfw.org/docs/gcode/G090.html
                    absolute_positioning = true;
                } else if (command.G.real == 91) {
                    // Relative positioning
                    // https://marlinfw.org/docs/gcode/G091.html
                    absolute_positioning = false;
                } else {
                    printf("Unknown command G%i\n", command.G.real);
                }
            } else if (command.first_field == 'M') {
                if (command.M.real == 114) {
                    // M114 get current position
                    // https://marlinfw.org/docs/gcode/M114.html
                    printf(
                        "Z:%0.2f A:%0.2f B:%0.2f Count Z:%i A:%i B:%i\n",
                        ZMotor_get_position_mm(&z_motor),
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
                    printf(
                        "R:%i G:%i B: %i\n",
                        LILG_FIELD(command, R).real,
                        LILG_FIELD(command, G).real,
                        LILG_FIELD(command, B).real);
                } else if (command.M.real == 204) {
                    // M204 Set Starting Acceleration
                    // https://marlinfw.org/docs/gcode/M204.html
                    float accel = lilg_Decimal_to_float(LILG_FIELD(command, T));
                    z_motor.acceleration_mm_s2 = accel;
                    printf("> Set acceleration to %0.2f mm/s^2\n", accel);
                } else if (command.M.real == 914) {
                    // M914 Set bump sensitivity
                    // https://marlinfw.org/docs/gcode/M914.html
                    z_motor.homing_sensitivity = command.Z.real;
                    printf("> Set homing sensitivity to %u\n", command.Z.real);
                } else if (command.M.real == 997) {
                    // M997 firmware update
                    // https://marlinfw.org/docs/gcode/M997.html
                    reset_usb_boot(0, 0);
                } else {
                    printf("Unknown command M%i\n", command.M.real);
                }
            }
            printf("ok\n");
        }
    }

    printf("Main loop exited due to end of file on stdin\n");
}
