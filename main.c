#include "config/motion.h"
#include "config/pins.h"
#include "drivers/neopixel.h"
#include "drivers/tmc2209.h"
#include "drivers/tmc2209_helper.h"
#include "drivers/tmc_uart.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "linear_axis.h"
#include "littleg/littleg.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "rotational_axis.h"
#include <math.h>
#include <stdio.h>

#define NUM_PIXELS 8
static uint8_t pixels[3 * NUM_PIXELS];

static struct TMC2209 tmc_left;
static struct TMC2209 tmc_right;
static struct TMC2209 tmc_z;

static struct LinearAxis z_motor;
static struct RotationalAxis l_motor;
static struct RotationalAxis r_motor;

static repeating_timer_t step_timer;

static bool absolute_positioning = true;

static bool step_timer_callback(repeating_timer_t* rt);
static void process_incoming_char(char c);
static void run_g_command(struct lilg_Command cmd);
static void run_m_command(struct lilg_Command cmd);

int main() {
    stdio_init_all();

    gpio_init(PIN_ACT_LED);
    gpio_set_dir(PIN_ACT_LED, GPIO_OUT);
    gpio_put(PIN_ACT_LED, true);

    gpio_init(PIN_AUX_PIN);
    gpio_set_dir(PIN_AUX_PIN, GPIO_OUT);
    gpio_put(PIN_AUX_PIN, false);

    Neopixel_init(PIN_CAM_LED);
    Neopixel_set_all(pixels, NUM_PIXELS, 255, 0, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    TMC2209_init(&tmc_z, uart0, 1, tmc_uart_read_write);
    TMC2209_init(&tmc_left, uart0, 0, tmc_uart_read_write);
    TMC2209_init(&tmc_right, uart0, 3, tmc_uart_read_write);

    LinearAxis_init(&z_motor, 'Z', &tmc_z, PIN_M1_EN, PIN_M1_DIR, PIN_M1_STEP, PIN_M1_DIAG);
    z_motor.steps_per_mm = Z_STEPS_PER_MM;
    z_motor.velocity_mm_s = Z_DEFAULT_VELOCITY_MM_S;
    z_motor.acceleration_mm_s2 = Z_DEFAULT_ACCELERATION_MM_S2;
    z_motor.homing_direction = Z_HOMING_DIR;
    z_motor.homing_distance_mm = Z_HOMING_DISTANCE_MM;
    z_motor.homing_bounce_mm = Z_HOMING_BOUNCE_MM;
    z_motor.homing_velocity_mm_s = Z_HOMING_VELOCITY_MM_S;
    z_motor.homing_acceleration_mm_s2 = Z_HOMING_ACCELERATION_MM_S2;
    z_motor.homing_sensitivity = Z_HOMING_SENSITIVITY;

    RotationalAxis_init(&l_motor, 'A', &tmc_left, PIN_M0_EN, PIN_M0_DIR, PIN_M0_STEP);
    l_motor.steps_per_deg = A_STEPS_PER_DEG;

    RotationalAxis_init(&r_motor, 'B', &tmc_right, PIN_M2_EN, PIN_M2_DIR, PIN_M2_STEP);
    r_motor.steps_per_deg = B_STEPS_PER_DEG;

    Neopixel_set_all(pixels, NUM_PIXELS, 0, 255, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    // Wait for USB connection before continuing.
    while (!stdio_usb_connected()) {}

    printf("Starting UART...\n");
    uart_init(uart0, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    printf("Starting motors...\n");
    LinearAxis_setup(&z_motor);
    RotationalAxis_setup(&l_motor);
    RotationalAxis_setup(&r_motor);

    printf("Setting motor current...\n");
    TMC2209_set_current(&tmc_z, Z_RUN_CURRENT, Z_RUN_CURRENT * Z_HOLD_CURRENT_MULTIPLIER);
    TMC2209_set_current(&tmc_left, A_RUN_CURRENT, A_RUN_CURRENT * A_HOLD_CURRENT_MULTIPLIER);
    TMC2209_set_current(&tmc_right, B_RUN_CURRENT, B_RUN_CURRENT * B_HOLD_CURRENT_MULTIPLIER);

    printf("Starting step timer...\n");
    uint32_t irq_status = save_and_disable_interrupts();
    // 25us is 100kHz, fast enough to achieve speeds up to 400mm/s.
    add_repeating_timer_us(25, step_timer_callback, NULL, &step_timer);
    restore_interrupts(irq_status);

    printf("Ready!\n");
    Neopixel_set_all(pixels, NUM_PIXELS, 0, 0, 255);
    Neopixel_write(pixels, NUM_PIXELS);

    while (1) {
        int in_c = getchar();
        if (in_c == EOF) {
            break;
        }

        process_incoming_char((char)(in_c));
    }

    printf("Main loop exited due to end of file on stdin\n");
}

static bool step_timer_callback(repeating_timer_t* rt) {
    LinearAxis_step(&z_motor);
    RotationalAxis_step(&l_motor);
    RotationalAxis_step(&r_motor);
    return true;
}

static void process_incoming_char(char c) {
    static struct lilg_Command cmd = {};

    enum lilg_ParseResult result = lilg_parse(&cmd, c);

    if (result == LILG_INCOMPLETE) {
        return;
    }

    if (result == LILG_INVALID) {
        printf("Invalid command\n");
        printf("ok\n");
        return;
    }

    switch (cmd.first_field) {
        case 'G': {
            run_g_command(cmd);
        } break;

        case 'M': {
            run_m_command(cmd);
        } break;

        default: {
            printf("Unexpected command %c%i\n", cmd.first_field, LILG_FIELDC(cmd, cmd.first_field));
        } break;
    }

    printf("ok\n");
}

static void run_g_command(struct lilg_Command cmd) {
    switch (cmd.G.real) {
        // Linear move
        // https://marlinfw.org/docs/gcode/G000-G001.html
        case 0: {
            // F is "feed rate", or velocity in mm/min
            if (LILG_FIELD(cmd, F).set) {
                float mm_per_min = lilg_Decimal_to_float(LILG_FIELD(cmd, F));
                z_motor.velocity_mm_s = mm_per_min / 60.0f;
            }

            if (cmd.Z.set) {
                float dest_mm = lilg_Decimal_to_float(cmd.Z);
                if (!absolute_positioning) {
                    dest_mm = LinearAxis_get_position_mm(&z_motor) + dest_mm;
                }
                LinearAxis_start_move(&z_motor, dest_mm);
            }
            if (LILG_FIELD(cmd, A).set) {
                float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
                if (!absolute_positioning) {
                    dest_deg = RotationalAxis_get_position_deg(&l_motor) + dest_deg;
                }
                RotationalAxis_start_move(&l_motor, dest_deg);
            }
            if (LILG_FIELD(cmd, B).set) {
                float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
                if (!absolute_positioning) {
                    dest_deg = RotationalAxis_get_position_deg(&r_motor) + dest_deg;
                }
                RotationalAxis_start_move(&r_motor, dest_deg);
            }

            // Wait for all axes to finish moving.
            if (cmd.Z.set) {
                LinearAxis_wait_for_move(&z_motor);
            }
            if (LILG_FIELD(cmd, A).set) {
                RotationalAxis_wait_for_move(&l_motor);
            }
            if (LILG_FIELD(cmd, B).set) {
                RotationalAxis_wait_for_move(&r_motor);
            }
        } break;

        // Home axes
        // https://marlinfw.org/docs/gcode/G28.html
        case 28: {
            LinearAxis_home(&z_motor);
        } break;

        // Absolute positioning
        // https://marlinfw.org/docs/gcode/G090.html
        case 90: {
            absolute_positioning = true;
        } break;

        // Relative positioning
        // https://marlinfw.org/docs/gcode/G091.html
        case 91: {
            absolute_positioning = false;
        } break;

        default:
            printf("Unknown command G%i\n", cmd.G.real);
            break;
    }
}

static void run_m_command(struct lilg_Command cmd) {
    switch (cmd.M.real) {
        // M17 enable steppers.
        case 17: {
            bool all = (!LILG_FIELD(cmd, Z).set) && (!LILG_FIELD(cmd, A).set) && (!LILG_FIELD(cmd, B).set);
            if (all || LILG_FIELD(cmd, Z).set) {
                gpio_put(z_motor.pin_enn, 0);
            }
            if (all || LILG_FIELD(cmd, A).set) {
                gpio_put(l_motor.pin_enn, 0);
            }
            if (all || LILG_FIELD(cmd, B).set) {
                gpio_put(r_motor.pin_enn, 0);
            }
        } break;

        // M18 disable steppers.
        case 18: {
            bool all = (!LILG_FIELD(cmd, Z).set) && (!LILG_FIELD(cmd, A).set) && (!LILG_FIELD(cmd, B).set);
            if (all || LILG_FIELD(cmd, Z).set) {
                gpio_put(z_motor.pin_enn, 1);
            }
            if (all || LILG_FIELD(cmd, A).set) {
                gpio_put(l_motor.pin_enn, 1);
            }
            if (all || LILG_FIELD(cmd, B).set) {
                gpio_put(r_motor.pin_enn, 1);
            }
        } break;

        // M114 get current position
        // https://marlinfw.org/docs/gcode/M114.html
        case 114: {
            printf(
                "Z:%0.2f A:%0.2f B:%0.2f Count Z:%i A:%i B:%i\n",
                LinearAxis_get_position_mm(&z_motor),
                RotationalAxis_get_position_deg(&l_motor),
                RotationalAxis_get_position_deg(&r_motor),
                z_motor.actual_steps,
                l_motor.actual_steps,
                r_motor.actual_steps);
        } break;

        // M115 get firmware info
        // https://marlinfw.org/docs/gcode/M115.html
        case 115: {
            printf("FIRMWARE_NAME:Picostep\n");
        } break;

        // M122 TMC debugging
        // https://marlinfw.org/docs/gcode/M122.html
        case 122: {
            TMC2209_print_all(&tmc_z);
            TMC2209_print_all(&tmc_left);
            TMC2209_print_all(&tmc_right);
        } break;

        // M150 set RGB
        // https://marlinfw.org/docs/gcode/M150.html
        case 150: {
            int32_t r = LILG_FIELD(cmd, R).real;
            int32_t g = LILG_FIELD(cmd, G).real;
            int32_t b = LILG_FIELD(cmd, B).real;
            Neopixel_set_all(pixels, NUM_PIXELS, r, g, b);
            Neopixel_write(pixels, NUM_PIXELS);
            printf("R:%i G:%i B: %i\n", r, g, b);
        } break;

        // M204 Set Starting Acceleration
        // https://marlinfw.org/docs/gcode/M204.html
        case 204: {
            float accel = lilg_Decimal_to_float(LILG_FIELD(cmd, T));
            z_motor.acceleration_mm_s2 = accel;
            printf("> Set acceleration to %0.2f mm/s^2\n", accel);
        } break;

        // M906 Set motor current
        // https://marlinfw.org/docs/gcode/M906.html
        case 906: {
            if (cmd.Z.set) {
                float current = lilg_Decimal_to_float(cmd.Z);
                TMC2209_set_current(&tmc_z, current, current * Z_HOLD_CURRENT_MULTIPLIER);
            }
            if (LILG_FIELD(cmd, A).set) {
                float current = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
                TMC2209_set_current(&tmc_left, current, current * A_HOLD_CURRENT_MULTIPLIER);
            }
            if (LILG_FIELD(cmd, B).set) {
                float current = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
                TMC2209_set_current(&tmc_right, current, current * B_HOLD_CURRENT_MULTIPLIER);
            }
        } break;

        // M914 Set bump sensitivity
        // https://marlinfw.org/docs/gcode/M914.html
        case 914: {
            z_motor.homing_sensitivity = cmd.Z.real;
            printf("> Set homing sensitivity to %u\n", cmd.Z.real);
        } break;

        // M997 firmware update
        // https://marlinfw.org/docs/gcode/M997.html
        case 997: {
            reset_usb_boot(0, 0);
        } break;

        default:
            printf("Unknown command M%i\n", cmd.M.real);
            break;
    }
}
