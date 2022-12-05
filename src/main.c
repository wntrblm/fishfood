/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "config/motion.h"
#include "config/pins.h"
#include "config/serial.h"
#include "drivers/neopixel.h"
#include "drivers/pca9495a.h"
#include "drivers/tmc_uart.h"
#include "drivers/xgzp6857d.h"
#include "gpio_commands.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "i2c_commands.h"
#include "littleg/littleg.h"
#include "machine.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "report.h"
#include <math.h>
#include <stdio.h>

#define NUM_PIXELS 8
static uint8_t pixels[3 * NUM_PIXELS];

static struct I2CCommandsState i2c_commands_state;
static struct Machine machine;

static void process_incoming_char(char c);
static void run_g_command(struct lilg_Command cmd);
static void run_m_command(struct lilg_Command cmd);

int main() {
    stdio_init_all();

    gpio_init(PIN_ACT_LED);
    gpio_set_dir(PIN_ACT_LED, GPIO_OUT);
    gpio_put(PIN_ACT_LED, true);

    gpio_init(PIN_AUX_OUT);
    gpio_set_dir(PIN_AUX_OUT, GPIO_OUT);
    gpio_put(PIN_AUX_OUT, false);

    Neopixel_init(PIN_CAM_LED);
    Neopixel_set_all(pixels, NUM_PIXELS, 255, 0, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    // Wait for USB connection before continuing.
    while (!stdio_usb_connected()) {}
    sleep_ms(1000);

    report_debug_ln("starting I2C peripheral bus...");
    i2c_init(PERIPH_I2C_INST, PERIPH_I2C_SPEED);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    i2c_commands_init(&i2c_commands_state);

    report_debug_ln("starting TMC UART...");
    uart_init(TMC_UART_INST, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    report_debug_ln("configuring motion and stepper motors...");
    // TODO: Check for errors!
    Machine_init(&machine);
    Machine_setup(&machine);

    report_info_ln("enabling steppers...");
    Machine_enable_steppers(&machine);

    report_info_ln("ready");
    Neopixel_set_all(pixels, NUM_PIXELS, 0, 0, 255);
    Neopixel_write(pixels, NUM_PIXELS);

    while (1) {
        int in_c = getchar_timeout_us(100);

        if (in_c == PICO_ERROR_TIMEOUT) {
            continue;
        }

        if (in_c == EOF) {
            break;
        }

        process_incoming_char((char)(in_c));
    }

    report_error_ln("main() loop exited due to end of file on stdin");
}

static inline void okay() { printf("\nok\n"); }

static void process_incoming_char(char c) {
    static struct lilg_Command cmd = {};

    enum lilg_ParseResult result = lilg_parse(&cmd, c);

    if (result == LILG_INCOMPLETE) {
        return;
    }

    if (result == LILG_INVALID) {
        report_error_ln("could not parse command");
        okay();
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
            report_error_ln("unexpected command %c%li\n", cmd.first_field, LILG_FIELDC(cmd, cmd.first_field).real);
        } break;
    }

    okay();
}

static void run_g_command(struct lilg_Command cmd) {
    switch (cmd.G.real) {
        // Linear move
        // https://marlinfw.org/docs/gcode/G000-G001.html
        case 0:
        case 1: {
            // F is "feed rate", or velocity in mm/min
            if (LILG_FIELD(cmd, F).set) {
                float vel_mm_s = lilg_Decimal_to_float(LILG_FIELD(cmd, F)) / 60.0f;
                Machine_set_linear_velocity(&machine, vel_mm_s);
            }

            Machine_move(&machine, cmd);
        } break;

        // Millimeter Units
        // https://marlinfw.org/docs/gcode/G21.html
        // OpenPnP sends this as part of CONNECT_COMMAND by default.
        case 21: {
            // no op
        } break;

        // Home axes
        // https://marlinfw.org/docs/gcode/G28.html
        case 28: {
            Machine_home(&machine, cmd.X.set, cmd.Y.set, cmd.Z.set);
        } break;

        // Absolute positioning
        // https://marlinfw.org/docs/gcode/G090.html
        case 90: {
            machine.absolute_positioning = true;
        } break;

        // Relative positioning
        // https://marlinfw.org/docs/gcode/G091.html
        case 91: {
            machine.absolute_positioning = false;
        } break;

        // Set position (without movement)
        // https://marlinfw.org/docs/gcode/G092.html
        case 92: {
            Machine_set_position(&machine, cmd);
        } break;

        default:
            report_error_ln("unknown command G%li", cmd.G.real);
            break;
    }
}

static void run_m_command(struct lilg_Command cmd) {
    switch (cmd.M.real) {
        // M17 enable steppers.
        case 17: {
            Machine_enable_steppers(&machine);
        } break;

        // M18 disable steppers.
        // https://marlinfw.org/docs/gcode/M018.html
        case 18: {
            Machine_disable_steppers(&machine);
        } break;

        // M42 Set pin state
        // This is slightly different from Marlin's implementation
        // https://marlinfw.org/docs/gcode/M042.html
        case 42: {
            gpio_commands_m42_set_pin(cmd);
        } break;

        // M43 Report pin state
        // https://marlinfw.org/docs/gcode/M043.html
        case 43: {
            gpio_commands_m43_report_pin(cmd);
        } break;

        // M82 Extruder Absolute Positioning
        // https://marlinfw.org/docs/gcode/M082.html
        // OpenPnP defaults to sending this as part of CONNECT_COMMAND
        case 82: {
            // no op
        } break;

        // M112 emergency stop
        // https://marlinfw.org/docs/gcode/M112.html
        case 112: {
            // disable all movement
            Machine_disable_steppers(&machine);

#ifdef STARFISH
            // disable outputs for pumps and valves.
            gpio_put(PIN_PUMP_A, false);
            gpio_put(PIN_PUMP_B, false);
            gpio_put(PIN_VALVE_A, false);
            gpio_put(PIN_VALVE_B, false);
#endif

            // give visual indicator of shutdown state with LEDs set to RED.
            Neopixel_set_all(pixels, NUM_PIXELS, 255, 0, 0);
            Neopixel_write(pixels, NUM_PIXELS);
        } break;

        // M114 get current position
        // https://marlinfw.org/docs/gcode/M114.html
        case 114: {
            Machine_report_position(&machine);
        } break;

        // M115 get firmware info
        // https://marlinfw.org/docs/gcode/M115.html
        case 115: {
            report_result_ln("FIRMWARE_NAME:Fishfoosh MACHINE_TYPE:" FISHFOOD_BOARD);
        } break;

        // M122 TMC debugging
        // https://marlinfw.org/docs/gcode/M122.html
        case 122: {
            Machine_report_tmc_info(&machine);
        } break;

        // M150 set RGB
        // https://marlinfw.org/docs/gcode/M150.html
        case 150: {
            uint8_t r = 0;  // off
            uint8_t g = 0;  // off
            uint8_t b = 0;  // off

            // Keep current color settings, if a specific index is not provided use first LED.
            // MUST be processed before other parameters
            if (LILG_FIELD(cmd, K).set) {
                size_t index = 0;
                if (LILG_FIELD(cmd, I).set) {
                    index = (size_t)LILG_FIELD(cmd, I).real;
                }
                Neopixel_get(pixels, index, &r, &g, &b);
            }

            // Red color component, value 0-255
            if (LILG_FIELD(cmd, R).set) {
                r = (uint8_t)LILG_FIELD(cmd, R).real;
            }

            // Green color component, value 0-255
            // Marlin docs indicate to use 'U' insead of 'G' so use whichever is set.
            if (LILG_FIELD(cmd, G).set) {
                g = (uint8_t)LILG_FIELD(cmd, G).real;
            } else if (LILG_FIELD(cmd, U).set) {
                g = (uint8_t)LILG_FIELD(cmd, U).real;
            }

            // Blue color component, value 0-255
            if (LILG_FIELD(cmd, B).set) {
                b = (uint8_t)LILG_FIELD(cmd, B).real;
            }

            // P parameter controls the brightness, value 0-255
            if (LILG_FIELD(cmd, P).set) {
                float intensity = LILG_FIELD(cmd, P).real / 255.0f;
                r = roundf(r * intensity);
                g = roundf(g * intensity);
                b = roundf(b * intensity);
            }

            // I parameter controls which pixel is being updated, if not present all pixels will be updated
            if (LILG_FIELD(cmd, I).set) {
                size_t index = (size_t)LILG_FIELD(cmd, I).real;
                Neopixel_set(pixels, index, r, g, b);
                report_result_ln("I:%zu R:%i G:%i B:%i", index, r, g, b);
            } else {
                Neopixel_set_all(pixels, NUM_PIXELS, r, g, b);
                report_result_ln("R:%i G:%i B:%i", r, g, b);
            }
            Neopixel_write(pixels, NUM_PIXELS);
        } break;

        // M204 Set Starting Acceleration
        // https://marlinfw.org/docs/gcode/M204.html
        case 204: {
            float accel = 0;
            bool set = false;
            if (LILG_FIELD(cmd, T).set) {
                accel = lilg_Decimal_to_float(LILG_FIELD(cmd, T));
                set = true;
            }
            // OpenPnP defaults to using 'S' instead of 'T' in the Issues & Solutions tab.
            if (LILG_FIELD(cmd, S).set) {
                accel = lilg_Decimal_to_float(LILG_FIELD(cmd, S));
                set = true;
            }
            if (set) {
                Machine_set_linear_acceleration(&machine, accel);
            }
            Machine_report_linear_acceleration(&machine);
        } break;

        // M260 I2C Send
        // https://marlinfw.org/docs/gcode/M260.html
        case 260: {
            i2c_commands_m260_send(&i2c_commands_state, cmd);
        } break;

        // M261 I2C Request
        case 261: {
            i2c_commands_m261_request(&i2c_commands_state, cmd);
        } break;

        // M262: I2C Scan
        // Non-standard.
        case 262: {
            i2c_commands_m262_scan();
        } break;

        // M263: I2C pressure sensor read
        // Non-standard
        case 263: {
            uint8_t which = LILG_FIELD(cmd, P).real == 0 ? 3 : 2;

            if (pca9495a_switch_channel(PERIPH_I2C_INST, I2C_MUX_ADDR, which, PERIPH_I2C_TIMEOUT) < 0) {
                report_error_ln("failed to change I2C multiplexer configuration");
                return;
            }

            int32_t pressure = XGZP6857D_read(PERIPH_I2C_INST, PERIPH_I2C_TIMEOUT);

            if (pressure > 0) {
                report_result_ln("pressure:%li", pressure);
            }
        } break;

        // M400: Finish moves
        // https://marlinfw.org/docs/gcode/M400.html
        case 400: {
            // no-op since Fishfood does not reply to G0/G1 until moves are finished.
        } break;

        // M503 Report Settings
        // https://marlinfw.org/docs/gcode/M503.html
        case 503: {
            Machine_report_linear_acceleration(&machine);
            Machine_report_position(&machine);
            Machine_report_tmc_info(&machine);
        } break;

        // M906 Set motor current
        // https://marlinfw.org/docs/gcode/M906.html
        case 906: {
            Machine_set_motor_current(&machine, cmd);
        } break;

        // M914 Set bump sensitivity
        // https://marlinfw.org/docs/gcode/M914.html
        case 914: {
            Machine_set_homing_sensitivity(&machine, cmd);
        } break;

        // M997 firmware update
        // https://marlinfw.org/docs/gcode/M997.html
        case 997: {
            reset_usb_boot(0, 0);
        } break;

        // M999 reboot
        // https://marlinfw.org/docs/gcode/M999.html
        case 999: {
            // This uses the watchdog to force an immediate reboot.
            watchdog_reboot(0, 0, 0);
        } break;

        default:
            report_error_ln("unknown command M%li\n", cmd.M.real);
            break;
    }
}
