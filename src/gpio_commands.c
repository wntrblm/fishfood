/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "gpio_commands.h"
#include "config/pins.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "report.h"
#include <stdio.h>

void gpio_commands_init() {
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 0xffu);
    for (size_t i = 0; i < 7; i++) { pwm_init(i, &config, true); }
}

void gpio_commands_m42_set_pin(const struct lilg_Command cmd) {
    uint8_t pin_index = LILG_FIELD(cmd, P).real;
    if (pin_index >= M42_PIN_TABLE_LEN) {
        report_error_ln("no pin at index %u", pin_index);
        return;
    }

    const struct M42PinTableEntry pin_desc = M42_PIN_TABLE[pin_index];

    if (LILG_FIELD(cmd, T).set) {
        switch (LILG_FIELD(cmd, T).real) {
            // Input
            case 0: {
                gpio_init(pin_desc.pin);
                gpio_set_dir(pin_desc.pin, GPIO_IN);
                gpio_set_function(pin_desc.pin, GPIO_FUNC_NULL);
            } break;
            // Output
            case 1: {
                gpio_init(pin_desc.pin);
                gpio_set_dir(pin_desc.pin, GPIO_OUT);
            } break;
            // Input, pull-up
            case 2: {
                gpio_init(pin_desc.pin);
                gpio_set_dir(pin_desc.pin, GPIO_IN);
                gpio_pull_up(pin_desc.pin);
            } break;
            // Input, pull-down
            case 3: {
                gpio_init(pin_desc.pin);
                gpio_set_dir(pin_desc.pin, GPIO_IN);
                gpio_pull_down(pin_desc.pin);
            } break;
            // Output, PWM
            case 4: {
                gpio_init(pin_desc.pin);
                gpio_set_dir(pin_desc.pin, GPIO_OUT);
                gpio_set_function(pin_desc.pin, GPIO_FUNC_PWM);
                // pwm_set_gpio_level(pin_desc.pin, 0);
            } break;
            default: {
                report_error_ln(
                    "invalid type %li, must be 0 (input), 1 (output), 2 (input, pull-up), 3 (input, pull-down), or 4 "
                    "(output, PWM).",
                    LILG_FIELD(cmd, S).real);
                return;
            };
        }
    }

    if (LILG_FIELD(cmd, S).set) {
        if (gpio_get_function(pin_desc.pin) == GPIO_FUNC_SIO) {
            bool val = LILG_FIELD(cmd, S).real > 0 ? true : false;
            gpio_put(pin_desc.pin, val);
            report_result_ln("P:%u name:%s GPIO:%u S:%u", pin_index, pin_desc.name, pin_desc.pin, val);
        } else if (gpio_get_function(pin_desc.pin) == GPIO_FUNC_PWM) {
            uint8_t val = (uint8_t)(LILG_FIELD(cmd, S).real);
            pwm_set_gpio_level(pin_desc.pin, val);
            report_result_ln("P:%u name:%s GPIO:%u S:%u (PWM)", pin_index, pin_desc.name, pin_desc.pin, val);
        } else {
            report_error_ln("pin %u is not an output pin", pin_index);
        }
    }
}

void gpio_commands_m43_report_pin(const struct lilg_Command cmd) {
    if (!LILG_FIELD(cmd, P).set) {
        // Report all pins
        for (size_t i = 0; i < M42_PIN_TABLE_LEN; i++) {
            const struct M42PinTableEntry pin_desc = M42_PIN_TABLE[i];
            report_result_ln(
                "P:%u name:%s GPIO:%u dir:%s state:%u",
                i,
                pin_desc.name,
                pin_desc.pin,
                gpio_is_dir_out(pin_desc.pin) ? "output" : "input",
                gpio_is_dir_out(pin_desc.pin) ? gpio_get_out_level(pin_desc.pin) : gpio_get(pin_desc.pin));
        }
    } else {
        uint8_t pin_index = LILG_FIELD(cmd, P).real;
        if (pin_index >= M42_PIN_TABLE_LEN) {
            report_error_ln("no pin at index %u", pin_index);
            return;
        }

        const struct M42PinTableEntry pin_desc = M42_PIN_TABLE[pin_index];

        report_result_ln(
            "P:%u name:%s GPIO:%u dir:%s state:%u",
            pin_index,
            pin_desc.name,
            pin_desc.pin,
            gpio_is_dir_out(pin_desc.pin) ? "output" : "input",
            gpio_is_dir_out(pin_desc.pin) ? gpio_get_out_level(pin_desc.pin) : gpio_get(pin_desc.pin));
    }
}
