#include "config/motion.h"
#include "config/pins.h"
#include "config/serial.h"
#include "drivers/neopixel.h"
#include "drivers/tmc2209.h"
#include "drivers/tmc2209_helper.h"
#include "drivers/tmc_uart.h"
#include "drivers/xgzp6857d.h"
#include "drivers/pca9495a.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "linear_axis.h"
#include "littleg/littleg.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "rotational_axis.h"
#include "bresenham.h"
#include "i2c_commands.h"
#include "wntr_pack.h"
#include <math.h>
#include <stdio.h>

#define NUM_PIXELS 8
static uint8_t pixels[3 * NUM_PIXELS];

static struct I2CCommandsState i2c_commands_state;

static struct TMC2209 tmc0;
static struct TMC2209 tmc1;
static struct TMC2209 tmc2;
static struct Stepper stepper0;
static struct Stepper stepper1;
static struct Stepper stepper2;

static struct LinearAxis x_axis;
static struct LinearAxis y_axis;
static struct LinearAxis z_axis;
static struct RotationalAxis a_axis;
static struct RotationalAxis b_axis;

static bool coordinated_move = false;
static struct Bresenham bresenham;
static struct LinearAxis* major_axis = NULL;
static struct LinearAxis* minor_axis = NULL;

static repeating_timer_t step_timer;

static bool absolute_positioning = true;

static int64_t step_timer_callback(alarm_id_t id, void* user_data);
static void process_incoming_char(char c);
static void run_g_command(struct lilg_Command cmd);
static void run_m_command(struct lilg_Command cmd);

int main() {
    stdio_init_all();

    irq_set_priority(TIMER_IRQ_0, PICO_DEFAULT_IRQ_PRIORITY + 1);
    irq_set_priority(TIMER_IRQ_1, PICO_DEFAULT_IRQ_PRIORITY + 1);
    irq_set_priority(TIMER_IRQ_2, PICO_DEFAULT_IRQ_PRIORITY + 1);
    irq_set_priority(TIMER_IRQ_3, PICO_DEFAULT_IRQ_PRIORITY + 1);

    gpio_init(PIN_ACT_LED);
    gpio_set_dir(PIN_ACT_LED, GPIO_OUT);
    gpio_put(PIN_ACT_LED, true);

    gpio_init(PIN_AUX_OUT);
    gpio_set_dir(PIN_AUX_OUT, GPIO_OUT);
    gpio_put(PIN_AUX_OUT, false);

    Neopixel_init(PIN_CAM_LED);
    Neopixel_set_all(pixels, NUM_PIXELS, 255, 0, 0);
    Neopixel_write(pixels, NUM_PIXELS);

    TMC2209_init(&tmc0, TMC_UART_INST, 0, tmc_uart_read_write);
    TMC2209_init(&tmc1, TMC_UART_INST, 1, tmc_uart_read_write);
    // Note: This should be 2, but both Jellyfish & Starfish skip address 2.
    TMC2209_init(&tmc2, TMC_UART_INST, 3, tmc_uart_read_write);

    // Wait for USB connection before continuing.
    while (!stdio_usb_connected()) {}

    printf("| Starting I2C peripheral bus...");
    i2c_commands_init(&i2c_commands_state);

    printf("| Starting TMC UART...\n");
    uart_init(TMC_UART_INST, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

#ifdef HAS_XY_AXES
    Stepper_init(&X_STEPPER, &X_TMC, X_PIN_EN, X_PIN_DIR, X_PIN_STEP, X_PIN_DIAG, X_REVERSED);
    Stepper_setup(&X_STEPPER);
    LinearAxis_init(&x_axis, 'X', &X_STEPPER);
    x_axis.steps_per_mm = X_STEPS_PER_MM;
    x_axis.velocity_mm_s = X_DEFAULT_VELOCITY_MM_S;
    x_axis.acceleration_mm_s2 = X_DEFAULT_ACCELERATION_MM_S2;
    x_axis.homing_direction = X_HOMING_DIR;
    x_axis.homing_distance_mm = X_HOMING_DISTANCE_MM;
    x_axis.homing_bounce_mm = X_HOMING_BOUNCE_MM;
    x_axis.homing_velocity_mm_s = X_HOMING_VELOCITY_MM_S;
    x_axis.homing_acceleration_mm_s2 = X_HOMING_ACCELERATION_MM_S2;
    x_axis.homing_sensitivity = X_HOMING_SENSITIVITY;

    Stepper_init(&Y1_STEPPER, &Y1_TMC, Y1_PIN_EN, Y1_PIN_DIR, Y1_PIN_STEP, Y1_PIN_DIAG, Y_REVERSED);
    Stepper_setup(&Y1_STEPPER);
    Stepper_init(&Y2_STEPPER, &Y2_TMC, Y2_PIN_EN, Y2_PIN_DIR, Y2_PIN_STEP, Y2_PIN_DIAG, !Y_REVERSED);
    Stepper_setup(&Y2_STEPPER);
    LinearAxis_init(&y_axis, 'Y', &Y1_STEPPER);
    LinearAxis_setup_dual(&y_axis, &Y2_STEPPER);
    y_axis.steps_per_mm = Y_STEPS_PER_MM;
    y_axis.velocity_mm_s = Y_DEFAULT_VELOCITY_MM_S;
    y_axis.acceleration_mm_s2 = Y_DEFAULT_ACCELERATION_MM_S2;
    y_axis.homing_direction = Y_HOMING_DIR;
    y_axis.homing_distance_mm = Y_HOMING_DISTANCE_MM;
    y_axis.homing_bounce_mm = Y_HOMING_BOUNCE_MM;
    y_axis.homing_velocity_mm_s = Y_HOMING_VELOCITY_MM_S;
    y_axis.homing_acceleration_mm_s2 = Y_HOMING_ACCELERATION_MM_S2;
    y_axis.homing_sensitivity = Y_HOMING_SENSITIVITY;
#endif

#ifdef HAS_Z_AXIS
    Stepper_init(&Z_STEPPER, &Z_TMC, Z_PIN_EN, Z_PIN_DIR, Z_PIN_STEP, Z_PIN_DIAG, Z_REVERSED);
    Stepper_setup(&Z_STEPPER);
    LinearAxis_init(&z_axis, 'Z', &Z_STEPPER);
    z_axis.steps_per_mm = Z_STEPS_PER_MM;
    z_axis.velocity_mm_s = Z_DEFAULT_VELOCITY_MM_S;
    z_axis.acceleration_mm_s2 = Z_DEFAULT_ACCELERATION_MM_S2;
    z_axis.homing_direction = Z_HOMING_DIR;
    z_axis.homing_distance_mm = Z_HOMING_DISTANCE_MM;
    z_axis.homing_bounce_mm = Z_HOMING_BOUNCE_MM;
    z_axis.homing_velocity_mm_s = Z_HOMING_VELOCITY_MM_S;
    z_axis.homing_acceleration_mm_s2 = Z_HOMING_ACCELERATION_MM_S2;
    z_axis.homing_sensitivity = Z_HOMING_SENSITIVITY;
#endif

#ifdef HAS_A_AXIS
    Stepper_init(&A_STEPPER, &A_TMC, A_PIN_EN, A_PIN_DIR, A_PIN_STEP, A_PIN_DIAG, false);
    Stepper_setup(&A_STEPPER);
    RotationalAxis_init(&a_axis, 'A', &A_STEPPER);
    a_axis.steps_per_deg = A_STEPS_PER_DEG;
#endif

#ifdef HAS_B_AXIS
    Stepper_init(&B_STEPPER, &B_TMC, B_PIN_EN, B_PIN_DIR, B_PIN_STEP, B_PIN_DIAG, false);
    Stepper_setup(&B_STEPPER);
    RotationalAxis_init(&b_axis, 'B', &B_STEPPER);
    b_axis.steps_per_deg = B_STEPS_PER_DEG;
#endif

    printf("| Setting motor current...\n");
#ifdef HAS_XY_AXES
    Stepper_set_current(x_axis.stepper, X_RUN_CURRENT, X_RUN_CURRENT * X_HOLD_CURRENT_MULTIPLIER);
    Stepper_set_current(y_axis.stepper, Y_RUN_CURRENT, Y_RUN_CURRENT * Y_HOLD_CURRENT_MULTIPLIER);
    Stepper_set_current(y_axis.stepper2, Y_RUN_CURRENT, Y_RUN_CURRENT * Y_HOLD_CURRENT_MULTIPLIER);
#endif
#ifdef HAS_Z_AXIS
    Stepper_set_current(z_axis.stepper, Z_RUN_CURRENT, Z_RUN_CURRENT * Z_HOLD_CURRENT_MULTIPLIER);
#endif
#ifdef HAS_A_AXIS
    Stepper_set_current(a_axis.stepper, A_RUN_CURRENT, A_RUN_CURRENT * A_HOLD_CURRENT_MULTIPLIER);
#endif
#ifdef HAS_B_AXIS
    Stepper_set_current(b_axis.stepper, B_RUN_CURRENT, B_RUN_CURRENT * B_HOLD_CURRENT_MULTIPLIER);
#endif

    printf("| Enabling steppers...\n");
#ifdef HAS_XY_AXES
    Stepper_enable(x_axis.stepper);
    Stepper_enable(y_axis.stepper);
    Stepper_enable(y_axis.stepper2);
#endif
#ifdef HAS_Z_AXIS
    Stepper_enable(z_axis.stepper);
#endif
#ifdef HAS_A_AXIS
    Stepper_enable(a_axis.stepper);
#endif
#ifdef HAS_B_AXIS
    Stepper_enable(b_axis.stepper);
#endif

    printf("| Starting step timer...\n");
    uint32_t irq_status = save_and_disable_interrupts();
    alarm_pool_t* alarm_pool = alarm_pool_get_default();
    alarm_pool_add_alarm_at(alarm_pool, make_timeout_time_us(1000), step_timer_callback, NULL, false);
    restore_interrupts(irq_status);

    printf("| Ready!\n");
    Neopixel_set_all(pixels, NUM_PIXELS, 0, 0, 255);
    Neopixel_write(pixels, NUM_PIXELS);

    while (1) {
        int in_c = getchar();
        if (in_c == EOF) {
            break;
        }

        process_incoming_char((char)(in_c));
    }

    printf("! Main loop exited due to end of file on stdin\n");
}

static int64_t step_timer_callback(alarm_id_t id, void* user_data) {
    gpio_put(PIN_AUX_LED, true);
#ifdef HAS_XY_AXES
    if(coordinated_move) {
        LinearAxis_step(major_axis);
        if(Bresenham_step(&bresenham)) {
            LinearAxis_step(minor_axis);
        }
    } else {
        LinearAxis_step(&x_axis);
        LinearAxis_step(&y_axis);
    }
#endif
#ifdef HAS_Z_AXIS
    LinearAxis_step(&z_axis);
#endif
#ifdef HAS_A_AXIS
    RotationalAxis_step(&a_axis);
#endif
#ifdef HAS_B_AXIS
    RotationalAxis_step(&b_axis);
#endif
    gpio_put(PIN_AUX_LED, false);
    return STEP_INTERVAL_US;
}

static void process_incoming_char(char c) {
    static struct lilg_Command cmd = {};

    enum lilg_ParseResult result = lilg_parse(&cmd, c);

    if (result == LILG_INCOMPLETE) {
        return;
    }

    if (result == LILG_INVALID) {
        printf("! Could not parse command\n");
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
            printf("! Unexpected command %c%i\n", cmd.first_field, LILG_FIELDC(cmd, cmd.first_field));
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
                x_axis.velocity_mm_s = mm_per_min / 60.0f;
                y_axis.velocity_mm_s = mm_per_min / 60.0f;
                z_axis.velocity_mm_s = mm_per_min / 60.0f;
            }

#ifdef HAS_XY_AXES
            float x_dest_mm = lilg_Decimal_to_float(cmd.X);
            float y_dest_mm = lilg_Decimal_to_float(cmd.Y);
            if (!absolute_positioning) {
                x_dest_mm = LinearAxis_get_position_mm(&x_axis) + x_dest_mm;
                y_dest_mm = LinearAxis_get_position_mm(&y_axis) + y_dest_mm;
            }
            struct LinearAxisMovement x_move = LinearAxis_calculate_move(&x_axis, x_dest_mm);
            struct LinearAxisMovement y_move = LinearAxis_calculate_move(&y_axis, y_dest_mm);

            uint32_t irq_status = save_and_disable_interrupts();

            if(cmd.X.set && cmd.Y.set) {
                coordinated_move = true;
                if (x_move.total_step_count > y_move.total_step_count) {
                    major_axis = &x_axis;
                    minor_axis = &y_axis;
                    Bresenham_init(&bresenham, 0, 0, x_move.total_step_count, y_move.total_step_count);
                } else {
                    major_axis = &y_axis;
                    minor_axis = &x_axis;
                    Bresenham_init(&bresenham, 0, 0, y_move.total_step_count, x_move.total_step_count);
                }
                printf("> Coordinated move: major axis: %c, minor axis: %c, major steps: %i, minor steps: %i\n", major_axis->name, minor_axis->name, bresenham.x1, bresenham.y1);
            } else {
                coordinated_move = false;
            }

            if (cmd.X.set) {
                LinearAxis_start_move(&x_axis, x_move);
            }
            if (cmd.Y.set) {
                LinearAxis_start_move(&y_axis, y_move);
            }

            restore_interrupts(irq_status);
#endif

#ifdef HAS_Z_AXIS
            if (cmd.Z.set) {
                float dest_mm = lilg_Decimal_to_float(cmd.Z);
                if (!absolute_positioning) {
                    dest_mm = LinearAxis_get_position_mm(&z_axis) + dest_mm;
                }
                LinearAxis_start_move(&z_axis, dest_mm);
            }
#endif
#ifdef HAS_A_AXIS
            if (LILG_FIELD(cmd, A).set) {
                float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
                if (!absolute_positioning) {
                    dest_deg = RotationalAxis_get_position_deg(&a_axis) + dest_deg;
                }
                RotationalAxis_start_move(&a_axis, dest_deg);
            }
#endif
#ifdef HAS_B_AXIS
            if (LILG_FIELD(cmd, B).set) {
                float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
                if (!absolute_positioning) {
                    dest_deg = RotationalAxis_get_position_deg(&b_axis) + dest_deg;
                }
                RotationalAxis_start_move(&b_axis, dest_deg);
            }
#endif

            // Wait for all axes to finish moving.
#ifdef HAS_XY_AXES
            if (cmd.X.set) {
                LinearAxis_wait_for_move(&x_axis);
            }
            if (cmd.Y.set) {
                LinearAxis_wait_for_move(&y_axis);
            }
#endif
#ifdef HAS_Z_AXIS
            if (cmd.Z.set) {
                LinearAxis_wait_for_move(&z_axis);
            }
#endif
#ifdef HAS_A_AXIS
            if (LILG_FIELD(cmd, A).set) {
                RotationalAxis_wait_for_move(&a_axis);
            }
#endif
#ifdef HAS_B_AXIS
            if (LILG_FIELD(cmd, B).set) {
                RotationalAxis_wait_for_move(&b_axis);
            }
#endif
        } break;

        // Home axes
        // https://marlinfw.org/docs/gcode/G28.html
        case 28: {
#ifdef HAS_XY_AXES
            if (cmd.X.set) {
                LinearAxis_home(&x_axis);
            }
            if (cmd.Y.set) {
                LinearAxis_home(&y_axis);
            }
#endif
#ifdef HAS_Z_AXIS
            if (cmd.Z.set) {
                LinearAxis_home(&z_axis);
            }
#endif
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
            printf("! Unknown command G%i\n", cmd.G.real);
            break;
    }
}

static void run_m_command(struct lilg_Command cmd) {
    switch (cmd.M.real) {
        // M17 enable steppers.
        case 17: {
            bool all = ((!LILG_FIELD(cmd, X).set) && (!LILG_FIELD(cmd, Y).set) && !LILG_FIELD(cmd, Z).set) &&
                       (!LILG_FIELD(cmd, A).set) && (!LILG_FIELD(cmd, B).set);
#ifdef HAS_XY_AXES
            if (all || LILG_FIELD(cmd, X).set) {
                Stepper_enable(x_axis.stepper);
            }
            if (all || LILG_FIELD(cmd, Y).set) {
                Stepper_enable(y_axis.stepper);
                Stepper_enable(y_axis.stepper2);
            }
#endif
#ifdef HAS_Z_AXIS
            if (all || LILG_FIELD(cmd, Z).set) {
                Stepper_enable(z_axis.stepper);
            }
#endif
#ifdef HAS_A_AXIS
            if (all || LILG_FIELD(cmd, A).set) {
                Stepper_enable(a_axis.stepper);
            }
#endif
#ifdef HAS_B_AXIS
            if (all || LILG_FIELD(cmd, B).set) {
                Stepper_enable(b_axis.stepper);
            }
#endif
        } break;

        // M18 disable steppers.
        case 18: {
            bool all = ((!LILG_FIELD(cmd, X).set) && (!LILG_FIELD(cmd, Y).set) && !LILG_FIELD(cmd, Z).set) &&
                       (!LILG_FIELD(cmd, A).set) && (!LILG_FIELD(cmd, B).set);
#ifdef HAS_XY_AXES
            if (all || LILG_FIELD(cmd, X).set) {
                Stepper_disable(x_axis.stepper);
            }
            if (all || LILG_FIELD(cmd, Y).set) {
                Stepper_disable(y_axis.stepper);
                Stepper_disable(y_axis.stepper2);
            }
#endif
#ifdef HAS_Z_AXIS
            if (all || LILG_FIELD(cmd, Z).set) {
                Stepper_disable(z_axis.stepper);
            }
#endif
#ifdef HAS_A_AXIS
            if (all || LILG_FIELD(cmd, A).set) {
                Stepper_disable(a_axis.stepper);
            }
#endif
#ifdef HAS_B_AXIS
            if (all || LILG_FIELD(cmd, B).set) {
                Stepper_disable(b_axis.stepper);
            }
#endif
        } break;

        // M42 Set pin state
        // This is slightly different from Marlin's implementation
        // https://marlinfw.org/docs/gcode/M042.html
        case 42: {
            uint8_t pin_index = LILG_FIELD(cmd, P).real;
            if (pin_index >= M42_PIN_TABLE_LEN) {
                printf("! No pin at index %u\n", pin_index);
                return;
            }

            const struct M42PinTableEntry pin_desc = M42_PIN_TABLE[pin_index];

            if (LILG_FIELD(cmd, T).set) {
                switch (LILG_FIELD(cmd, S).real) {
                    // Input
                    case 0: {
                        gpio_init(pin_desc.pin);
                        gpio_set_dir(pin_desc.pin, GPIO_IN);
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
                    default: {
                        printf("! Invalid type %u, must be 0, 1, 2, or 3.\n", LILG_FIELD(cmd, S).real);
                        return;
                    };
                }
            }

            if (LILG_FIELD(cmd, S).set) {
                bool val = LILG_FIELD(cmd, S).real > 0 ? true : false;
                gpio_put(pin_desc.pin, val);
                printf("> P:%u name:%s GPIO:%u S:%u\n", pin_index, pin_desc.name, pin_desc.pin, val);
            }
        } break;

        // M43 Report pin state
        // https://marlinfw.org/docs/gcode/M043.html
        case 43: {
            if (!LILG_FIELD(cmd, P).set) {
                // Report all pins
                for (size_t i = 0; i < M42_PIN_TABLE_LEN; i++) {
                    const struct M42PinTableEntry pin_desc = M42_PIN_TABLE[i];
                    printf(
                        "> P:%u name:%s GPIO:%u dir:%s state:%u\n",
                        i,
                        pin_desc.name,
                        pin_desc.pin,
                        gpio_is_dir_out(pin_desc.pin) ? "output" : "input",
                        gpio_is_dir_out(pin_desc.pin) ? gpio_get_out_level(pin_desc.pin) : gpio_get(pin_desc.pin));
                }
            } else {
                uint8_t pin_index = LILG_FIELD(cmd, P).real;
                if (pin_index >= M42_PIN_TABLE_LEN) {
                    printf("! No pin at index %u\n", pin_index);
                    return;
                }

                const struct M42PinTableEntry pin_desc = M42_PIN_TABLE[pin_index];

                printf(
                    "> P:%u name:%s GPIO:%u dir:%s state:%u",
                    pin_index,
                    pin_desc.name,
                    pin_desc.pin,
                    gpio_is_dir_out(pin_desc.pin) ? "output" : "input",
                    gpio_is_dir_out(pin_desc.pin) ? gpio_get_out_level(pin_desc.pin) : gpio_get(pin_desc.pin));
            }
        } break;

        // M114 get current position
        // https://marlinfw.org/docs/gcode/M114.html
        case 114: {
            printf(
                "Z:%0.2f A:%0.2f B:%0.2f Count Z:%i A:%i B:%i\n",
                LinearAxis_get_position_mm(&z_axis),
                RotationalAxis_get_position_deg(&a_axis),
                RotationalAxis_get_position_deg(&b_axis),
                z_axis.stepper->total_steps,
                a_axis.stepper->total_steps,
                b_axis.stepper->total_steps);
        } break;

        // M115 get firmware info
        // https://marlinfw.org/docs/gcode/M115.html
        case 115: {
            printf("> firmware_name:Picostep\n");
        } break;

        // M122 TMC debugging
        // https://marlinfw.org/docs/gcode/M122.html
        case 122: {
            TMC2209_print_all(&tmc0);
            TMC2209_print_all(&tmc1);
            TMC2209_print_all(&tmc2);
        } break;

        // M150 set RGB
        // https://marlinfw.org/docs/gcode/M150.html
        case 150: {
            int32_t r = LILG_FIELD(cmd, R).real;
            int32_t g = LILG_FIELD(cmd, G).real;
            int32_t b = LILG_FIELD(cmd, B).real;
            Neopixel_set_all(pixels, NUM_PIXELS, r, g, b);
            Neopixel_write(pixels, NUM_PIXELS);
            printf("> R:%i G:%i B: %i\n", r, g, b);
        } break;

        // M204 Set Starting Acceleration
        // https://marlinfw.org/docs/gcode/M204.html
        case 204: {
            float reported_accel = 0;
            if (LILG_FIELD(cmd, T).set) {
                float accel = lilg_Decimal_to_float(LILG_FIELD(cmd, T));
#ifdef HAS_XY_AXES
                x_axis.acceleration_mm_s2 = accel;
                reported_accel = accel;
                y_axis.acceleration_mm_s2 = accel;
                reported_accel = accel;
#endif
#ifdef HAS_Z_AXIS
                z_axis.acceleration_mm_s2 = accel;
                reported_accel = accel;
#endif
            }
            printf("> T:%0.2f mm/s^2\n", reported_accel);
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
            uint8_t which = LILG_FIELD(cmd, P).real == 0 ? 0x08 : 0x04;

            if (pca9495a_switch_channel(PERIPH_I2C_INST, I2C_MUX_ADDR, which, PERIPH_I2C_TIMEOUT) < 0) {
                printf("! Failed to change I2C multiplexer configuration.\n");
                return;
            }

            int32_t pressure = XGZP6857D_read(PERIPH_I2C_INST, PERIPH_I2C_TIMEOUT);
            printf("> Pressure: %u\n", pressure);
        } break;

        // M906 Set motor current
        // https://marlinfw.org/docs/gcode/M906.html
        case 906: {
#ifdef HAS_XY_AXES
            if (cmd.X.set) {
                float current = lilg_Decimal_to_float(cmd.X);
                TMC2209_set_current(&X_TMC, current, current * X_HOLD_CURRENT_MULTIPLIER);
            }
            if (cmd.Y.set) {
                float current = lilg_Decimal_to_float(cmd.Y);
                TMC2209_set_current(&Y1_TMC, current, current * Y_HOLD_CURRENT_MULTIPLIER);
                TMC2209_set_current(&Y2_TMC, current, current * Y_HOLD_CURRENT_MULTIPLIER);
            }
#endif
#ifdef HAS_Z_AXIS
            if (cmd.Z.set) {
                float current = lilg_Decimal_to_float(cmd.Z);
                TMC2209_set_current(&Z_TMC, current, current * Z_HOLD_CURRENT_MULTIPLIER);
            }
#endif
#ifdef HAS_A_AXIS
            if (LILG_FIELD(cmd, A).set) {
                float current = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
                TMC2209_set_current(&A_TMC, current, current * A_HOLD_CURRENT_MULTIPLIER);
            }
#endif
#ifdef HAS_B_AXIS
            if (LILG_FIELD(cmd, B).set) {
                float current = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
                TMC2209_set_current(&B_TMC, current, current * B_HOLD_CURRENT_MULTIPLIER);
            }
#endif
        } break;

        // M914 Set bump sensitivity
        // https://marlinfw.org/docs/gcode/M914.html
        case 914: {
#ifdef HAS_XY_AXES
            if (cmd.X.set) {
                x_axis.homing_sensitivity = cmd.X.real;
            }
            if (cmd.Y.set) {
                y_axis.homing_sensitivity = cmd.Y.real;
            }
#endif
#ifdef HAS_Z_AXIS
            if (cmd.Z.set) {
                z_axis.homing_sensitivity = cmd.Z.real;
            }
#endif
            printf("> ");
#ifdef HAS_XY_AXES
            printf("X:%u ", x_axis.homing_sensitivity);
            printf("Y:%u ", y_axis.homing_sensitivity);
#endif
#ifdef HAS_Z_AXIS
            printf("Z:%u ", z_axis.homing_sensitivity);
#endif
            printf("\n");
        } break;

        // M997 firmware update
        // https://marlinfw.org/docs/gcode/M997.html
        case 997: {
            reset_usb_boot(0, 0);
        } break;

        default:
            printf("! Unknown command M%i\n", cmd.M.real);
            break;
    }
}
