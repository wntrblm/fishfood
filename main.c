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
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "i2c_commands.h"
#include "littleg/littleg.h"
#include "machine.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

#define NUM_PIXELS 8
static uint8_t pixels[3 * NUM_PIXELS];

static struct I2CCommandsState i2c_commands_state;
static struct Machine machine;

static repeating_timer_t step_timer;

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

    // Wait for USB connection before continuing.
    while (!stdio_usb_connected()) {}

    printf("| Starting I2C peripheral bus...");
    i2c_init(PERIPH_I2C_INST, PERIPH_I2C_SPEED);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
    i2c_commands_init(&i2c_commands_state);

    printf("| Starting TMC UART...\n");
    uart_init(TMC_UART_INST, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    printf("| Configuring steppers...\n");
    Machine_init(&machine);
    Machine_setup(&machine);

    printf("| Enabling steppers...\n");
    Machine_enable_steppers(&machine);

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

static int64_t step_timer_callback(alarm_id_t id, void* user_data) { return Machine_step(&machine); }

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
                float vel_mm_s = lilg_Decimal_to_float(LILG_FIELD(cmd, F)) / 60.0f;
                Machine_set_linear_velocity(&machine, vel_mm_s);
            }

#ifdef HAS_XY_AXES
            Machine_coordinated_xy_move(&machine, cmd);
#endif
            Machine_basic_move(&machine, cmd);
            Machine_wait_for_moves_to_finish(&machine);
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

        default:
            printf("! Unknown command G%i\n", cmd.G.real);
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

        // M114 get current position
        // https://marlinfw.org/docs/gcode/M114.html
        case 114: {
            Machine_report_position(&machine);
        } break;

        // M115 get firmware info
        // https://marlinfw.org/docs/gcode/M115.html
        case 115: {
            printf("> firmware_name:Picostep\n");
        } break;

        // M122 TMC debugging
        // https://marlinfw.org/docs/gcode/M122.html
        case 122: {
            Machine_report_tmc_info(&machine);
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
            if (LILG_FIELD(cmd, T).set) {
                float accel = lilg_Decimal_to_float(LILG_FIELD(cmd, T));
                Machine_set_linear_acceleration(&machine, accel);
                printf("> T:%0.2f mm/s^2\n", accel);
            }
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

        default:
            printf("! Unknown command M%i\n", cmd.M.real);
            break;
    }
}
