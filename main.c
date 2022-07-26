#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "tmc2209.h"
#include "tmc_config.h"
#include "tmc_uart.h"
#include <stdio.h>

static struct TMC2209 tmc_left;
static struct TMC2209 tmc_right;
static struct TMC2209 tmc_z;

static repeating_timer_t step_timer;
static repeating_timer_t tmc_timer;
static bool step_value = false;
static volatile uint32_t steps = 0;
static volatile bool dir = false;

bool step_timer_callback(repeating_timer_t* rt) {
    if(steps == 0) return true;
    gpio_put(PIN_M0_STEP, step_value);
    step_value = !step_value;
    steps--;
    return true;
}

void diag_rise_callback(uint gpio, uint32_t events) {
    steps = 0;
    dir = !dir;
    gpio_put(PIN_M0_DIR, dir);
    printf("!!! External interrupt GPIO %d: %u\n", gpio, events);
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
    // while (!stdio_usb_connected()) {}

    printf("Starting UART...\n");
    uart_init(uart1, 115200);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);

    printf("Starting TMC drivers...\n");
    TMC2209_init(&tmc_left, uart1, 0, tmc_uart_read_write);
    if (!tmc_config(&tmc_left, PIN_M0_EN)) {
        printf("Error configuring tmc_left!");
        return 0;
    }

    printf("Configuring DIAG interrupt...\n");
    gpio_set_irq_enabled_with_callback(PIN_M0_DIAG, GPIO_IRQ_EDGE_RISE, true, &diag_rise_callback);

    printf("Starting steppers...\n");
    add_repeating_timer_us(-150, step_timer_callback, NULL, &step_timer);

    while (1) {
        enum TMC2209_read_result result;
        printf(".");
        sleep_ms(200);
        steps = 50 * 32;

        // printf("- Checking tmc_left status...\n");

        // uint32_t drvstatus;
        // result = TMC2209_read(&tmc_left, TMC2209_DRVSTATUS, &drvstatus);
        // TMC2209_print_DRVSTATUS(drvstatus);

        // uint32_t sg_result;
        // result = TMC2209_read(&tmc_left, TMC2209_SG_RESULT, &sg_result);
        // printf("SG_RESULT: %u / 510\n", sg_result);
    }
}
