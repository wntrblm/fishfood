#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "tmc2209.h"
#include "tmc_config.h"
#include "tmc_uart.h"
#include <stdio.h>

const uint8_t PIN_CAM_LED = 12;
const uint8_t PIN_AUX_PIN = 13;
const uint8_t PIN_UART_TX = 8;
const uint8_t PIN_UART_RX = 9;
const uint8_t PIN_M0_DIR = 24;
const uint8_t PIN_M0_STEP = 25;
const uint8_t PIN_M0_DIAG = 26;
const uint8_t PIN_M0_EN = 27;
const uint8_t PIN_M1_DIR = 20;
const uint8_t PIN_M1_STEP = 21;
const uint8_t PIN_M1_DIAG = 22;
const uint8_t PIN_M1_EN = 23;
const uint8_t PIN_M2_DIR = 16;
const uint8_t PIN_M2_STEP = 17;
const uint8_t PIN_M2_DIAG = 18;
const uint8_t PIN_M2_EN = 19;

static struct TMC2209 tmc_left;
static struct TMC2209 tmc_right;
static struct TMC2209 tmc_z;

static repeating_timer_t step_timer;
static repeating_timer_t tmc_timer;
static bool step_value = false;

bool step_timer_callback(repeating_timer_t* rt) {
    gpio_put(PIN_M0_STEP, step_value);
    step_value = !step_value;
    return true;
}

int main() {
    stdio_init_all();

    gpio_init(PIN_M0_DIR);
    gpio_set_dir(PIN_M0_DIR, GPIO_OUT);
    gpio_init(PIN_M0_STEP);
    gpio_set_dir(PIN_M0_STEP, GPIO_OUT);
    gpio_init(PIN_M0_EN);
    gpio_set_dir(PIN_M0_EN, GPIO_OUT);

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
    if(!tmc_config(&tmc_left, PIN_M0_EN)) {
        printf("Error configuring tmc_left!");
        return 0;
    }

    printf("Starting steppers...\n");
    add_repeating_timer_us(-200, step_timer_callback, NULL, &step_timer);

    while (1) {
        printf(".");
        sleep_ms(5000);

        printf("- Checking tmc_left status...\n");

        uint32_t drvstatus;
        enum TMC2209_read_result result = TMC2209_read(&tmc_left, TMC2209_DRVSTATUS, &drvstatus);
        TMC2209_print_DRVSTATUS(drvstatus);
    }
}
