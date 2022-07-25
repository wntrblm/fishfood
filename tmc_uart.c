#include "tmc_uart.h"
#include "hardware/uart.h"
#include <stdio.h>

static inline void print_hex_array(uint8_t* data, size_t len) {
    printf("[");
    for (size_t i = 0; i < len; i++) { printf("0x%02X ", data[i]); }
    printf("]\n");
}

void tmc_uart_read_write(
    struct TMC2209* tmc, uint8_t* send_buf, size_t send_len, uint8_t* receive_buf, size_t receive_len) {
    printf("> TMC UART send: %i, receive: %i\n", send_len, receive_len);
    printf("Write: ");
    print_hex_array(send_buf, send_len);

    // clear any existing rx bytes
    while (uart_is_readable(uart1)) { uart_getc(uart1); }

    uart_write_blocking(uart1, send_buf, send_len);

    // clear echoed rx bytes
    while (uart_is_readable_within_us(uart1, 100)) { uart_getc(uart1); }

    if (receive_len > 0) {
        uint8_t byte = 0;
        size_t n = 1;

        // Read until the sync byte is seen.
        while (true) {
            byte = uart_getc(uart1);
            if (byte == 0x05) {
                receive_buf[0] = byte;
                break;
            }
        }

        while (n < receive_len) {
            receive_buf[n] = uart_getc(uart1);
            n++;
        }

        printf("Read %u/%u bytes: ", n, receive_len);
        print_hex_array(receive_buf, n);
    }
}
