#include "tmc_uart.h"
#include "hardware/uart.h"
#include <string.h>

void tmc_uart_read_write(
    struct TMC2209* tmc, uint8_t* send_buf, size_t send_len, uint8_t* receive_buf, size_t receive_len) {

    // clear any existing rx bytes
    while (uart_is_readable_within_us(tmc->uart, 100)) { uart_getc(tmc->uart); }

    uart_write_blocking(tmc->uart, send_buf, send_len);

    /*
        Here's where things get tricky. Since TX and RX are shared the bytes
        we just wrote are now echoed in the RX buffer. So we gotta read and
        discard them until we get to the reply. This does so by "looking ahead"
        to see if the reply sync and address bytes are received.
    */
    if (receive_len > 0) {
        uint8_t rx_bytes[64] = {};
        size_t rx_num = 0;
        bool seen_sync = false;
        uint8_t byte = 0;

        // Read until the sync byte is seen.
        while (true) {
            byte = uart_getc(tmc->uart);
            rx_bytes[rx_num] = byte;
            rx_num++;

            if (!seen_sync) {
                if (byte == 0x05) {
                    receive_buf[0] = byte;
                    seen_sync = true;
                    continue;
                }
            } else {
                // The address byte should be immediately after the
                // sync byte. If it's not, then start over.
                if (byte == 0xFF) {
                    receive_buf[1] = byte;
                    break;
                } else if (byte == 0x05) {
                    continue;
                } else {
                    // printf("Saw incorrect addr byte 0x%02x, resetting...\n", byte);
                    seen_sync = false;
                }
            }
        }

        // Now read the rest of the reply.
        for (size_t n = 2; n < receive_len; n++) { receive_buf[n] = uart_getc(tmc->uart); }
    }
}
