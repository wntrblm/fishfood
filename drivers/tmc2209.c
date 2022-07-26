#include "tmc2209.h"
#include <stdio.h>

/* Contants and macros */

#define TMC_SYNC_BTYE 0x05
#define TMC_WRITE_ADDR 0x80

/* Private data and methods */

/* Public methods */

void TMC2209_init(
    struct TMC2209* tmc, void* uart, uint8_t uart_address, TMC2209_uart_send_receive_func uart_send_receive) {
    tmc->uart = uart;
    tmc->uart_address = uart_address;
    tmc->uart_send_receive = uart_send_receive;
}

enum TMC2209_read_result TMC2209_read(struct TMC2209* tmc, uint8_t register_addr, uint32_t* out) {
    uint8_t read_datagram[] = {TMC_SYNC_BTYE, tmc->uart_address, register_addr, 0};
    read_datagram[3] = TMC2209_CRC8(read_datagram, 3);

    uint8_t reply_datagram[8] = {};

    tmc->uart_send_receive(tmc, read_datagram, 4, reply_datagram, 8);

    if (reply_datagram[0] != 0x05) {
        return TMC_READ_BAD_SYNC;
    }
    if (reply_datagram[1] != 0xFF) {
        return TMC_READ_BAD_ADDRESS;
    }
    if (reply_datagram[2] != register_addr) {
        return TMC_READ_BAD_REGISTER;
    }
    if (reply_datagram[7] != TMC2209_CRC8(reply_datagram, 7)) {
        return TMC_READ_BAD_CRC;
    }

    (*out) = ((uint32_t)(reply_datagram[3]) << 24) | ((uint32_t)(reply_datagram[4]) << 16) |
             ((uint32_t)(reply_datagram[5]) << 8) | (uint32_t)(reply_datagram[6]);

    return TMC_READ_OK;
}

void TMC2209_write(struct TMC2209* tmc, uint8_t register_addr, uint32_t value) {
    uint8_t datagram[8] = {
        TMC_SYNC_BTYE,
        tmc->uart_address,
        register_addr | TMC_WRITE_ADDR,
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value,
        0};

    datagram[7] = TMC2209_CRC8(datagram, 7);

    tmc->uart_send_receive(tmc, datagram, 8, NULL, 0);
}

uint8_t TMC2209_CRC8(uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t byte_n = 0; byte_n < len; byte_n++) {
        uint8_t byte = data[byte_n];
        for (size_t bit_n = 0; bit_n < 8; bit_n++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            byte = byte >> 1;
        }
    }
    return crc;
}
