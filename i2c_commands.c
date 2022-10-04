#include <stdio.h>
#include "config/pins.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "wntr_pack.h"
#include "i2c_commands.h"

/*
    Public methods
*/

void i2c_commands_init(struct I2CCommandsState* s) {
    s->_addr = 0;
    s->_idx = 0;

    i2c_init(PERIPH_I2C_INST, PERIPH_I2C_SPEED);
    gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
};

void i2c_commands_m260_send(struct I2CCommandsState* s, const struct lilg_Command cmd) {
    if (LILG_FIELD(cmd, A).set) {
        s->_addr = LILG_FIELD(cmd, A).real;
        printf("> i2c address:0x%02X\n", s->_addr);
    }

    if (LILG_FIELD(cmd, R).set) {
        s->_idx = 0;
        printf("> i2c buffer reset\n");
    }

    if (LILG_FIELD(cmd, B).set) {
        if (s->_idx == PERIPH_I2C_BUF_LEN - 1) {
            printf("! i2c data buffer full\n");
            return;
        }

        uint8_t byte = LILG_FIELD(cmd, B).real;
        s->_data[s->_idx] = byte;
        printf("> i2c buffer[%u]: %u\n", s->_idx, byte);
        s->_idx++;
    }

    if (LILG_FIELD(cmd, S).set) {
        printf("> i2c sending %u bytes to %u...", s->_idx, s->_addr);
        int result = i2c_write_timeout_us(PERIPH_I2C_INST, s->_addr, s->_data, s->_idx, false, PERIPH_I2C_TIMEOUT);

        s->_idx = 0;

        if (result == PICO_ERROR_GENERIC) {
            printf("\n! Failed, device not present or not responding.\n");
            return;
        } else if (result == PICO_ERROR_TIMEOUT) {
            printf("\n! Failed, timeout exceeded.\n");
            return;
        } else {
            printf(" done.\n");
            return;
        }
    }
}

void i2c_commands_m261_request(struct I2CCommandsState* s, const struct lilg_Command cmd) {
    uint8_t addr = LILG_FIELD(cmd, A).set ? LILG_FIELD(cmd, A).real : s->_addr;
    size_t count = LILG_FIELD(cmd, B).set ? LILG_FIELD(cmd, B).real : 1;
    uint8_t style = LILG_FIELD(cmd, S).real;

    if (addr == 0) {
        printf("! No address specified in the A field\n");
        return;
    }
    if (count < 1) {
        printf("! Can not read zero bytes, set the B field to >= 1\n");
        return;
    }
    if (count > PERIPH_I2C_BUF_LEN) {
        printf("! Can not read more than %u bytes\n", PERIPH_I2C_BUF_LEN);
        return;
    }

    int result = i2c_read_timeout_us(PERIPH_I2C_INST, addr, s->_data, count, false, PERIPH_I2C_TIMEOUT);

    if (result == PICO_ERROR_GENERIC) {
        printf("! Failed, device 0x%2X not present or not responding\n", addr);
        return;
    } else if (result == PICO_ERROR_TIMEOUT) {
        printf("! Failed, timeout exceeded\n");
        return;
    } else if (result < 0) {
        printf("! Failed, unknown error %i occurred\n", result);
        return;
    }

    size_t bytes_read = result;

    printf("> i2c reply: from:%u bytes:%u data:", addr, bytes_read);
    switch (style) {
        case 1: {
            for (size_t i = 0; i < bytes_read; i++) { printf("%02X ", s->_data[i]); }
        } break;
        case 2: {
            switch (bytes_read) {
                case 1:
                    printf("%u", s->_data[0]);
                    break;
                case 2:
                    printf("%u", WNTR_UNPACK_16(s->_data, 0));
                    break;
                case 4:
                    printf("%u", WNTR_UNPACK_32(s->_data, 0));
                    break;
                default:
                    printf("! Wrong number of bytes for integer reply: %u", bytes_read);
                    return;
            }
        } break;

        case 0:
        default: {
            for (size_t i = 0; i < bytes_read; i++) { printf("%c", s->_data[i]); }
        } break;
    }

    printf("\n");
}


void i2c_commands_m262_scan() {
    for (uint8_t addr = 0; addr < 127; addr++) {
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            continue;
        }

        uint8_t out[] = {0};
        int result = i2c_read_timeout_us(PERIPH_I2C_INST, addr, out, 1, false, PERIPH_I2C_TIMEOUT);

        if (result < 0) {
            printf("> 0x%2X: no response\n", addr);
        } else {
            printf("> 0x%2X: replied 0x%2X\n", addr, out[0]);
        }
    }
}
