/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#include "i2c_commands.h"
#include "config/pins.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "libwinter/wntr_pack.h"
#include "report.h"

/*
    Public methods
*/

void i2c_commands_init(struct I2CCommandsState* s) {
    s->_addr = 0;
    s->_idx = 0;
};

void i2c_commands_m260_send(struct I2CCommandsState* s, const struct lilg_Command cmd) {
    if (LILG_FIELD(cmd, A).set) {
        s->_addr = LILG_FIELD(cmd, A).real;
        report_result_ln("i2c.address:0x%02X", s->_addr);
    }

    if (LILG_FIELD(cmd, R).set) {
        s->_idx = 0;
        report_result_ln("i2c.buffer reset");
    }

    if (LILG_FIELD(cmd, B).set) {
        if (s->_idx == PERIPH_I2C_BUF_LEN - 1) {
            report_error_ln("i2c.buffer full");
            return;
        }

        uint8_t byte = LILG_FIELD(cmd, B).real;
        s->_data[s->_idx] = byte;
        report_result_ln("i2c.buffer[%u]: %u", s->_idx, byte);
        s->_idx++;
    }

    if (LILG_FIELD(cmd, S).set) {
        report_info_ln("i2c sending %u bytes to %u...", s->_idx, s->_addr);
        int result = i2c_write_timeout_us(PERIPH_I2C_INST, s->_addr, s->_data, s->_idx, false, PERIPH_I2C_TIMEOUT);

        s->_idx = 0;

        if (result == PICO_ERROR_GENERIC) {
            report_error_ln("failed, device not present or not responding");
            return;
        } else if (result == PICO_ERROR_TIMEOUT) {
            report_error_ln("failed, timeout exceeded");
            return;
        } else {
            report_result_ln("i2c message sent");
            return;
        }
    }
}

void i2c_commands_m261_request(struct I2CCommandsState* s, const struct lilg_Command cmd) {
    uint8_t addr = LILG_FIELD(cmd, A).set ? LILG_FIELD(cmd, A).real : s->_addr;
    size_t count = LILG_FIELD(cmd, B).set ? LILG_FIELD(cmd, B).real : 1;
    uint8_t style = LILG_FIELD(cmd, S).real;

    if (addr == 0) {
        report_error_ln("no address specified in the A field");
        return;
    }
    if (count < 1) {
        report_error_ln("can not read zero bytes, set the B field to >= 1");
        return;
    }
    if (count > PERIPH_I2C_BUF_LEN) {
        report_error_ln("can not read more than %u bytes", PERIPH_I2C_BUF_LEN);
        return;
    }

    int result = i2c_read_timeout_us(PERIPH_I2C_INST, addr, s->_data, count, false, PERIPH_I2C_TIMEOUT);

    if (result == PICO_ERROR_GENERIC) {
        report_error_ln("failed, device 0x%2X not present or not responding", addr);
        return;
    } else if (result == PICO_ERROR_TIMEOUT) {
        report_error_ln("failed, timeout exceeded");
        return;
    } else if (result < 0) {
        report_error_ln("failed, unknown error %i occurred\n", result);
        return;
    }

    size_t bytes_read = result;

    report_result("i2c.reply: from:%u bytes:%u data:", addr, bytes_read);

    switch (style) {
        case 1: {
            for (size_t i = 0; i < bytes_read; i++) { report_result("%02X ", s->_data[i]); }
        } break;
        case 2: {
            switch (bytes_read) {
                case 1:
                    report_result("%u", s->_data[0]);
                    break;
                case 2:
                    report_result("%u", WNTR_UNPACK_16(s->_data, 0));
                    break;
                case 4:
                    report_result("%u", WNTR_UNPACK_32(s->_data, 0));
                    break;
                default:
                    report_result_ln("");
                    report_error_ln("wrong number of bytes for integer reply: %u", bytes_read);
                    return;
            }
        } break;

        case 0:
        default: {
            for (size_t i = 0; i < bytes_read; i++) { report_result("%c", s->_data[i]); }
        } break;
    }

    report_result_ln("");
}

void i2c_commands_m262_scan() {
    for (uint8_t addr = 0; addr < 127; addr++) {
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            continue;
        }

        uint8_t out[] = {0};
        int result = i2c_read_timeout_us(PERIPH_I2C_INST, addr, out, 1, false, PERIPH_I2C_TIMEOUT);

        if (result < 0) {
            report_result_ln("0x%2X: no response", addr);
        } else {
            report_result_ln("0x%2X: reply:0x%2X", addr, out[0]);
        }
    }
}
