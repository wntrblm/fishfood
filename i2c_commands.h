#pragma once

#include "config/serial.h"
#include "littleg/littleg.h"

struct I2CCommandsState {
    uint8_t _addr;
    uint8_t _data[PERIPH_I2C_BUF_LEN];
    size_t _idx;
};

void i2c_commands_init(struct I2CCommandsState* s);
void i2c_commands_m260_send(struct I2CCommandsState* s, const struct lilg_Command cmd);
void i2c_commands_m261_request(struct I2CCommandsState* s, const struct lilg_Command cmd);
void i2c_commands_m262_scan();
