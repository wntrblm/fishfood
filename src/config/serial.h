/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#define TMC_UART_INST uart0
#define RS485_UART_INST uart1
#define RS485_BAUD 9600
#define PERIPH_I2C_INST i2c1
#define PERIPH_I2C_SPEED (100 * 1000)
#define PERIPH_I2C_BUF_LEN (64)
#define PERIPH_I2C_TIMEOUT (100000)
#define I2C_MUX_ADDR (0x58)

#ifdef STARFISH
#define HAS_RS485
#endif
