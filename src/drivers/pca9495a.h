/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

/*
    Driver for the PCA9495A I2C multiplexer.

    Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9545A_45B_45C.pdf
*/

#include "hardware/i2c.h"
#include <stdint.h>

// Allows enabling/disabling multiple channels. Channels is a 4-bit bitmap that controls
// the channels.
inline static int pca9495a_set_channels(i2c_inst_t* i2c, uint8_t pca_addr, uint8_t channels, uint timeout_us) {
    uint8_t buf[1] = {channels};
    return i2c_write_timeout_us(i2c, pca_addr, buf, 1, false, timeout_us);
}

// Switches to a single channel (0-3)
inline static int pca9495a_switch_channel(i2c_inst_t* i2c, uint8_t pca_addr, uint8_t channel, uint timeout_us) {
    return pca9495a_set_channels(i2c, pca_addr, 1 << channel, timeout_us);
}
