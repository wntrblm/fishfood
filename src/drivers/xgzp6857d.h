/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

/*
    Driver for the XGZP6857D vacuum sensor.

    Datasheet: https://www.cfsensor.com/static/upload/file/20220412/XGZP6857D%20Pressure%20Sensor%20Module%20V2.4.pdf
*/

#include "hardware/i2c.h"
#include <stdint.h>

int32_t XGZP6857D_read(i2c_inst_t* i2c, uint timeout_us);
