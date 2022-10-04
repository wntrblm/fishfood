#pragma once

/*
    Driver for the XGZP6857D vacuum sensor.

    Datasheet: https://www.cfsensor.com/static/upload/file/20220412/XGZP6857D%20Pressure%20Sensor%20Module%20V2.4.pdf
*/

#include <stdint.h>
#include "hardware/i2c.h"


int32_t XGZP6857D_read(i2c_inst_t *i2c, uint timeout_us);
