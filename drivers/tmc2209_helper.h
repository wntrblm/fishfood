#pragma once

#include "drivers/tmc2209.h"

bool TMC2209_write_config(struct TMC2209* tmc, uint32_t enable_pin);
