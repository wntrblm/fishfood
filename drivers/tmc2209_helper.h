#pragma once

#include "drivers/tmc2209.h"

bool TMC2209_write_config(struct TMC2209* tmc, uint32_t enable_pin);

void TMC2209_print_GCONF(uint32_t gconf);

void TMC2209_print_CHOPCONF(uint32_t chopconf);

void TMC2209_print_PWMCONF(uint32_t pwmconf);

void TMC2209_print_DRVSTATUS(uint32_t drvstatus);
