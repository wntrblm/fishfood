#pragma once

#include "drivers/tmc2209.h"
#include <math.h>


/*
    Get the Vref for a given vsense setting
    From datasheet section 20.2
*/
#define TMC_VSENSE_VOLTAGE(vsense) (vsense == 0 ? 0.325f : 0.180f)

/*
    Calculate current (in Amps RMS) from current scale (IRUN/IHOLD), sense
    resistor value (in Ohms), and vsense setting (0 or 1).
    From datasheet section 9
*/
#define TMC2209_CS_TO_RMS(rsense, vsense, cs) (((float(cs) + 1.0f) / 32.0f) * (TMC_VSENSE_VOLTAGE(vsense) / (rsense + 0.02f)) * 0.7071f)

/*
    Calculate current scale (IRUN/HOLD) from current (in Amps RMS), sense
    resistor value (in Ohms), and vsense setting (0 or 1)
*/
#define _TMC2209_MIN(a, b) (a < b ? a : b)
#define TMC2209_RMS_TO_CS(rsense, vsense, a) _TMC2209_MIN(32, (roundf((a / 0.7071f / TMC_VSENSE_VOLTAGE(vsense) / (rsense + 0.02f) * 32.0f) - 1.0f)))

bool TMC2209_write_config(struct TMC2209* tmc, uint32_t enable_pin);

void TMC2209_print_GCONF(uint32_t gconf);

void TMC2209_print_CHOPCONF(uint32_t chopconf);

void TMC2209_print_PWMCONF(uint32_t pwmconf);

void TMC2209_print_DRVSTATUS(uint32_t drvstatus);
