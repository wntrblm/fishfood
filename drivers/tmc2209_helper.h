#pragma once

#include "drivers/tmc2209.h"
#include <math.h>

#define _TMC2209_MIN(a, b) (a < b ? a : b)
#define _TMC2209_MAX(a, b) (a > b ? a : b)

#define TMC2209_CLK_FREQ 12000000
#define TMC2209_CLK_PERIOD (1.0f / (float)(TMC2209_CLK_FREQ))

/*
    Get the Vref for a given vsense setting
    From datasheet section 20.2
*/
#define TMC2209_VSENSE_VOLTAGE(vsense) (vsense == 0 ? 0.325f : 0.180f)

/*
    Converting between current (in Amps RMS) and current scale (IRUN/IHOLD)
    using  sense resistor value (in Ohms), and vsense setting (0 or 1).
    From datasheet section 93
*/
#define TMC2209_CS_TO_RMS(rsense, vsense, cs)                                                                          \
    ((((float)(cs) + 1.0f) / 32.0f) * (TMC2209_VSENSE_VOLTAGE(vsense) / (rsense + 0.02f)) * 0.7071f)
#define TMC2209_RMS_TO_CS(rsense, vsense, i)                                                                           \
    _TMC2209_MAX(                                                                                                      \
        0,                                                                                                             \
        _TMC2209_MIN(                                                                                                  \
            31, (roundf(((i * 1.4142f * (rsense + 0.02f) / TMC2209_VSENSE_VOLTAGE(vsense) * 32.0f) - 1.0f)))))

/*
    Converting between time (in seconds) and TPOWERDOWN value
*/
#define TMC2209_TPOWERDOWN_TO_S(val) ((float)(val * (2 << 17)) * TMC2209_CLK_PERIOD)
#define TMC2209_S_TO_TPOWERDOWN(s) (((uint32_t)((float)(s) / TMC2209_CLK_PERIOD) / (2 << 17)) & 0xFF)

bool TMC2209_write_config(struct TMC2209* tmc, uint32_t enable_pin);

void TMC2209_print_GCONF(uint32_t gconf);

void TMC2209_print_CHOPCONF(uint32_t chopconf);

void TMC2209_print_PWMCONF(uint32_t pwmconf);

void TMC2209_print_DRVSTATUS(uint32_t drvstatus);
