#pragma once

/*
    Common configuration for all TMC2209 drivers.
*/

// Candela uses the internal VREF
#define CONFIG_TMC_EXTERNAL_VREF 0
// Candela has external RDSon sense resistors
#define CONFIG_TMC_INTERNAL_RSENSE 0
// Rsense value, in Ohms
#define CONFIG_TMC_RSENSE 0.220f
// Vsense option (0 = 325mV, 1 = 180mV)
// TODO: Revisit this
#define CONFIG_TMC_VSENSE 0
// Use 16 microsteps
#define CONFIG_TMC_MICROSTEPS 32
// Use microstep interpolation
#define CONFIG_TMC_INTERPOLATION 1
// Run current (Amps RMS)
#define CONFIG_TMC_RUN_CURRENT 1.0
// Hold current (Amps RMS)
#define CONFIG_TMC_HOLD_CURRENT 1.0
// How long after stopping the motor before it lowers to hold current
// 0 to around 5.6s
#define CONFIG_TMC_HOLD_TIME 3.0
