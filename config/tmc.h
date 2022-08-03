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
#define CONFIG_TMC_MICROSTEPS 16
// Use microstep interpolation
#define CONFIG_TMC_INTERPOLATION 1
// Run current (0 to 31)
#define CONFIG_TMC_RUN_CURRENT 31
// Hold current (0 to 31)
#define CONFIG_TMC_HOLD_CURRENT 25
// How long after stopping the motor before it lowers to hold current
// 0 - 255 corresponding to 0 to 5.6s
#define CONFIG_TMC_HOLD_TIME 128
// Default homing stall detection threshold
// 0 to 255, higher is more sensitive
#define CONFIG_TMC_STALL_THRESHOLD 80
