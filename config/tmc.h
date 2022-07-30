#pragma once

/*
    Common configuration for all TMC2209 drivers.
*/

// Candela uses the internal VREF
#define CONFIG_TMC_EXTERNAL_VREF 0
// Candela has external RDSon sense resistors
#define CONFIG_TMC_INTERNAL_RSENSE 0
// Use 32 microsteps
#define CONFIG_TMC_MICROSTEPS 32
// Use microstep interpolation
#define CONFIG_TMC_INTERPOLATION 1
// Run current (0 to 31)
#define CONFIG_TMC_RUN_CURRENT 31
// Hold current (0 to 31)
#define CONFIG_TMC_HOLD_CURRENT 20
// Stall detection threshold (0 to 255), higher is more sensitive.
#define CONFIG_TMC_STALL_THRESHOLD 50
