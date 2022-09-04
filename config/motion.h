#pragma once

#include "drivers/tmc2209_helper.h"

/*
    Common configuration for all TMC2209 drivers.
*/

// Candela uses the internal VREF
#define TMC_EXTERNAL_VREF 0
// Candela has external RDSon sense resistors
#define TMC_INTERNAL_RSENSE 0
// Rsense value in Ohms
#define TMC_RSENSE 0.220f
// Vsense option (0 = 325mV, 1 = 180mV)
#define TMC_VSENSE 0
// Use 32 microsteps per full step
#define TMC_MICROSTEPS 32
// Use microstep interpolation
#define TMC_INTERPOLATION 1
// How long after stopping the motor before it lowers to hold
// current, can be between 0 and 5.6s.
#define TMC_HOLD_TIME 3.0f

/*
    Configuration for the Z axis
*/

#define Z_RUN_CURRENT 0.5f
#define Z_HOLD_CURRENT_MULTIPLIER 0.8f
#define Z_DEFAULT_VELOCITY_MM_S 150.0f
#define Z_DEFAULT_ACCELERATION_MM_S2 3000.0f
#define Z_STEPS_PER_MM 160.0f
// Sensorless homing stall detection threshold
// 0 to 255, higher is more sensitive
#define Z_HOMING_SENSITIVITY 50
#define Z_HOMING_VELOCITY_MM_S 160.0f
#define Z_HOMING_ACCELERATION_MM_S2 15000.0f
#define Z_HOMING_DISTANCE_MM 100.0f
#define Z_HOMING_BOUNCE_MM 20.0f
#define Z_HOMING_DIR -1

/*
    Configuration for the left/right motors (axes A & B)
*/

#define A_RUN_CURRENT 0.2f
#define A_HOLD_CURRENT_MULTIPLIER 0.5f
#define A_STEPS_PER_DEG 17.778f

#define B_RUN_CURRENT 0.2f
#define B_HOLD_CURRENT_MULTIPLIER 0.5f
#define B_STEPS_PER_DEG 17.778f
