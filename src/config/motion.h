/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

#include "config/pins.h"
#include "drivers/tmc2209_helper.h"

/*
    Common configuration for all TMC2209 drivers.
*/

// Jellyfish/Starfish uses the internal VREF
#define TMC_EXTERNAL_VREF 0
// Jellyfish/Starfish has external RDSon sense resistors
#define TMC_INTERNAL_RSENSE 0
// Rsense value in Ohms - Jellfish is 220mOhm, Starfish is 120mOhm
#ifdef JELLYFISH
#define TMC_RSENSE 0.220f
#endif
#ifdef STARFISH
#define TMC_RSENSE 0.120f
#endif
// Vsense option (0 = 325mV, 1 = 180mV)
#define TMC_VSENSE 0
// Use 32 microsteps per full step
#define TMC_MICROSTEPS 32
// Use microstep interpolation
#define TMC_INTERPOLATION 1
// How long after stopping the motor before it lowers to hold
// current, can be between 0 and 5.6s.
#define TMC_HOLD_TIME 3.0f

// TODO: All linear axes need soft limits.

/*
    Configuration for the linear axes (X, Y, and Z).
*/

#ifdef STARFISH
#define HAS_XY_AXES 1

#define X_STEPPER 0
#define X_REVERSED 0
// Note: this value likely needs tweaking depending on the exact motor you're using.
#define X_RUN_CURRENT 1.0f
#define X_HOLD_CURRENT_MULTIPLIER 0.5f

#define X_DEFAULT_VELOCITY_MM_S 300.0f
#define X_DEFAULT_ACCELERATION_MM_S2 1000.0f
// Note: steps/mm is dependent on the microsteps, if you change those this
// will also need to be updated.
#define X_STEPS_PER_MM 160.0f
// Sensorless homing stall detection threshold
// 0 to 255, higher is more sensitive
#define X_HOMING_SENSITIVITY 100
#define X_HOMING_VELOCITY_MM_S 200.0f
#define X_HOMING_ACCELERATION_MM_S2 5000.0f
#define X_HOMING_DISTANCE_MM 500.0f
#define X_HOMING_BOUNCE_MM 20.0f
#define X_HOMING_DIR -1

#define Y_STEPPER 1
#define Y_REVERSED 1
// Note: this value likely needs tweaking depending on the exact motor you're using.
#define Y_RUN_CURRENT 1.0f
#define Y_HOLD_CURRENT_MULTIPLIER 0.3f

#define Y2_STEPPER 2
#define Y2_REVERSED 0
#define Y2_RUN_CURRENT Y_RUN_CURRENT
#define Y2_HOLD_CURRENT_MULTIPLIER Y_HOLD_CURRENT_MULTIPLIER

#define Y_DEFAULT_VELOCITY_MM_S X_DEFAULT_VELOCITY_MM_S
#define Y_DEFAULT_ACCELERATION_MM_S2 X_DEFAULT_ACCELERATION_MM_S2
// Note: steps/mm is dependent on the microsteps, if you change those this
// will also need to be updated.
#define Y_STEPS_PER_MM 160.0f
// Sensorless homing stall detection threshold
// 0 to 255, higher is more sensitive
#define Y_HOMING_SENSITIVITY 150
#define Y_HOMING_VELOCITY_MM_S X_HOMING_VELOCITY_MM_S
#define Y_HOMING_ACCELERATION_MM_S2 X_HOMING_ACCELERATION_MM_S2
#define Y_HOMING_DISTANCE_MM 500.0f
#define Y_HOMING_BOUNCE_MM X_HOMING_BOUNCE_MM
#define Y_HOMING_DIR -1
#endif

#ifdef JELLYFISH
#define HAS_Z_AXIS
#define Z_STEPPER 1
#define Z_REVERSED 1
// Note: this value likely needs tweaking depending on the exact motor you're using.
// Based on testing with my own Z-Axis motor, 0.6A is a good balance between
// heat dissipated and hold torque. It's important that the Z axis' hold torque
// is high enough to not lose steps when the spring-loaded tip of the nozzle
// is compressed.
#define Z_RUN_CURRENT 0.6f
#define Z_HOLD_CURRENT_MULTIPLIER 0.75f
#define Z_DEFAULT_VELOCITY_MM_S 200.0f
#define Z_DEFAULT_ACCELERATION_MM_S2 1000.0f
#define Z_STEPS_PER_MM 160.0f
#define Z_HOMING_SENSITIVITY 130
#define Z_HOMING_VELOCITY_MM_S 50.0f
#define Z_HOMING_ACCELERATION_MM_S2 2000.0f
#define Z_HOMING_DISTANCE_MM 100.0f
#define Z_HOMING_BOUNCE_MM 5.0f
#define Z_HOMING_DIR -1
#define Z_HOME_ENDSTOP PIN_IN_2
#endif

/*
    Configuration for the rotational axes- the left (A) & right (B) motor.
*/

#ifdef JELLYFISH
#define HAS_A_AXIS
#define A_STEPPER 0
#define A_REVERSED 0
#define A_RUN_CURRENT 0.2f
#define A_HOLD_CURRENT_MULTIPLIER 0.5f
#define A_STEPS_PER_DEG 17.778f

#define HAS_B_AXIS
#define B_STEPPER 2
#define B_REVERSED 0
#define B_RUN_CURRENT 0.2f
#define B_HOLD_CURRENT_MULTIPLIER 0.5f
#define B_STEPS_PER_DEG 17.778f
#endif
