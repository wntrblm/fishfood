#pragma once

#include "drivers/tmc2209_helper.h"
#include "config/pins.h"

/*
    Configuration for the step timer
*/
#define STEP_INTERVAL_US 20

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

#define X_REVERSED 0
// TODO: This value needs to be empirically validated.
#define X_RUN_CURRENT 1.0f
#define X_HOLD_CURRENT_MULTIPLIER 0.5f
#define X_DEFAULT_VELOCITY_MM_S 400.0f
#define X_DEFAULT_ACCELERATION_MM_S2 8000.0f
// Note: steps/mm is dependent on the microsteps, if you change those this
// will also need to be updated.
#define X_STEPS_PER_MM 160.0f
// Sensorless homing stall detection threshold
// 0 to 255, higher is more sensitive
#define X_HOMING_SENSITIVITY 100
#define X_HOMING_VELOCITY_MM_S 400.0f
#define X_HOMING_ACCELERATION_MM_S2 20000.0f
#define X_HOMING_DISTANCE_MM 500.0f
#define X_HOMING_BOUNCE_MM 20.0f
#define X_HOMING_DIR -1

#ifdef STARFISH
#define HAS_X_AXIS 1
#define X_TMC (tmc0)
#define X_STEPPER (stepper0)
#define X_PIN_DIR PIN_M0_DIR
#define X_PIN_DIAG PIN_M0_DIAG
#define X_PIN_STEP PIN_M0_STEP
#define X_PIN_EN PIN_M0_EN
#endif

#define Y_REVERSED 1
// TODO: This value needs to be empirically validated.
#define Y_RUN_CURRENT 1.0f
#define Y_HOLD_CURRENT_MULTIPLIER 0.3f
#define Y_DEFAULT_VELOCITY_MM_S 400.0f
#define Y_DEFAULT_ACCELERATION_MM_S2 8000.0f
// Note: steps/mm is dependent on the microsteps, if you change those this
// will also need to be updated.
#define Y_STEPS_PER_MM 160.0f
// Sensorless homing stall detection threshold
// 0 to 255, higher is more sensitive
#define Y_HOMING_SENSITIVITY 150
#define Y_HOMING_VELOCITY_MM_S 400.0f
#define Y_HOMING_ACCELERATION_MM_S2 20000.0f
#define Y_HOMING_DISTANCE_MM 500.0f
#define Y_HOMING_BOUNCE_MM 20.0f
#define Y_HOMING_DIR -1

#ifdef STARFISH
#define HAS_Y_AXIS 1
#define Y1_TMC (tmc1)
#define Y1_STEPPER (stepper1)
#define Y1_PIN_DIR PIN_M1_DIR
#define Y1_PIN_DIAG PIN_M1_DIAG
#define Y1_PIN_STEP PIN_M1_STEP
#define Y1_PIN_EN PIN_M1_EN
#define Y2_TMC (tmc2)
#define Y2_STEPPER (stepper2)
#define Y2_PIN_DIR PIN_M2_DIR
#define Y2_PIN_DIAG PIN_M2_DIAG
#define Y2_PIN_STEP PIN_M2_STEP
#define Y2_PIN_EN PIN_M2_EN
#endif

// Based on testing with my own Z-Axis motor, 0.7A is a good balance between
// heat dissipated and hold torque. It's important that the Z axis' hold torque
// is high enough to not lose steps when the spring-loaded tip of the nozzle
// is compressed.
#define Z_REVERSED 0
#define Z_RUN_CURRENT 0.7f
#define Z_HOLD_CURRENT_MULTIPLIER 0.8f
#define Z_DEFAULT_VELOCITY_MM_S 150.0f
#define Z_DEFAULT_ACCELERATION_MM_S2 8000.0f
#define Z_STEPS_PER_MM 160.0f
#define Z_HOMING_SENSITIVITY 100
#define Z_HOMING_VELOCITY_MM_S 150.0f
#define Z_HOMING_ACCELERATION_MM_S2 20000.0f
#define Z_HOMING_DISTANCE_MM 100.0f
#define Z_HOMING_BOUNCE_MM 20.0f
#define Z_HOMING_DIR -1

#ifdef JELLYFISH
#define HAS_Z_AXIS 1
#define Z_TMC (tmc1)
#define Z_STEPPER (stepper1)
#define Z_PIN_DIR PIN_M1_DIR
#define Z_PIN_DIAG PIN_M1_DIAG
#define Z_PIN_STEP PIN_M1_STEP
#define Z_PIN_EN PIN_M1_EN
#endif

/*
    Configuration for the rotational axes- the left (A) & right (B) motor.
*/

#define A_RUN_CURRENT 0.2f
#define A_HOLD_CURRENT_MULTIPLIER 0.5f
#define A_STEPS_PER_DEG 17.778f

#ifdef JELLYFISH
#define HAS_A_AXIS 1
#define A_TMC (tmc0)
#define A_STEPPER (stepper0)
#define A_PIN_DIR PIN_M0_DIR
#define A_PIN_DIAG PIN_M0_DIAG
#define A_PIN_STEP PIN_M0_STEP
#define A_PIN_EN PIN_M0_EN
#endif

#define B_RUN_CURRENT 0.2f
#define B_HOLD_CURRENT_MULTIPLIER 0.5f
#define B_STEPS_PER_DEG 17.778f

#ifdef JELLYFISH
#define HAS_B_AXIS 1
#define B_TMC (tmc2)
#define B_STEPPER (stepper2)
#define B_PIN_DIR PIN_M2_DIR
#define B_PIN_DIAG PIN_M2_DIAG
#define B_PIN_STEP PIN_M2_STEP
#define B_PIN_EN PIN_M2_EN
#endif
