#pragma once

#define Z_DEFAULT_VELOCITY_MM_S 150.0f
#define Z_DEFAULT_ACCELERATION_MM_S2 3000.0f
#define Z_STEPS_PER_MM 160
#define Z_MM_PER_STEP ((1.0f / Z_STEPS_PER_MM))
// Sensorless homing stall detection threshold
// 0 to 255, higher is more sensitive
#define Z_HOMING_SENSITIVITY 40
#define Z_HOMING_VELOCITY_MM_S 160.0f
#define Z_HOMING_ACCELERATION_MM_S2 15000.0f
#define Z_HOMING_DISTANCE_MM 100.0f
#define Z_HOMING_BOUNCE_MM 20.0f
#define Z_HOMING_DIR -1

#define LR_STEPS_PER_DEG 17.778f
#define LR_DEGS_PER_STEP ((1.0f / LR_STEPS_PER_DEG))
