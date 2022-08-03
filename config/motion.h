#pragma once

#define Z_DEFAULT_VELOCITY_MM_PER_S 100.0f
#define Z_DEFAULT_ACCELERATION_MM_PER_S2 50.0f
#define Z_STEPS_PER_MM 160
#define Z_MM_PER_STEP ((1.0f / Z_STEPS_PER_MM))
#define Z_HOMING_DIR -1

#define LR_STEPS_PER_DEG 17.778f
#define LR_DEGS_PER_STEP ((1.0f / LR_STEPS_PER_DEG))