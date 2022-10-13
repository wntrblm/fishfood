#pragma once

#include "drivers/tmc2209.h"
#include "drivers/tmc2209_helper.h"
#include "drivers/tmc_uart.h"
#include "littleg/littleg.h"
#include "motion/bresenham.h"
#include "motion/linear_axis.h"
#include "motion/rotational_axis.h"
#include "motion/stepper.h"

struct Machine {
    struct TMC2209 tmc[3];
    struct Stepper stepper[3];
    struct LinearAxis x;
    struct LinearAxis y;
    struct LinearAxis z;
    struct RotationalAxis a;
    struct RotationalAxis b;
    bool absolute_positioning;

    /* State */
    bool _is_coordinated_move;
    struct LinearAxis* _major_axis;
    struct LinearAxis* _minor_axis;
    struct Bresenham _bresenham;
};

void Machine_init(struct Machine* m);
void Machine_setup(struct Machine* m);
void Machine_enable_steppers(struct Machine* m);
void Machine_disable_steppers(struct Machine* m);
void Machine_set_linear_velocity(struct Machine* m, float vel_mm_s);
void Machine_set_linear_acceleration(struct Machine* m, float accel_mm_s2);
void Machine_set_motor_current(struct Machine* m, const struct lilg_Command cmd);
void Machine_set_homing_sensitivity(struct Machine* m, const struct lilg_Command cmd);
void Machine_home(struct Machine* m, bool x, bool y, bool z);
void Machine_move(struct Machine* m, const struct lilg_Command cmd);
void Machine_report_position(struct Machine* m);
void Machine_set_position(struct Machine* m, const struct lilg_Command cmd);
void Machine_report_tmc_info(struct Machine* m);
int64_t Machine_step(struct Machine* m);
