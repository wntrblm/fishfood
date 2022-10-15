#include "machine.h"
#include "config/motion.h"
#include "config/serial.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "report.h"

/*
    Macros
*/

#define PIN_M(n, f) PIN_M##n##_##f

#define INIT_STEPPER(number, LETTER)                                                                                   \
    Stepper_init(                                                                                                      \
        &(m->stepper[number]),                                                                                         \
        &(m->tmc[number]),                                                                                             \
        PIN_M(number, EN),                                                                                             \
        PIN_M(number, DIR),                                                                                            \
        PIN_M(number, STEP),                                                                                           \
        PIN_M(number, DIAG),                                                                                           \
        LETTER##_REVERSED,                                                                                             \
        LETTER##_RUN_CURRENT,                                                                                          \
        LETTER##_RUN_CURRENT* LETTER##_HOLD_CURRENT_MULTIPLIER);

#define INIT_LINEAR_AXIS(letter, LETTER)                                                                               \
    INIT_STEPPER(LETTER##_STEPPER, LETTER);                                                                            \
    LinearAxis_init(&(m->letter), #LETTER[0], &(m->stepper[LETTER##_STEPPER]));                                        \
    m->letter.steps_per_mm = LETTER##_STEPS_PER_MM;                                                                    \
    m->letter.velocity_mm_s = LETTER##_DEFAULT_VELOCITY_MM_S;                                                          \
    m->letter.acceleration_mm_s2 = LETTER##_DEFAULT_ACCELERATION_MM_S2;                                                \
    m->letter.homing_direction = LETTER##_HOMING_DIR;                                                                  \
    m->letter.homing_distance_mm = LETTER##_HOMING_DISTANCE_MM;                                                        \
    m->letter.homing_bounce_mm = LETTER##_HOMING_BOUNCE_MM;                                                            \
    m->letter.homing_velocity_mm_s = LETTER##_HOMING_VELOCITY_MM_S;                                                    \
    m->letter.homing_acceleration_mm_s2 = LETTER##_HOMING_ACCELERATION_MM_S2;                                          \
    m->letter.homing_sensitivity = LETTER##_HOMING_SENSITIVITY;

#define INIT_ROTATIONAL_AXIS(letter, LETTER)                                                                           \
    INIT_STEPPER(LETTER##_STEPPER, LETTER);                                                                            \
    RotationalAxis_init(&(m->letter), #LETTER[0], &(m->stepper[LETTER##_STEPPER]));                                    \
    m->letter.steps_per_deg = LETTER##_STEPS_PER_DEG;

/*
    Public functions
*/

void Machine_init(struct Machine* m) {
    m->absolute_positioning = true;
    m->_is_coordinated_move = false;

    TMC2209_init(&m->tmc[0], TMC_UART_INST, 0, tmc_uart_read_write);
    TMC2209_init(&m->tmc[1], TMC_UART_INST, 1, tmc_uart_read_write);
    // Note: This should be 2, but both Jellyfish & Starfish skip address 2.
    TMC2209_init(&m->tmc[2], TMC_UART_INST, 3, tmc_uart_read_write);

#ifdef HAS_XY_AXES
    INIT_LINEAR_AXIS(x, X);
    INIT_LINEAR_AXIS(y, Y);
    INIT_STEPPER(Y2_STEPPER, Y2);

    LinearAxis_setup_dual(&(m->y), &(m->stepper[Y2_STEPPER]));
#endif

#ifdef HAS_Z_AXIS
    INIT_LINEAR_AXIS(z, Z);
#endif

#ifdef HAS_A_AXIS
    INIT_ROTATIONAL_AXIS(a, A);
#endif

#ifdef HAS_B_AXIS
    INIT_ROTATIONAL_AXIS(b, B);
#endif
}

void Machine_setup(struct Machine* m) {
    Stepper_setup(&(m->stepper[0]));
    Stepper_setup(&(m->stepper[1]));
    Stepper_setup(&(m->stepper[2]));
}

void Machine_enable_steppers(struct Machine* m) {
    Stepper_enable(&(m->stepper[0]));
    Stepper_enable(&(m->stepper[1]));
    Stepper_enable(&(m->stepper[2]));
}

void Machine_disable_steppers(struct Machine* m) {
    Stepper_disable(&(m->stepper[0]));
    Stepper_disable(&(m->stepper[1]));
    Stepper_disable(&(m->stepper[2]));
}

void Machine_set_linear_velocity(struct Machine* m, float vel_mm_s) {
#ifdef HAS_XY_AXES
    m->x.velocity_mm_s = vel_mm_s;
    m->y.velocity_mm_s = vel_mm_s;
#endif
#ifdef HAS_Z_AXIS
    m->z.velocity_mm_s = vel_mm_s;
#endif
}

void Machine_set_linear_acceleration(struct Machine* m, float accel_mm_s2) {
#ifdef HAS_XY_AXES
    m->x.acceleration_mm_s2 = accel_mm_s2;
    m->y.acceleration_mm_s2 = accel_mm_s2;
#endif
#ifdef HAS_Z_AXIS
    m->z.acceleration_mm_s2 = accel_mm_s2;
#endif

    report_result_ln("T:%0.2f mm/s^2", (double)accel_mm_s2);
}

void Machine_set_motor_current(struct Machine* m, const struct lilg_Command cmd) {
#ifdef HAS_XY_AXES
    if (cmd.X.set) {
        float current = lilg_Decimal_to_float(cmd.X);
        TMC2209_set_current(m->x.stepper->tmc, current, current * X_HOLD_CURRENT_MULTIPLIER);
    }
    if (cmd.Y.set) {
        float current = lilg_Decimal_to_float(cmd.Y);
        TMC2209_set_current(m->y.stepper->tmc, current, current * Y_HOLD_CURRENT_MULTIPLIER);
        TMC2209_set_current(m->y.stepper2->tmc, current, current * Y_HOLD_CURRENT_MULTIPLIER);
    }
#endif
#ifdef HAS_Z_AXIS
    if (cmd.Z.set) {
        float current = lilg_Decimal_to_float(cmd.Z);
        TMC2209_set_current((m->z.stepper->tmc), current, current * Z_HOLD_CURRENT_MULTIPLIER);
    }
#endif
#ifdef HAS_A_AXIS
    if (LILG_FIELD(cmd, A).set) {
        float current = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
        TMC2209_set_current((m->a.stepper->tmc), current, current * A_HOLD_CURRENT_MULTIPLIER);
    }
#endif
#ifdef HAS_B_AXIS
    if (LILG_FIELD(cmd, B).set) {
        float current = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
        TMC2209_set_current((m->b.stepper->tmc), current, current * B_HOLD_CURRENT_MULTIPLIER);
    }
#endif
}

void Machine_set_homing_sensitivity(struct Machine* m, const struct lilg_Command cmd) {
#ifdef HAS_XY_AXES
    if (cmd.X.set) {
        m->x.homing_sensitivity = cmd.X.real;
    }
    report_result("X:%u ", m->x.homing_sensitivity);

    if (cmd.Y.set) {
        m->y.homing_sensitivity = cmd.Y.real;
    }
    report_result("Y:%u ", m->y.homing_sensitivity);
#endif

#ifdef HAS_Z_AXIS
    if (cmd.Z.set) {
        m->z.homing_sensitivity = cmd.Z.real;
    }
    report_result("Z:%u ", m->z.homing_sensitivity);
#endif

    report_result_ln("");
}

void Machine_home(struct Machine* m, bool x __unused, bool y __unused, bool z __unused) {
#ifdef HAS_XY_AXES
    if (x) {
        LinearAxis_home(&(m->x));
    }
    if (y) {
        LinearAxis_home(&(m->y));
    }
#endif
#ifdef HAS_Z_AXIS
    if (z) {
        LinearAxis_home(&(m->z));
    }
#endif
}

struct LinearAxisMovement
__not_in_flash_func(calculate_linear_axis_move)(struct Machine* m, struct LinearAxis* axis, struct lilg_Decimal field) {
    float dest_mm = lilg_Decimal_to_float(field);
    if (!m->absolute_positioning) {
        dest_mm = LinearAxis_get_position_mm(axis) + dest_mm;
    }
    return LinearAxis_calculate_move(axis, dest_mm);
}

#ifdef HAS_XY_AXES
void __not_in_flash_func(bresenham_xy_move)(struct Machine* m, const struct lilg_Command cmd) {
    struct LinearAxisMovement x_move = calculate_linear_axis_move(m, &(m->x), cmd.X);
    struct LinearAxisMovement y_move = calculate_linear_axis_move(m, &(m->y), cmd.Y);

    if (x_move.total_step_count > y_move.total_step_count) {
        m->_major_axis = &(m->x);
        m->_minor_axis = &(m->y);
        Bresenham_init(&(m->_bresenham), 0, 0, x_move.total_step_count, y_move.total_step_count);
    } else {
        m->_major_axis = &(m->y);
        m->_minor_axis = &(m->x);
        Bresenham_init(&(m->_bresenham), 0, 0, y_move.total_step_count, x_move.total_step_count);
    }

    report_info_ln(
        "coordinated move: major axis: %c, minor axis: %c, major steps: %li, minor steps: %li",
        m->_major_axis->name,
        m->_minor_axis->name,
        m->_bresenham.x1,
        m->_bresenham.y1);

    LinearAxis_start_move(&(m->x), x_move);
    LinearAxis_start_move(&(m->y), y_move);
    while (LinearAxis_is_moving(m->_major_axis)) {
        if (LinearAxis_timed_step(m->_major_axis)) {
            if (Bresenham_step(&(m->_bresenham))) {
                LinearAxis_direct_step(m->_minor_axis);
            }
        }
    }

    // Make sure to finish the minor axis' movement:
    while (LinearAxis_is_moving(m->_minor_axis)) { LinearAxis_direct_step(m->_minor_axis); }
}
#endif

void Machine_move(struct Machine* m, const struct lilg_Command cmd) {

#ifdef HAS_XY_AXES
    if (cmd.X.set && cmd.Y.set) {
        bresenham_xy_move(m, cmd);
    } else {
        if (cmd.X.set) {
            struct LinearAxisMovement move = calculate_linear_axis_move(m, &(m->x), cmd.X);
            LinearAxis_start_move(&(m->x), move);
            LinearAxis_wait_for_move(&(m->x));
        }
        if (cmd.Y.set) {
            struct LinearAxisMovement move = calculate_linear_axis_move(m, &(m->y), cmd.Y);
            LinearAxis_start_move(&(m->y), move);
            LinearAxis_wait_for_move(&(m->y));
        }
    }
#endif

    // TODO: Maybe run all of these basic axes concurrently / round robin?

#ifdef HAS_Z_AXIS
    if (cmd.Z.set) {
        struct LinearAxisMovement move = calculate_linear_axis_move(m, &(m->z), cmd.Z);
        LinearAxis_start_move(&(m->z), move);
        LinearAxis_wait_for_move(&(m->z));
    }
#endif

#ifdef HAS_A_AXIS
    if (LILG_FIELD(cmd, A).set) {
        float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
        if (!m->absolute_positioning) {
            dest_deg = RotationalAxis_get_position_deg(&(m->a)) + dest_deg;
        }
        RotationalAxis_start_move(&(m->a), dest_deg);
        RotationalAxis_wait_for_move(&(m->a));
    }
#endif
#ifdef HAS_B_AXIS
    if (LILG_FIELD(cmd, B).set) {
        float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
        if (!m->absolute_positioning) {
            dest_deg = RotationalAxis_get_position_deg(&(m->b)) + dest_deg;
        }
        RotationalAxis_start_move(&(m->b), dest_deg);
        RotationalAxis_wait_for_move(&(m->b));
    }
#endif
}

void Machine_report_position(struct Machine* m) {
    report_result("position:");
#ifdef HAS_XY_AXES
    report_result(
        " X:%0.3f Y:%0.3f", (double)LinearAxis_get_position_mm(&(m->x)), (double)LinearAxis_get_position_mm(&(m->y)));
#endif
#ifdef HAS_Z_AXIS
    report_result(" Z:%0.3f", (double)LinearAxis_get_position_mm(&(m->z)));
#endif
#ifdef HAS_A_AXIS
    report_result(" A:%0.1f", (double)RotationalAxis_get_position_deg(&(m->a)));
#endif
#ifdef HAS_B_AXIS
    report_result(" B:%0.1f", (double)RotationalAxis_get_position_deg(&(m->b)));
#endif
    report_result(" count:");
#ifdef HAS_XY_AXES
    report_result(" X:%li Y:%li", m->x.stepper->total_steps, m->y.stepper->total_steps);
#endif
#ifdef HAS_Z_AXIS
    report_result(" Z:%li", m->y.stepper->total_steps);
#endif
#ifdef HAS_A_AXIS
    report_result(" A:%li", m->a.stepper->total_steps);
#endif
#ifdef HAS_B_AXIS
    report_result(" B:%li", m->z.stepper->total_steps);
#endif
    report_result_ln("");
}

void Machine_set_position(struct Machine* m, const struct lilg_Command cmd) {
#ifdef HAS_XY_AXES
    if (cmd.X.set) {
        LinearAxis_set_position_mm(&(m->x), lilg_Decimal_to_float(cmd.X));
    }
    if (cmd.Y.set) {
        LinearAxis_set_position_mm(&(m->y), lilg_Decimal_to_float(cmd.Y));
    }
#endif
#ifdef HAS_Z_AXIS
    if (cmd.Z.set) {
        LinearAxis_set_position_mm(&(m->z), lilg_Decimal_to_float(cmd.Z));
    }
#endif
#ifdef HAS_A_AXIS
    if (LILG_FIELD(cmd, A).set) {
        RotationalAxis_set_position_deg(&(m->a), lilg_Decimal_to_float(LILG_FIELD(cmd, A)));
    }
#endif
#ifdef HAS_B_AXIS
    if (LILG_FIELD(cmd, B).set) {
        RotationalAxis_set_position_deg(&(m->b), lilg_Decimal_to_float(LILG_FIELD(cmd, B)));
    }
#endif

    Machine_report_position(m);
}

void Machine_report_tmc_info(struct Machine* m) {
    TMC2209_print_all(&(m->tmc[0]));
    TMC2209_print_all(&(m->tmc[1]));
    TMC2209_print_all(&(m->tmc[2]));
}
