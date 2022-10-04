#include "machine.h"
#include "config/motion.h"
#include "config/serial.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include <stdio.h>

/*
    Macros
*/
#define INIT_STEPPER(number, LETTER)                                                                                   \
    Stepper_init(                                                                                                      \
        &(m->stepper[number]),                                                                                         \
        (m->stepper[number].tmc),                                                                                      \
        LETTER##_PIN_EN,                                                                                               \
        LETTER##_PIN_DIR,                                                                                              \
        LETTER##_PIN_STEP,                                                                                             \
        LETTER##_PIN_DIAG,                                                                                             \
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
    RotationalAxis_init(&(m->##letter), #LETTER, &(m->stepper[LETTER##_STEPPER]));                                     \
    m->##letter.steps_per_deg = LETTER##_STEPS_PER_DEG;

/*
    Public functions
*/

void Machine_init(struct Machine* m) {
    m->absolute_positioning = true;
    m->_is_coordinated_move = false;

    TMC2209_init(&m->tmc[0], TMC_UART_INST, 0, tmc_uart_read_write);
    TMC2209_init(&m->tmc[1], TMC_UART_INST, 1, tmc_uart_read_write);
    // Note: This should be 2, but both Jellyfish & Starfish skip address 2.
    TMC2209_init(&m->tmc[3], TMC_UART_INST, 3, tmc_uart_read_write);

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
    INIT_ROTATIONAL_AXIS(A, a);
#endif

#ifdef HAS_B_AXIS
    INIT_ROTATIONAL_AXIS(B, b);
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
    printf("> ");
#ifdef HAS_XY_AXES
    if (cmd.X.set) {
        m->x.homing_sensitivity = cmd.X.real;
    }
    printf("X:%u ", m->x.homing_sensitivity);
    if (cmd.Y.set) {
        m->y.homing_sensitivity = cmd.Y.real;
    }
    printf("Y:%u ", m->y.homing_sensitivity);
#endif
#ifdef HAS_Z_AXIS
    if (cmd.Z.set) {
        m->z.homing_sensitivity = cmd.Z.real;
    }
    printf("Z:%u ", m->z.homing_sensitivity);
#endif
    printf("\n");
}

void Machine_home(struct Machine* m, bool x, bool y, bool z) {
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

#ifdef HAS_XY_AXES
void Machine_coordinated_xy_move(struct Machine* m, const struct lilg_Command cmd) {
    float x_dest_mm = lilg_Decimal_to_float(cmd.X);
    float y_dest_mm = lilg_Decimal_to_float(cmd.Y);

    if (!m->absolute_positioning) {
        x_dest_mm = LinearAxis_get_position_mm(&(m->x)) + x_dest_mm;
        y_dest_mm = LinearAxis_get_position_mm(&(m->y)) + y_dest_mm;
    }
    struct LinearAxisMovement x_move = LinearAxis_calculate_move(&(m->x), x_dest_mm);
    struct LinearAxisMovement y_move = LinearAxis_calculate_move(&(m->y), y_dest_mm);

    uint32_t irq_status = save_and_disable_interrupts();

    if (cmd.X.set && cmd.Y.set) {
        m->_is_coordinated_move = true;
        if (x_move.total_step_count > y_move.total_step_count) {
            m->_major_axis = &(m->x);
            m->_minor_axis = &(m->y);
            Bresenham_init(&(m->_bresenham), 0, 0, x_move.total_step_count, y_move.total_step_count);
        } else {
            m->_major_axis = &(m->y);
            m->_minor_axis = &(m->x);
            Bresenham_init(&(m->_bresenham), 0, 0, y_move.total_step_count, x_move.total_step_count);
        }
        printf(
            "> Coordinated move: major axis: %c, minor axis: %c, major steps: %i, minor steps: %i\n",
            m->_major_axis->name,
            m->_minor_axis->name,
            m->_bresenham.x1,
            m->_bresenham.y1);
    } else {
        m->_is_coordinated_move = false;
    }

    if (cmd.X.set) {
        LinearAxis_start_move(&(m->x), x_move);
    }
    if (cmd.Y.set) {
        LinearAxis_start_move(&(m->y), y_move);
    }

    restore_interrupts(irq_status);
}
#endif

void Machine_basic_move(struct Machine* m, const struct lilg_Command cmd) {
#ifdef HAS_Z_AXIS
    if (cmd.Z.set) {
        float dest_mm = lilg_Decimal_to_float(cmd.Z);
        if (!absolute_positioning) {
            dest_mm = LinearAxis_get_position_mm(&z_axis) + dest_mm;
        }
        LinearAxis_start_move(&(m->z), dest_mm);
    }
#endif
#ifdef HAS_A_AXIS
    if (LILG_FIELD(cmd, A).set) {
        float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, A));
        if (!absolute_positioning) {
            dest_deg = RotationalAxis_get_position_deg(&a_axis) + dest_deg;
        }
        RotationalAxis_start_move(&(m->a), dest_deg);
    }
#endif
#ifdef HAS_B_AXIS
    if (LILG_FIELD(cmd, B).set) {
        float dest_deg = lilg_Decimal_to_float(LILG_FIELD(cmd, B));
        if (!absolute_positioning) {
            dest_deg = RotationalAxis_get_position_deg(&b_axis) + dest_deg;
        }
        RotationalAxis_start_move(&(m->b), dest_deg);
    }
#endif
}

void Machine_wait_for_moves_to_finish(struct Machine* m) {
#ifdef HAS_XY_AXES
    LinearAxis_wait_for_move(&(m->x));
    LinearAxis_wait_for_move(&(m->y));
#endif
#ifdef HAS_Z_AXIS
    LinearAxis_wait_for_move(&(m->z));
#endif
#ifdef HAS_A_AXIS
    RotationalAxis_wait_for_move(&(m->a));
#endif
#ifdef HAS_B_AXIS
    RotationalAxis_wait_for_move(&(m->b));
#endif
}

void Machine_report_position(struct Machine* m) {
    printf("> position:");
#ifdef HAS_XY_AXES
    printf(" X:%0.3f Y:%0.3f", LinearAxis_get_position_mm(&(m->x)), LinearAxis_get_position_mm(&(m->y)));
#endif
#ifdef HAS_Z_AXIS
    printf(" Z:%0.3f", LinearAxis_get_position_mm(&(m->z)));
#endif
#ifdef HAS_A_AXIS
    printf(" A:%0.1f", RotationalAxis_get_position_deg(&(m->a)));
#endif
#ifdef HAS_B_AXIS
    printf(" B:%0.1f", RotationalAxis_get_position_deg(&(m->b)));
#endif
    printf("count:");
#ifdef HAS_XY_AXES
    printf(" X:%i Y:%i", m->x.stepper->total_steps, m->y.stepper->total_steps);
#endif
#ifdef HAS_Z_AXIS
    printf(" Z:%i", m->y.stepper->total_steps);
#endif
#ifdef HAS_A_AXIS
    printf(" A:%i", m->a.stepper->total_steps);
#endif
#ifdef HAS_B_AXIS
    printf(" B:%i", m->z.stepper->total_steps);
#endif
    printf("\n");
}

void Machine_report_tmc_info(struct Machine* m) {
    TMC2209_print_all(&(m->tmc[0]));
    TMC2209_print_all(&(m->tmc[1]));
    TMC2209_print_all(&(m->tmc[2]));
}

int64_t Machine_step(struct Machine* m) {
#ifdef HAS_XY_AXES
    // if (coordinated_move) {
    //     LinearAxis_step(major_axis);
    //     if (Bresenham_step(&bresenham)) {
    //         LinearAxis_step(minor_axis);
    //     }
    // } else {
    //     LinearAxis_step(&x_axis);
    //     LinearAxis_step(&y_axis);
    // }
    LinearAxis_step(&(m->x));
    LinearAxis_step(&(m->y));
#endif
#ifdef HAS_Z_AXIS
    LinearAxis_step(&(m->z));
#endif
#ifdef HAS_A_AXIS
    RotationalAxis_step(&(m->a));
#endif
#ifdef HAS_B_AXIS
    RotationalAxis_step(&(m->b));
#endif
    return STEP_INTERVAL_US;
}
