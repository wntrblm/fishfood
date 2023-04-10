const std = @import("std");
const print = std.debug.print;
const testing = std.testing;
const c = @import("cdefs.zig");
const stubs = @import("stubs.zig");

test "c import" {
    var stepper = c.Stepper{
        .tmc = null,
        .pin_enn = 0,
        .pin_dir = 0,
        .pin_step = 0,
        .pin_diag = 0,
        .reversed = false,
        .run_current = 0,
        .hold_current = 0,
        .direction = 1,
        .total_steps = 0,
    };

    var axis = c.LinearAxis{
        .name = 'X',
        .stepper = &stepper,
        .stepper2 = null,
        .steps_per_mm = 160.0,
        .velocity_mm_s = 100,
        .acceleration_mm_s2 = 1000,
        .homing_direction = -1,
        .homing_distance_mm = 1000,
        .homing_bounce_mm = 10,
        .homing_velocity_mm_s = 100,
        .homing_acceleration_mm_s2 = 1000,
        .homing_sensitivity = 127,
        .endstop = 0,
        ._step_interval = 0,
        ._next_step_at = 0,
        ._current_move = .{
            .direction = 1,
            .accel_step_count = 0,
            .decel_step_count = 0,
            .coast_step_count = 0,
            .total_step_count = 0,
            .steps_taken = 0,
        },
    };

    print("calculating move\n", .{});

    const move = c.LinearAxis_calculate_move(&axis, 100.0);

    print("move: {}\n", .{move});
}
