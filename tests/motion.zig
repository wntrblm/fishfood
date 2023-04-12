const std = @import("std");
const print = std.debug.print;
const testing = std.testing;
const c = @import("cdefs.zig");
const stubs = @import("stubs.zig");

fn make_stepper() c.Stepper {
    return c.Stepper{
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
}

fn make_axis(stepper: *c.Stepper) c.LinearAxis {
    return c.LinearAxis{
        .name = 'X',
        .stepper = stepper,
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
            .lut = [_]u16{0} ** c.LINEAR_AXIS_LUT_COUNT,
        },
    };
}

test "LinearAxis: calculate move" {
    var stepper = make_stepper();
    var axis = make_axis(&stepper);

    var move = c.LinearAxis_calculate_move(&axis, 100.0);

    // Millimeters to steps
    // (160 steps/millimeter) × (100 millimeters) = 16000 steps
    try testing.expectEqual(move.total_step_count, 16000);

    // Acceleration and final velocity to total acceleration time - how long it
    // takes to reach final velocity.
    // (100 millimeters/second) / (1000 millimeters/(second²)) = 100 ms
    // Acceleration and time to displacement - this is how far the axis has
    // to travel before it reaches final velocity.
    // (((1 / 2) × (1000 millimeters)) / (second²)) × ((100 milliseconds)^2) = 5 mm
    // Millimeters to step
    // (160 steps/millimeter) × (5 millimeters) = 800 steps
    try testing.expectEqual(move.accel_step_count, 800);
    try testing.expectEqual(move.decel_step_count, 800);

    // With 5 mm needed each to accelerate and decelerate, that leaves 90 mm
    // at full speed.
    // (160 steps/millimeter) × (90 millimeters) = 14400 steps
    try testing.expectEqual(move.coast_step_count, 14400);

    // Now test a move that doesn't have enough time to get to full speed.
    move = c.LinearAxis_calculate_move(&axis, 5.0);

    // (160 steps/millimeter) × (5 millimeters) = 800 steps
    try testing.expectEqual(move.total_step_count, 800);

    // Acceleration and deceleration should equally share steps.
    try testing.expectEqual(move.accel_step_count, 400);
    try testing.expectEqual(move.decel_step_count, 400);
    try testing.expectEqual(move.coast_step_count, 0);
}

test "LinearAxis: step interval" {
    var stepper = make_stepper();
    var axis = make_axis(&stepper);

    var move = c.LinearAxis_calculate_move(&axis, 100.0);

    // Simple case first: middle of the coast phase, so the axis should be
    // moving at its final velocity.
    axis._current_move = move;
    axis._current_move.steps_taken = 8000;

    c.LinearAxis_lookup_step_interval(&axis);

    // working backwards:
    // (1 / ((62 microseconds) / step)) × (1 / (160 steps/millimeter)) ≈ 100.8064516 mm/s
    try testing.expectEqual(axis._step_interval, 62);

    // Acceleration case: halfway through the acceleration steps, so the
    // axis should be around 3/4 the final velocity.
    axis._current_move.steps_taken = 400;

    c.LinearAxis_lookup_step_interval(&axis);

    // working backwards
    // (1 / ((88 microseconds) / step)) × (1 / (160 steps/millimeter)) ≈ 71.02272727 mm/s
    try testing.expectEqual(axis._step_interval, 88);
    try testing.expectEqual(axis._step_interval, axis._current_move.lut[c.LINEAR_AXIS_LUT_COUNT >> 1]);

    // Deceleration case: halfway through the deceleration steps, so the velocity
    // should be the same as above.
    axis._current_move.steps_taken = 800 + 14400 + 400;
    c.LinearAxis_lookup_step_interval(&axis);
    try testing.expectEqual(axis._step_interval, 88);

    // Finally, last step case - the velocity should be as low as it'll get
    axis._current_move.steps_taken = 800 + 14400 + 799;
    c.LinearAxis_lookup_step_interval(&axis);

    try testing.expectEqual(axis._step_interval, 1767);
    try testing.expectEqual(axis._step_interval, axis._current_move.lut[1]);
}

test "LinearAxis: move look up table" {
    var stepper = make_stepper();
    var axis = make_axis(&stepper);
    var move = c.LinearAxis_calculate_move(&axis, 100.0);

    // Each entry in the lut table should be less than or equal to the entry
    // before, since the lut table stores values corresponding to increasing
    // velocity.
    var last = move.lut[0];
    for (move.lut) |current| {
        try testing.expect(current <= last);
        last = current;
    }
}
