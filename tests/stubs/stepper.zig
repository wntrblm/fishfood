const c = @import("../cdefs.zig");

export fn Stepper_enable_stallguard(s: [*c]c.Stepper, run_current: f32, hold_current: f32) void {
    _ = s;
    _ = run_current;
    _ = hold_current;
}

export fn Stepper_disable_stallguard(s: [*c]c.Stepper) void {
    _ = s;
}

export fn Stepper_stalled(s: [*c]c.Stepper) bool {
    _ = s;
    return false;
}

export fn Stepper_step(s: [*c]c.Stepper) void {
    _ = s;
}

export fn Stepper_step_two(s1: [*c]c.Stepper, s2: [*c]c.Stepper) void {
    _ = s1;
    _ = s2;
}

export fn Stepper_update_direction(s: [*c]c.Stepper) void {
    _ = s;
}
