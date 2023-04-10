const std = @import("std");
const print = std.debug.print;
const testing = std.testing;
const motion = @import("motion.zig");

test "basic" {
    try testing.expectEqual(4, 2 + 2);

    _ = motion;
}
