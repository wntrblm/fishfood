const std = @import("std");
const Builder = std.build.Builder;

pub fn build(b: *Builder) void {
    const target = b.standardTargetOptions(.{});
    const cflags = [_][]const u8{
        "-std=gnu17",
        "-DSTARFISH",
    };

    const lib = b.addStaticLibrary(.{
        .name = "stubs",
        .root_source_file = .{ .path = "stubs.zig" },
        .target = target,
        .optimize = .Debug,
    });
    lib.addIncludePath("stubs");
    lib.addIncludePath("../src");
    lib.addIncludePath("../src/motion");

    lib.install();

    const main = b.addTest(.{
        .root_source_file = .{ .path = "test.zig" },
        .target = target,
        .optimize = .Debug,
    });

    main.linkLibC();
    main.linkLibrary(lib);
    main.addIncludePath("stubs");
    main.addIncludePath("../src");
    main.addIncludePath("../src/motion");
    main.addCSourceFile("../src/report.c", &cflags);
    main.addCSourceFile("../src/motion/linear_axis.c", &cflags);

    main.install();

    b.default_step.dependOn(&main.run().step);
}
