const std = @import("std");
const microzig = @import("microzig");
const zprobe = @import("zprobe");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const mb = microzig.MicroBuild(.{ .rp2xxx = true }).init(b, b.dependency("microzig", .{})) orelse return;
    const fw = mb.add_firmware(.{
        .name = "firmware",
        .root_source_file = b.path("src/main.zig"),
        .target = mb.ports.rp2xxx.boards.raspberrypi.pico,
        .optimize = optimize,
    });
    mb.install_firmware(fw, .{ .format = .elf });

    const tests_exe = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });

    const tests_run = b.addRunArtifact(tests_exe);
    const test_step = b.step("test", "run tests");
    test_step.dependOn(&tests_run.step);

    const zprobe_dep = b.dependency("zprobe", .{
        .target = b.graph.host,
        .optimize = .ReleaseSafe,
    });
    const zprobe_load_run = zprobe.load(zprobe_dep, .{
        .elf_file = fw.get_emitted_elf(),
        .chip = .RP2040,
        .rtt = true,
    });
    const run_step = b.step("run", "Run firmware");
    run_step.dependOn(zprobe_load_run);
}
