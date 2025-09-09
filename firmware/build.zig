const std = @import("std");
const microzig = @import("microzig");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const mb = microzig.MicroBuild(.{ .rp2xxx = true }).init(b, b.dependency("microzig", .{})) orelse return;
    const fw = mb.add_firmware(.{
        .name = "playground",
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

    const printer_mod = b.dependency("printer", .{}).module("printer");
    const serial_mod = b.dependency("serial", .{}).module("serial");

    const flasher_exe = b.addExecutable(.{
        .name = "flasher",
        .root_module = b.createModule(.{
            .root_source_file = b.path("tools/flasher.zig"),
            .imports = &.{
                .{ .name = "printer", .module = printer_mod },
                .{ .name = "serial", .module = serial_mod },
            },
            .target = target,
            .optimize = .ReleaseSafe,
        }),
    });

    const serial_dev_path = b.option([]const u8, "serial_dev", "serial port device path") orelse "/dev/ttyUSB0";
    const serial_baud_rate = b.option(usize, "serial_baud_rate", "serial port baud rate") orelse 115_200;

    const flasher_run = b.addRunArtifact(flasher_exe);
    flasher_run.addFileArg(fw.get_emitted_elf());
    flasher_run.addArgs(&.{ serial_dev_path, b.fmt("{d}", .{serial_baud_rate}) });

    const run_step = b.step("run", "run");
    run_step.dependOn(&flasher_run.step);
}
