const std = @import("std");
const microzig = @import("microzig");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});
    const board = b.option(Board, "board", "Which board to build for") orelse .madflight_rp2350;

    const platform_options = b.addOptions();
    platform_options.addOption(Board, "board", board);
    const platform_options_mod = platform_options.createModule();

    // switch (board) {
    //     .simulator => {
    const jsbsim_dep = b.lazyDependency("jsbsim", .{}) orelse return;

    const exe = b.addExecutable(.{
        .name = "simulator",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main_simulator.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "options", .module = platform_options_mod },
                .{ .name = "jsbsim_c", .module = jsbsim_dep.module("bindings") },
            },
        }),
    });
    b.installArtifact(exe);

    const run = b.addRunArtifact(exe);
    const run_step = b.step("run", "run sim");
    run_step.dependOn(&run.step);
    //  },
    //     else => |uc_board| {
    //         const mz_dep = b.lazyDependency("microzig", .{}) orelse return;
    //         const mb = MicroBuild.init(b, mz_dep) orelse return;
    //
    //         const root_source_file = switch (uc_board) {
    //             .madflight_rp2350 => b.path("src/main_firmware.zig"),
    //             .simulator => unreachable,
    //         };
    //
    //         const mz_target = switch (uc_board) {
    //             .madflight_rp2350 => blk: {
    //                 // TODO: technically we should derive rp2350_arm instead of pico2_arm
    //                 // TODO: should be easier to override memory regions
    //                 var modified_chip = mb.ports.rp2xxx.boards.raspberrypi.pico2_arm.chip;
    //                 modified_chip.memory_regions = &.{
    //                     .{ .tag = .flash, .offset = 0x10000000, .length = 16 * 1024 * 1024, .access = .rx },
    //                     .{ .tag = .ram, .offset = 0x20000000, .length = 512 * 1024, .access = .rwx },
    //                     // TODO: maybe these can be used for stacks
    //                     .{ .tag = .ram, .offset = 0x20080000, .length = 4 * 1024, .access = .rwx },
    //                     .{ .tag = .ram, .offset = 0x20081000, .length = 4 * 1024, .access = .rwx },
    //                 };
    //
    //                 break :blk mb.ports.rp2xxx.boards.raspberrypi.pico2_arm.derive(.{
    //                     .chip = modified_chip,
    //                 });
    //             },
    //             .simulator => unreachable,
    //         };
    //
    //         const fw = mb.add_firmware(.{
    //             .name = b.fmt("firmware-{s}", .{@tagName(uc_board)}),
    //             .target = mz_target,
    //             .optimize = optimize,
    //             .root_source_file = root_source_file,
    //             .imports = &.{
    //                 .{ .name = "options", .module = platform_options_mod },
    //             },
    //         });
    //
    //         mb.install_firmware(fw, .{});
    //         mb.install_firmware(fw, .{ .format = .elf });
    //     },
    // }

    const tests_exe = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/root_test.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "options", .module = platform_options_mod },
            },
        }),
    });

    const tests_run = b.addRunArtifact(tests_exe);
    const test_step = b.step("test", "run tests");
    test_step.dependOn(&tests_run.step);
}

const Board = enum {
    madflight_rp2350,
    simulator,
};

const MicroBuild = microzig.MicroBuild(.{
    .rp2xxx = true,
});
