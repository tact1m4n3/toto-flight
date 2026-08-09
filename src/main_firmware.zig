const microzig = @import("microzig");
const hw = @import("hw.zig");

pub const panic = microzig.panic;
pub const std_options = microzig.std_options(.{});
pub const microzig_options: microzig.Options = .{
    .interrupts = hw.chip.interrupts,
};
comptime {
    _ = microzig.export_startup();
}
pub const main = hw.chip.main;
