comptime {
    _ = @import("Scheduler.zig");
    _ = @import("math.zig");
    _ = @import("receiver.zig");
    _ = @import("drivers/storage.zig");
    _ = @import("drivers/imu/lsm6dsv.zig");
    _ = @import("parsers/CRSF.zig");
    _ = @import("utils/ring_buffer.zig");
}
