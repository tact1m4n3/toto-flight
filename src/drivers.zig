pub const imu = struct {
    pub const lsm6dsv = @import("drivers/imu/lsm6dsv.zig");
};

pub const battery = struct {
    pub const ina226 = @import("drivers/battery/ina226.zig");
};

pub const storage = @import("drivers/storage.zig");

pub const Color = extern struct {
    r: u8,
    g: u8,
    b: u8,

    pub const black: Color = .{ .r = 0, .g = 0, .b = 0 };
    pub const dark_green: Color = .{ .r = 0, .g = 40, .b = 0 };
    pub const dark_blue: Color = .{ .r = 0, .g = 0, .b = 40 };
    pub const dark_red: Color = .{ .r = 40, .g = 0, .b = 0 };
};
