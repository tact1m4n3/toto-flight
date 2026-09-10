const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;

pub var msg_attitude: Message(Attitude) = .{};

/// Euler angles, radians.
pub const Attitude = extern struct {
    roll: f32,
    pitch: f32,
    yaw: f32,
};
