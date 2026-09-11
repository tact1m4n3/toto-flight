const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;

pub var msg_attitude: Message(math.Vec3) = .{};
