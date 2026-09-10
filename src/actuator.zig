const Scheduler = @import("Scheduler.zig");
const Receiver = Scheduler.Receiver;
const control = @import("control.zig");

pub const Actuator = struct {
    rcv_output: Receiver(control.ActuatorOutput),

    pub fn init(actuator: *Actuator, scheduler: *Scheduler) void {}
};
