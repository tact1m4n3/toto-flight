const imu = @import("imu.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;

pub const Command = union(enum) {
    disarm,
    manual: struct {
        throttle: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
    },
    /// rad/s
    rate: struct {
        throttle: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
    },
};

pub const RateController = struct {
    rcv_imu_data: Receiver(imu.Data) = undefined,
    rcv_command: Receiver(Command) = undefined,

    pub fn init(
        rate_controller: *RateController,
        scheduler: *Scheduler,
        msg_imu_data: *Message(imu.Data),
        msg_command: *Message(Command),
    ) void {
        rate_controller.* = .{};

        msg_imu_data.subscribe(&rate_controller.rcv_imu_data, RateController, rate_controller, imu_data_callback, scheduler);
        msg_command.subscribe(&rate_controller.rcv_command, RateController, rate_controller, command_callback, scheduler);
    }

    fn imu_data_callback(rate_controller: *RateController, data: imu.Data) void {
        _ = rate_controller; // autofix
        _ = data; // autofix
    }

    fn command_callback(rate_controller: *RateController, command: Command) void {
        _ = rate_controller; // autofix
        _ = command; // autofix
    }
};
