const imu = @import("imu.zig");
const Scheduler = @import("Scheduler.zig");
const Subscriber = Scheduler.Subscriber;

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
    imu_data_sub: Scheduler.Subscriber(imu.Data),
    command_sub: Scheduler.Subscriber(Command),

    pub fn init(
        rate_controller: *RateController,
        scheduler: *Scheduler,
        imu_msg: *Scheduler.PubSub(imu.Data),
        command_msg: *Scheduler.PubSub(Command),
    ) void {
        rate_controller.* = .{
            .imu_sub = undefined,
            .command_sub = undefined,
        };
        rate_controller.imu_data_sub.subscribe(
            imu_msg,
            RateController,
            rate_controller,
            imu_data_callback,
            scheduler,
        );
        rate_controller.command_sub.subscribe(
            command_msg,
            RateController,
            rate_controller,
            command_callback,
            scheduler,
        );
    }

    pub fn imu_data_callback(rate_controller: *RateController, data: imu.Data) void {
        _ = rate_controller; // autofix
        _ = data; // autofix
        _ = rate_controller;
    }

    pub fn command_callback(rate_controller: *RateController, command: Command) void {
        _ = rate_controller; // autofix
        _ = command; // autofix
    }
};
