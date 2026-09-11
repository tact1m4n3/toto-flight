const std = @import("std");

const hw = @import("hw.zig");
const math = @import("math.zig");
const time = @import("time.zig");
const fusion = @import("fusion.zig");
const imu = @import("imu.zig");
const receiver = @import("receiver.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const ParamTable = Scheduler.ParamTable;

const log = std.log.scoped(.control);

pub var param_table_rate: ParamTable(RateParams) = .{};

pub var msg_status: Message(Status) = .{};
pub var msg_actuator_output: Message(ActuatorOutput) = .{};

pub const Loop = struct {
    rcv_tick_rate: Receiver(imu.Data) = undefined,
    rcv_tick_nav: Receiver(time.Absolute) = undefined,
    rcv_command: Receiver(Command) = undefined,

    last_rate_tick: time.Absolute = .from_us(0),

    command_received_time: ?time.Absolute = null,
    command: Command = .disarm,

    rate_command: ?RateCommand = null,

    arm_state: bool = false,
    failsafe_state: Failsafe = .failsafe,

    rate_controller: RateController = .{},

    pub fn init(
        loop: *Loop,
        scheduler: *Scheduler,
    ) void {
        loop.* = .{};

        imu.msg_data.subscribe(
            &loop.rcv_tick_rate,
            *Loop,
            loop,
            rate_tick_callback,
            scheduler,
        );

        hw.Ticker.@"100Hz".subscribe(
            &loop.rcv_tick_nav,
            *Loop,
            loop,
            nav_tick_callback,
            scheduler,
        );

        receiver.msg_command.subscribe(
            &loop.rcv_command,
            *Loop,
            loop,
            new_command_callback,
            scheduler,
        );
    }

    fn rate_tick_callback(control: *Loop, data: imu.Data) void {
        const duration_since_last_tick = data.ts.diff(control.last_rate_tick);
        control.last_rate_tick = data.ts;

        const dt = duration_since_last_tick.to_secs_f32();

        const params: RateParams = param_table_rate.get() orelse .default;

        const command: Command = if (control.arm_state) control.command else .disarm;
        loop: switch (command) {
            .angle => { // and other nav tick handled commands
                if (control.rate_command) |rate_command| {
                    continue :loop .{ .rate = rate_command };
                }
            },
            .rate => |target| {
                continue :loop .{
                    .manual = .{
                        .throttle = target.throttle,
                        .throw = control.rate_controller.update(params, target.rate, data.gyro, dt),
                    },
                };
            },
            .manual => |target| {
                msg_actuator_output.publish(target);
            },
            .disarm => {
                control.rate_controller.reset_i_term();
                msg_actuator_output.publish(.off);
            },
        }
    }

    fn nav_tick_callback(control: *Loop, now: time.Absolute) void {
        const MIN_COMMAND_PERIOD: time.Duration = .from_hz(5);
        const FAILSAFE_PROBATION_DURATION: time.Duration = .from_ms(500);
        const FAILSAFE_RECOVERY_DURATION: time.Duration = .from_ms(1000);

        const ANGLE_ROLL_KP = 0.5;
        const ANGLE_PITCH_KP = 0.5;

        const command_in_due_time = if (control.command_received_time) |command_received_time|
            now.diff(command_received_time).less_than(MIN_COMMAND_PERIOD)
        else
            false;

        switch (control.failsafe_state) {
            .off => if (!command_in_due_time) {
                control.failsafe_state = .{ .probation = now.add_duration(FAILSAFE_PROBATION_DURATION) };
            },
            .probation => |probation_end| if (command_in_due_time) {
                control.failsafe_state = .off;
            } else if (probation_end.is_reached_by(now)) {
                control.failsafe_state = .failsafe;
                control.arm_state = false;
            },
            .failsafe => if (command_in_due_time) {
                control.failsafe_state = .{ .recovery = now.add_duration(FAILSAFE_RECOVERY_DURATION) };
            },
            .recovery => |recovery_end| if (!command_in_due_time) {
                control.failsafe_state = .failsafe;
            } else if (recovery_end.is_reached_by(now)) {
                control.failsafe_state = .off;
            },
        }

        switch (control.arm_state) {
            false => if (control.command != .disarm) {
                if (control.failsafe_state.is_failsafe()) {
                    log.warn("failed to arm: arm checks didn't pass", .{});
                } else {
                    log.info("armed", .{});
                    control.arm_state = true;
                }
            },
            true => if (control.command == .disarm) {
                log.info("disarmed", .{});
                control.arm_state = false;
            },
        }

        msg_status.publish(.{
            .failsafe = control.failsafe_state.is_failsafe(),
            .arm = control.arm_state,
        });

        const command: Command = if (control.arm_state) control.command else .disarm;
        control.rate_command = switch (command) {
            .angle => |target| blk: {
                const attitude = fusion.msg_attitude.get() orelse
                    break :blk null;

                break :blk .{
                    .throttle = target.throttle,
                    .rate = .{
                        .x = (target.angle_roll - attitude.x) * ANGLE_ROLL_KP,
                        .y = (target.angle_pitch - attitude.y) * ANGLE_PITCH_KP,
                        .z = 0.0,
                    },
                };
            },
            else => null,
        };
    }

    fn new_command_callback(control: *Loop, command: Command) void {
        control.command = command;
        control.command_received_time = hw.get_time_since_boot();
    }
};

const Failsafe = union(enum) {
    off,
    probation: time.Absolute,
    failsafe,
    recovery: time.Absolute,

    pub fn is_failsafe(self: Failsafe) bool {
        return switch (self) {
            .off => false,
            .probation => false,
            .failsafe => true,
            .recovery => true,
        };
    }
};

pub const Command = union(enum) {
    disarm,
    manual: ActuatorOutput,
    /// rad/s
    rate: RateCommand,
    /// rad
    angle: struct {
        throttle: f32,
        angle_roll: f32,
        angle_pitch: f32,
    },
};

pub const ActuatorOutput = struct {
    /// 0..=1
    throttle: f32,
    /// -1..=1
    throw: math.Vec3,

    pub const off: ActuatorOutput = .{
        .throttle = 0.0,
        .throw = .zero,
    };
};

pub const RateCommand = struct {
    /// 0..=1
    throttle: f32,
    /// rad/s
    rate: math.Vec3,
};

pub const RateParams = extern struct {
    roll: AxisGains,
    pitch: AxisGains,
    yaw: AxisGains,

    pub const AxisGains = extern struct {
        kff: f32,
        kp: f32,
        ki: f32,
    };

    pub const default: RateParams = .{
        .roll = .{ .kff = 1.0, .kp = 1.5, .ki = 0.8 },
        .pitch = .{ .kff = 1.0, .kp = 1.5, .ki = 0.8 },
        .yaw = .{ .kff = 1.0, .kp = 1.5, .ki = 0.8 },
    };
};

/// The i term only accumulates while the rate error is small, so a large
/// tracking error can't wind it up.
const i_term_max_error: f32 = 0.5;

pub const RateController = struct {
    roll: AxisControl = .{},
    pitch: AxisControl = .{},
    yaw: AxisControl = .{},

    pub const AxisControl = struct {
        i_term: f32 = 0.0,

        pub fn update(
            axis: *AxisControl,
            gains: RateParams.AxisGains,
            target_rate: f32,
            current_rate: f32,
            dt: f32,
        ) f32 {
            const rate_error = target_rate - current_rate;

            const ff_term = gains.kff * target_rate;
            const p_term = gains.kp * rate_error;

            if (@abs(rate_error) <= i_term_max_error) {
                axis.i_term += gains.ki * rate_error * dt;
            }

            return ff_term + p_term + axis.i_term;
        }
    };

    pub fn reset_i_term(controller: *RateController) void {
        controller.roll.i_term = 0.0;
        controller.pitch.i_term = 0.0;
        controller.yaw.i_term = 0.0;
    }

    pub fn update(
        controller: *RateController,
        params: RateParams,
        target_rate: math.Vec3,
        current_rate: math.Vec3,
        dt: f32,
    ) math.Vec3 {
        return .{
            .x = controller.roll.update(params.roll, target_rate.x, current_rate.x, dt),
            .y = controller.pitch.update(params.pitch, target_rate.y, current_rate.y, dt),
            .z = controller.yaw.update(params.yaw, target_rate.z, current_rate.z, dt),
        };
    }
};

pub const Status = packed struct {
    arm: bool,
    failsafe: bool,
};

const testing = std.testing;

test "RateController.AxisControl" {
    var axis: RateController.AxisControl = .{};
    const gains: RateParams.AxisGains = .{ .kff = 1.0, .kp = 2.0, .ki = 0.5 };

    const dt: f32 = 0.002;
    // ff 1*1 + p 2*0.75 + i 0.5*0.75*0.002
    try testing.expectApproxEqAbs(1.0 + 1.5 + 0.5 * 0.75 * dt, axis.update(gains, 1.0, 0.25, dt), 1e-6);

    // error past i_term_max_error freezes the i term
    _ = axis.update(gains, 100.0, 0.0, dt);
    try testing.expectApproxEqAbs(0.5 * 0.75 * dt, axis.i_term, 1e-6);
}

test "RateController" {
    var controller: RateController = .{};
    const gyro: math.Vec3 = .{ .x = 0.5, .y = 0.0, .z = 0.0 };
    const target: RateCommand = .{ .x = 1.0, .y = 0.0, .z = 0.0 };

    const out = controller.update(.default, target, gyro, 0.002);
    // roll: ff 1*1 + p 1.5*0.5 + i 0.8*0.5*0.002
    try testing.expectApproxEqAbs(1.0 + 0.75 + 0.8 * 0.5 * 0.002, out.x, 1e-5);
    try testing.expectApproxEqAbs(0.0, out.y, 1e-6);
    try testing.expectApproxEqAbs(0.0, out.z, 1e-6);

    controller.reset_i_term();
    try testing.expectEqual(@as(f32, 0.0), controller.roll.i_term);
    try testing.expectEqual(@as(f32, 0.0), controller.pitch.i_term);
    try testing.expectEqual(@as(f32, 0.0), controller.yaw.i_term);
}
