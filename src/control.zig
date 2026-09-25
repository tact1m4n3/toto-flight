const std = @import("std");
const assert = std.debug.assert;

const hw = @import("hw.zig");
const math = @import("math.zig");
const time = @import("time.zig");
const fusion = @import("fusion.zig");
const imu = @import("imu.zig");
const storage = @import("storage.zig");
const receiver = @import("receiver.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const parameter = @import("parameter.zig");

const log = std.log.scoped(.control);

var global_status: std.atomic.Value(Status) = .init(.{
    .armed = false,
    .failsafe = true,
});
pub var msg_actuator_output: Message(ActuatorOutput) = .{};

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
        .roll = .{ .kff = 1.0, .kp = 0.1, .ki = 0.05 },
        .pitch = .{ .kff = 1.0, .kp = 0.1, .ki = 0.05 },
        .yaw = .{ .kff = 1.0, .kp = 0.0, .ki = 0.0 },
    };
};

pub fn get_status() Status {
    return global_status.load(.monotonic);
}

pub const Status = packed struct(u32) {
    armed: bool,
    failsafe: bool,
    _reserved: u30 = 0,
};

pub const Loop = struct {
    rcv_tick_rate: Receiver(imu.Data) = undefined,
    rcv_tick_nav: Receiver(void) = undefined,
    rcv_command: Receiver(Command) = undefined,

    last_rate_tick: ?time.Absolute = null,

    command_received_time: ?time.Absolute = null,
    command: Command = .disarm,

    rate_command: ?RateCommand = null,

    // NOTE: start as arm failed to prevent accidental arming on boot
    arm_state: ArmState = .arm_failed,
    failsafe_state: FailsafeState = .failsafe,

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

        hw.ticker(.@"100Hz").subscribe(
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
        // TODO: helper struct for dt calculation
        const now = hw.get_time_since_boot();
        const last_tick = control.last_rate_tick orelse now;
        control.last_rate_tick = now;
        const dt = now.diff(last_tick);
        const dt_f32 = dt.to_secs_f32();

        const params = parameter.get(struct {
            rate: RateParams,
        });

        const command: Command = if (control.arm_state == .armed) control.command else .disarm;
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
                        .throw = control.rate_controller.update(&params.rate, target.rate, data.gyro, dt_f32),
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

    fn nav_tick_callback(control: *Loop, _: void) void {
        // TODO: configurable
        const MIN_COMMAND_PERIOD: time.Duration = .from_hz(5);
        const FAILSAFE_PROBATION_DURATION: time.Duration = .from_ms(500);
        const FAILSAFE_RECOVERY_DURATION: time.Duration = .from_ms(1000);

        const ANGLE_ROLL_KP = 0.5;
        const ANGLE_PITCH_KP = 0.5;

        const now = hw.get_time_since_boot();

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
            .disarmed => if (!control.failsafe_state.is_failsafe() and control.command != .disarm) {
                // if it is failsafe, don't trust the command

                if (control.arm_checks_pass()) {
                    log.info("armed", .{});
                    control.arm_state = .armed;
                } else {
                    log.warn("arm failed", .{});
                    control.arm_state = .arm_failed;
                }
            },
            .armed, .arm_failed => if (control.command == .disarm or control.failsafe_state.is_failsafe()) {
                log.info("disarmed", .{});
                control.arm_state = .disarmed;
            },
        }

        global_status.store(.{
            .armed = control.arm_state == .armed,
            .failsafe = control.failsafe_state.is_failsafe(),
        }, .monotonic);

        const command: Command = if (control.arm_state == .armed) control.command else .disarm;
        control.rate_command = switch (command) {
            .angle => |target| blk: {
                const attitude = fusion.msg_attitude.get() orelse
                    break :blk null;
                const euler_angles = attitude.to_euler_angles();

                break :blk .{
                    .throttle = target.throttle,
                    .rate = .{
                        .x = (target.angle_roll - euler_angles.x) * ANGLE_ROLL_KP,
                        .y = (target.angle_pitch - euler_angles.y) * ANGLE_PITCH_KP,
                        .z = target.rate_yaw,
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

    fn arm_checks_pass(_: *Loop) bool {
        const MAX_ARM_THROTTLE_US = 1100; // TODO: configurable

        const channels: receiver.Channels = receiver.msg_channels.get() orelse .default;
        if (channels.get(.throttle) > MAX_ARM_THROTTLE_US) {
            log.warn("failed to arm... throttle high", .{});
            return false;
        }

        if (imu.arm_block_calibrating.is_blocking()) {
            log.warn("failed to arm... imu calibration", .{});
            return false;
        }

        if (storage.arm_block.is_blocking()) {
            log.warn("failed to arm... storage operation", .{});
            return false;
        }

        return true;
    }
};

const ArmState = enum {
    disarmed,
    arm_failed,
    armed,
};

const FailsafeState = union(enum) {
    off,
    probation: time.Absolute,
    failsafe,
    recovery: time.Absolute,

    pub fn is_failsafe(self: FailsafeState) bool {
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
    rate: RateCommand,
    angle: struct {
        throttle: f32,
        /// rad
        angle_roll: f32,
        /// rad
        angle_pitch: f32,
        /// rad/s
        rate_yaw: f32,
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
            gains: *const RateParams.AxisGains,
            target_rate: f32,
            current_rate: f32,
            dt: f32,
        ) f32 {
            const rate_error = target_rate - current_rate;

            const ff_term = gains.kff * target_rate / std.math.pi;
            const p_term = gains.kp * rate_error / std.math.pi;

            if (@abs(rate_error) <= i_term_max_error) {
                axis.i_term += gains.ki * rate_error / std.math.pi * dt;
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
        params: *const RateParams,
        target_rate: math.Vec3,
        current_rate: math.Vec3,
        dt: f32,
    ) math.Vec3 {
        return .{
            .x = controller.roll.update(&params.roll, target_rate.x, current_rate.x, dt),
            .y = controller.pitch.update(&params.pitch, target_rate.y, current_rate.y, dt),
            .z = controller.yaw.update(&params.yaw, target_rate.z, current_rate.z, dt),
        };
    }
};

pub const ArmBlock = struct {
    state: std.atomic.Value(bool),

    pub const init: ArmBlock = .{
        .state = .init(false),
    };

    pub fn is_blocking(arm_block: *ArmBlock) bool {
        return arm_block.state.load(.monotonic);
    }

    pub fn acquire(arm_block: *ArmBlock) !void {
        if (global_status.load(.acquire).armed) {
            return error.Armed;
        }
        if (arm_block.state.swap(true, .acquire) != false) {
            return error.AlreadyAcquired;
        }
    }

    pub fn release(arm_block: *ArmBlock) void {
        arm_block.state.store(false, .release);
    }
};
