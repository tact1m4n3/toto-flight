const math = @import("math.zig");
const time = @import("time.zig");
const fusion = @import("fusion.zig");
const imu = @import("imu.zig");
const receiver = @import("receiver.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;

// TODO: Make a EventGroup kind of struct, so you could listen for armed, failsafe
// etc. events directly.
pub var msg_status: Message(Status) = .{};
pub var msg_actuator_output: Message(void) = .{};

pub const Loop = struct {
    rcv_imu_data: Receiver(imu.Data) = undefined,
    rcv_tick: Receiver(void) = undefined,
    rcv_command: Receiver(Command) = undefined,

    command_received_time: ?time.Absolute = null,
    command: Command = .disarm,

    nav_command: ?RateCommand = null,

    arm_state: bool = false,
    failsafe_state: FailsafeState = .off,

    pub fn init(
        loop: *Loop,
        scheduler: *Scheduler,
    ) void {
        loop.* = .{};

        imu.msg_data.subscribe(
            &loop.rcv_imu_data,
            *Loop,
            loop,
            imu_tick,
            scheduler,
        );

        time.tickers.@"100Hz".subscribe(
            &loop.rcv_tick,
            *Loop,
            loop,
            nav_tick,
            scheduler,
        );

        receiver.msg_command.subscribe(
            &loop.rcv_command,
            *Loop,
            loop,
            handle_command,
            scheduler,
        );
    }

    fn imu_tick(loop: *Loop, data: imu.Data) void {
        _ = loop; // autofix
        _ = data; // autofix
    }

    fn nav_tick(control: *Loop, _: void) void {
        const MIN_COMMAND_PERIOD: time.Duration = .from_hz(5);
        const FAILSAFE_STAGE1_DURATION: time.Duration = .from_ms(500);

        const ANGLE_ROLL_KP = 0.5;
        const ANGLE_PITCH_KP = 0.5;

        const now = time.get_time_since_boot();

        var status_update: bool = false;

        if (control.command_received_time) |command_received_time| {
            if (!now.diff(command_received_time).less_than(MIN_COMMAND_PERIOD)) {
                status_update = true;
                control.failsafe_state = .{ .stage1 = now.add_duration(FAILSAFE_STAGE1_DURATION) };
            }
        }
        switch (control.failsafe_state) {
            .stage1 => |failsafe_at| if (failsafe_at.is_reached_by(now)) {
                status_update = true;
                control.failsafe_state = .failsafe;
            },
            else => {},
        }

        if (!control.arm_state and control.command != .disarm) {
            // if (checks)
            status_update = true;
            control.arm_state = true;
        } else if (control.arm_state and control.command == .disarm) {
            status_update = true;
            control.arm_state = false;
        }

        if (status_update) {
            msg_status.publish(.{
                .failsafe = control.failsafe_state == .failsafe,
                .arm = control.arm_state,
            });
        }

        const maybe_rate_command: ?Command = switch (control.command) {
            .angle => |angle| blk: {
                const attitude: math.Vec3 = fusion.msg_attitude.get() orelse
                    break :blk .disarm;

                break :blk .{ .rate = .{
                    .throttle = angle.throttle,
                    .roll = (angle.roll - attitude.roll) * ANGLE_ROLL_KP,
                    .pitch = (angle.pitch - attitude.pitch) * ANGLE_PITCH_KP,
                    .yaw = 0.0,
                } };
            },
            _ => null,
        };

        control.rate_command = maybe_rate_command;
    }

    fn handle_command(control: *Loop, command: Command) void {
        control.command = command;
        control.command_received_time = time.get_time_since_boot();
    }

    const FailsafeState = union(enum) {
        off,
        stage1: time.Absolute,
        failsafe,
    };
};

pub const Command = union(enum) {
    disarm,
    manual: struct {
        throttle: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
    },
    /// rad/s
    rate: RateCommand,
    /// rad
    angle: struct {
        throttle: f32,
        roll: f32,
        pitch: f32,
    },
};

pub const RateCommand = struct {
    /// rad/s
    throttle: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
};

pub const Status = packed struct {
    arm: bool,
    failsafe: bool,
};
