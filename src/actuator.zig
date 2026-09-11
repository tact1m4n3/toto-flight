const std = @import("std");

const hw = @import("hw.zig");
const Scheduler = @import("Scheduler.zig");
const ParamTable = Scheduler.ParamTable;
const Receiver = Scheduler.Receiver;
const control = @import("control.zig");
const math = @import("math.zig");

var param_table: ParamTable(Params) = .{};

pub const Actuator = struct {
    rcv_output: Receiver(control.ActuatorOutput) = undefined,

    pub fn init(actuator: *Actuator, scheduler: *Scheduler) void {
        actuator.* = .{};

        control.msg_actuator_output.subscribe(
            &actuator.rcv_output,
            *Actuator,
            actuator,
            command_callback,
            scheduler,
        );
    }

    pub fn command_callback(_: *Actuator, output: control.ActuatorOutput) void {
        // if we don't have params, don't output anything
        const params: Params = param_table.get() orelse .wing;

        if (output.throttle > 0.0) {
            var values_motor: [hw.motors.count]f32 = undefined;
            for (&values_motor, &params.mtr) |*value_ptr, *params_motor| {
                const mix = params_motor.mix.clamp(-1.0, 1.0);

                const mixed = math.Vec3.dot(mix, output.throw);
                const mixed_clamped = std.math.clamp(mixed, 0.0, 1.0);
                value_ptr.* = mixed_clamped;
            }
            hw.motors.write(&values_motor);
        } else {
            hw.motors.disarm();
        }

        var values_servo: [hw.servos.count]u16 = undefined;
        for (&values_servo, &params.srv) |*value_ptr, *params_servo| {
            const mix = params_servo.mix.clamp(-1.0, 1.0);
            const mid = std.math.clamp(params_servo.mid, 1000, 2000);
            const thr = @min(params_servo.thr, mid);

            const throw = output.throw.clamp(-1.0, 1.0);
            const mixed = math.Vec3.dot(mix, throw);
            const mixed_clamped = std.math.clamp(mixed, -1.0, 1.0);
            const throw_us: i16 = @round(
                @as(f32, @floatFromInt(thr)) * mixed_clamped,
            );
            const value: u16 = @intCast(@as(i16, @intCast(mid)) + throw_us);
            value_ptr.* = value;
        }

        hw.servos.write(&values_servo);
    }
};

pub const Params = extern struct {
    mtr: [hw.motors.count]extern struct {
        // TODO: throttle curve
        mix: math.Vec3,
    },
    srv: [hw.servos.count]extern struct {
        mid: u16,
        thr: u16,
        mix: math.Vec3,
    },

    pub const wing: Params = .{
        .mtr = @splat(.{ .mix = .zero }),
        .srv = .{
            .{
                .mid = 1700,
                .thr = 200,
                .mix = .{ .x = -1.0, .y = -1.0, .z = 0.0 },
            },
            .{
                .mid = 1300,
                .thr = 200,
                .mix = .{ .x = -1.0, .y = 1.0, .z = 0.0 },
            },
        },
    };
};
