const std = @import("std");

const control = @import("control.zig");
const hw = @import("hw.zig");
const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const time = @import("time.zig");
const drivers = @import("drivers.zig");

const log = std.log.scoped(.imu);

pub var msg_data: Message(Data) = .{};
pub var msg_params: Message(Params) = .{};
pub var msg_calibrate: Message(void) = .{};

pub const Data = struct {
    timestamp: time.Absolute,
    gyro: math.Vec3,
    accel: math.Vec3,
};

pub const Params = extern struct {
    gyro_bias: math.Vec3,
    accel_bias: math.Vec3,
    accel_scale: math.Vec3,

    _board_forward_angle: f32,
    _imu_to_body: math.Mat3,

    pub const default: Params = .{
        .gyro_bias = .zero,
        .accel_bias = .zero,
        .accel_scale = .one,
        ._board_forward_angle = 0.0,
        ._imu_to_body = .identity,
    };
};

const Driver = switch (hw.def.imu.type) {
    .lsm6dsv => drivers.imu.lsm6dsv.Lsm6dsv(hw.SPI),
};

pub const Imu = struct {
    driver: Driver,
    params: Params = .default,

    rcv_params: Receiver(Params) = undefined,
    rcv_calibrate: Receiver(void) = undefined,

    calibrator: Calibrator = .disable,

    pub fn init(imu: *Imu, scheduler: *Scheduler) void {
        if (comptime hw.def.imu.tick_period != time.Duration.from_hz(1000))
            @compileError("imu tick period must be 1kHz for now");

        const driver = switch (hw.def.imu.type) {
            .lsm6dsv => Driver.init(.imu, &hw.clock, .{}) catch |err| {
                log.err("failed to init: {t}", .{err});
                return;
            },
        };

        imu.* = .{
            .driver = driver,
        };

        hw.InterruptPin.imu.subscribe(*Imu, imu, tick, scheduler);
        msg_params.subscribe(&imu.rcv_params, *Imu, imu, params_callback, scheduler);
        msg_calibrate.subscribe(&imu.rcv_calibrate, *Imu, imu, calibrate_callback, scheduler);
    }

    fn tick(imu: *Imu) void {
        const timestamp = time.get_time_since_boot();
        const raw_data = imu.driver.read() catch |err| {
            log.err("failed to read: {t}", .{err});
        };

        imu.calibrator.update(raw_data, &imu.params);

        var gyro = raw_data.gyro;
        gyro = .sub(gyro, imu.params.gyro_bias);
        gyro = imu.params._imu_to_body.transform(gyro);

        var accel = raw_data.accel;
        accel = .sub(accel, imu.params.accel_bias);
        accel = .mul(accel, imu.params.accel_scale);
        accel = imu.params._imu_to_body.transform(accel);

        msg_data.publish(.{
            .timestamp = timestamp,
            .gyro = gyro,
            .accel = accel,
        });
    }

    fn params_callback(imu: *Imu, params: Params) void {
        imu.params = params;
    }

    fn calibrate_callback(imu: *Imu, _: void) void {
        const arm_state = if (control.msg_status.get()) |status| status.arm else false;
        if (!arm_state) {
            imu.calibrator = .enable;
        } else {
            log.warn("skipping imu calibration because system is armed", .{});
            return;
        }
    }
};

pub const Calibrator = struct {
    enabled: bool,
    sample_count: u32,
    gyro_bias_accum: math.Vec3,
    accel_bias_accum: math.Vec3,

    pub const enable: Calibrator = .{
        .enabled = true,
        .sample_count = 0,
        .gyro_bias_accum = .zero,
        .accel_bias_accum = .zero,
    };

    pub const disable: Calibrator = .{
        .enabled = false,
        .sample_count = 0,
        .gyro_bias_accum = .zero,
        .accel_bias_accum = .zero,
    };

    pub fn update(calibrator: *Calibrator, data: Driver.Data, params: *const Params) void {
        const SAMPLE_COUNT: u32 = 1000;

        if (!calibrator.enabled) return;

        calibrator.gyro_bias_accum = .add(calibrator.gyro_bias_accum, data.gyro);
        calibrator.accel_bias_accum = .add(calibrator.accel_bias_accum, data.accel);
        calibrator.sample_count += 1;

        if (calibrator.sample_count >= SAMPLE_COUNT) {
            const gyro_mean: math.Vec3 = calibrator.gyro_bias_accum.div_scalar(@floatFromInt(calibrator.sample_count));
            const accel_mean: math.Vec3 = calibrator.accel_bias_accum.div_scalar(@floatFromInt(calibrator.sample_count));

            const down = accel_mean.normalize().negate();
            const front_ref = if (down.length() < 0.9) math.Vec3.axis(.x) else math.Vec3.axis(.z).negate();

            const board_rotation: math.Quaternion = .from_axis_angle(down, params._board_forward_angle);

            const unrotated_right = down.cross(front_ref).normalize();
            const right = board_rotation.rotate(unrotated_right);

            const front = right.cross(down).normalize();

            const imu_to_body: math.Mat3 = .init_from_rows(front, right, down);

            msg_params.publish(.{
                .gyro_bias = gyro_mean,
                .accel_bias = params.accel_bias,
                .accel_scale = params.accel_scale,
                ._imu_to_body = imu_to_body,
                ._board_forward_angle = params._board_forward_angle,
            });

            calibrator.* = .disable;
        }
    }
};
