const std = @import("std");

const control = @import("control.zig");
const hw = @import("hw.zig");
const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Task = Scheduler.Task;
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const parameter = @import("parameter.zig");
const time = @import("time.zig");
const drivers = @import("drivers.zig");

const log = std.log.scoped(.imu);

pub var arm_block_calibrating: control.ArmBlock = .init;
pub var msg_data: Message(Data) = .{};
pub var msg_calibrate: Message(void) = .{};

pub const Params = extern struct {
    gyr_bias: math.Vec3,
    acc_bias: math.Vec3,
    acc_scale: math.Vec3,
    _to_body: math.Mat3,

    pub const default: Params = .{
        .gyr_bias = .zero,
        .acc_bias = .zero,
        .acc_scale = .one,
        ._to_body = .identity,
    };
};

pub const Data = struct {
    ts: time.Absolute,
    /// rad/s
    gyro: math.Vec3,
    /// m/s^2
    accel: math.Vec3,
};

const Driver = switch (hw.def.imu.type) {
    .lsm6dsv => drivers.imu.lsm6dsv.Lsm6dsv(hw.SPI),
};

pub const Imu = struct {
    rcv_calibrate: Receiver(void) = undefined,

    driver: Driver,
    calibrator: Calibrator = .init,

    pub fn init(imu: *Imu, scheduler: *Scheduler) void {
        const driver = switch (hw.def.imu.type) {
            .lsm6dsv => Driver.init(hw.SPI.get(.imu), hw.Clock.instance, .{}) catch |err| {
                log.err("failed to init: {t}", .{err});
                return;
            },
        };

        imu.* = .{
            .driver = driver,
        };

        hw.InterruptPin.imu.subscribe(*Imu, imu, read_sample_callback, scheduler);
        msg_calibrate.subscribe(&imu.rcv_calibrate, *Imu, imu, calibrate_callback, scheduler);
    }

    fn read_sample_callback(imu: *Imu) void {
        const ts = hw.get_time_since_boot();
        const raw_data = imu.driver.read() catch |err| {
            log.err("failed to read sample: {t}", .{err});
        };

        const params = parameter.get(struct {
            imu: Params,
        });

        if (imu.calibrator.update(raw_data)) {
            arm_block_calibrating.release();
        }

        var gyro = raw_data.gyro;
        gyro = .sub(gyro, params.imu.gyr_bias);
        gyro = params.imu._to_body.transform(gyro);
        gyro = .mul_scalar(gyro, comptime math.radians_from_degrees(1.0));

        var accel = raw_data.accel;
        accel = .sub(accel, params.imu.acc_bias);
        accel = .mul(accel, params.imu.acc_scale);
        accel = params.imu._to_body.transform(accel);
        accel = .mul_scalar(accel, 9.80665);

        msg_data.publish(.{
            .ts = ts,
            .gyro = gyro,
            .accel = accel,
        });
    }

    fn calibrate_callback(imu: *Imu, _: void) void {
        arm_block_calibrating.acquire() catch |err| {
            log.warn("skipping calibration: {}", .{err});
            return;
        };
        imu.calibrator = .enable;
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

    pub const init: Calibrator = .{
        .enabled = false,
        .sample_count = 0,
        .gyro_bias_accum = .zero,
        .accel_bias_accum = .zero,
    };

    pub fn update(calibrator: *Calibrator, data: Driver.Data) bool {
        const SAMPLE_COUNT: u32 = 1000;

        if (!calibrator.enabled) return false;

        calibrator.gyro_bias_accum = .add(calibrator.gyro_bias_accum, data.gyro);
        calibrator.accel_bias_accum = .add(calibrator.accel_bias_accum, data.accel);
        calibrator.sample_count += 1;

        if (calibrator.sample_count >= SAMPLE_COUNT) {
            const gyro_mean: math.Vec3 = calibrator.gyro_bias_accum.div_scalar(@floatFromInt(calibrator.sample_count));
            const accel_mean: math.Vec3 = calibrator.accel_bias_accum.div_scalar(@floatFromInt(calibrator.sample_count));

            const down = accel_mean.normalize().negate();
            const front_ref = if (down.length() < 0.9) math.Vec3.axis(.x) else math.Vec3.axis(.z).negate();

            const board_forward_angle = parameter.get(struct {
                cor: struct {
                    fwd_angl: f32,
                },
            }).cor.fwd_angl;

            const board_rotation: math.Quaternion = .from_axis_angle(down, board_forward_angle);

            const unrotated_right = down.cross(front_ref).normalize();
            const right = board_rotation.rotate(unrotated_right);

            const front = right.cross(down).normalize();

            const imu_to_body: math.Mat3 = .init_from_rows(front, right, down);

            parameter.modify(.{ .imu = .{
                .gyro_bias = gyro_mean,
                ._imu_to_body = imu_to_body,
            } });

            calibrator.* = .init;

            return true;
        } else {
            return false;
        }
    }
};
