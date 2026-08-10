const std = @import("std");

const hw = @import("hw.zig");
const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const time = @import("time.zig");

const log = std.log.scoped(.imu);

pub const Data = struct {
    timestamp: time.Absolute,
    gyro: math.Vec3,
    accel: math.Vec3,
};

pub const Params = struct {
    gyro_bias: math.Vec3,
    accel_bias: math.Vec3,
    accel_scale: math.Vec3,

    _imu_to_body: math.Mat3,

    pub const default: Params = .{
        .gyro_bias = .zero,
        .accel_bias = .zero,
        .accel_scale = .one,
        ._imu_to_body = .identity,
    };
};

const Driver = switch (hw.def.imu.type) {
    .lsm6dsv => @import("drivers/imu/lsm6dsv.zig").Lsm6dsv(hw.SPI),
};

pub const Imu = struct {
    driver: Driver,
    msg_data: *Message(Data),
    rcv_params: Receiver(Params) = undefined,
    params: Params = .default,

    pub const Resources = struct {
        spi: hw.SPI,
        interrupt_pin: hw.InterruptPin,
        msg_data: *Message(Data),
        msg_params: *Message(Params),
    };

    pub fn init(
        imu: *Imu,
        scheduler: *Scheduler,
        resources: Resources,
    ) void {
        if (comptime hw.def.imu.tick_period != time.Duration.from_hz(1000))
            @compileError("imu tick period must be 1kHz for now");

        const driver = switch (hw.def.imu.type) {
            .lsm6dsv => Driver.init(resources.spi, hw.clock, .{}) catch |err| {
                log.err("failed to init: {t}", .{err});
                return;
            },
        };

        imu.* = .{
            .driver = driver,
            .msg_data = resources.msg_data,
        };

        resources.msg_params.subscribe(&imu.rcv_params, *Imu, imu, params_callback, scheduler);
        resources.interrupt_pin.subscribe(*Imu, imu, tick, scheduler);
    }

    fn params_callback(imu: *Imu, params: Params) void {
        imu.params = params;
    }

    fn tick(imu: *Imu) void {
        const timestamp = hw.get_time_since_boot();
        const raw_data = imu.driver.read() catch |err| {
            log.err("failed to read: {t}", .{err});
        };

        var gyro = raw_data.gyro;
        gyro = .sub(gyro, imu.params.gyro_bias);
        gyro = .transform(gyro, imu.params._imu_to_body);

        var accel = raw_data.accel;
        accel = .sub(accel, imu.params.accel_bias);
        accel = .component_mul(accel, imu.params.accel_scale);
        accel = .transform(accel, imu.params._imu_to_body);

        imu.msg_data.publish(.{
            .timestamp = timestamp,
            .gyro = gyro,
            .accel = accel,
        });
    }
};
