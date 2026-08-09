const std = @import("std");
const log = std.log.scoped(.imu);

const Scheduler = @import("Scheduler.zig");
const hw = @import("hw.zig");
const math = @import("math.zig");
const time = @import("time.zig");

pub const Data = struct {
    gyro: math.Vec3,
    accel: math.Vec3,
};

const Driver = switch (hw.def.imu.type) {
    .lsm6dsv => @import("drivers/imu/lsm6dsv.zig").Lsm6dsv(hw.SPI),
};

pub const Imu = struct {
    driver: Driver,

    pub fn init(imu: *Imu, scheduler: *Scheduler, spi: hw.SPI, interrupt_pin: hw.InterruptPin) void {
        if (hw.def.imu.tick_period != time.Duration.from_hz(1000))
            @compileError("imu tick period must be 1kHz for now");
        const driver: Driver = .init(spi, hw.ClockDevice, .{}) catch |err| {
            std.log.err("failed to init: {t}", .{err});
            return;
        };
        imu.* = .{ .driver = driver };

        interrupt_pin.subscribe(*Imu, imu, tick, scheduler);
    }

    fn tick(imu: *Imu) void {
        const raw_data = imu.driver.read() catch |err| {
            std.log.err("failed to read: {t}", .{err});
        };
        _ = raw_data;
    }
};

// pub const Params = struct {
//     gyro_bias: math.Vec3,
//     accel_bias: math.Vec3,
//     accel_scale: math.Vec3,
//
//     _imu_to_body: math.Mat3,
// };
//
// pub var params: Params = .{
//     .gyro_bias = .zero,
//     .accel_bias = .zero,
//     .accel_scale = .one,
//     ._imu_to_body = .identity,
// };
//
// const sensor = hw.def.imu;
//
// pub fn init() void {
//     sensor.init();
//
//     // imu.blocking_init();
//     imu_interrupt_pin.on_rising_edge(query_data);
// }
//
// pub fn query_data(instant: Absolute) void {
//     sensor.read_async(process_data, .{ .timestamp = instant });
// }
//
// pub fn process_data(raw_data: ImuRawData) void {
//     const raw_data = imu.read() catch continue;
//
//     var gyro = raw_data.gyro;
//     gyro = .sub(gyro, params.gyro_bias);
//     gyro = .transform(gyro, params._imu_to_body);
//
//     var accel = raw_data.accel;
//     accel = .sub(accel, params.accel_bias);
//     accel = .component_mul(accel, params.accel_scale);
//     accel = .transform(accel, params._imu_to_body);
// }
//
// pub fn task(rt: *RT) noreturn {
//     if (def.tick_period != time.Duration.from_hz(1000))
//         @compileError("imu tick period must be 1kHz for now");
//     const SPI_Device = hw.SPI_Device(def.spi);
//     var clock_dev: hw.ClockDevice = .init(rt);
//     var imu = switch (hw.def.imu.type) {
//         .lsm6dsv => drivers.imu.lsm6dsv.Lsm6dsv(SPI_Device).init(
//             SPI_Device.init(rt) catch @panic("failed to init imu device"),
//             &clock_dev,
//             .{},
//         ) catch @panic("failed to init imu"),
//     };
//
//     // TODO: motor and servo hw
//
//     while (true) {
//         const sample_ts = hw.wait_for_imu_interrupt(rt);
//         _ = sample_ts;
//     }
// }
