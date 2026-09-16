const std = @import("std");

const hw = @import("hw.zig");
const Scheduler = @import("Scheduler.zig");
const Task = Scheduler.Task;
const Message = Scheduler.Message;
const time = @import("time.zig");
const drivers = @import("drivers.zig");

const log = std.log.scoped(.battery);

pub var msg_data: Message(Data) = .{};

pub const Data = struct {
    /// V
    voltage: f32,
    /// A
    current: f32,
    /// mAh
    capacity_used: f32,
};

const cfg = if (hw.def.battery) |battery_cfg|
    battery_cfg
else
    @compileError("battery configuration not available");

const Driver = switch (cfg.type) {
    .ina226 => drivers.battery.ina226.INA_226(hw.I2C),
};

pub const Battery = struct {
    driver: Driver,
    last_tick: ?time.Absolute = null,
    prev_current: f32 = 0.0,
    capacity_used: f32 = 0.0,

    pub fn init(battery: *Battery, scheduler: *Scheduler) void {
        battery.* = .{
            .driver = switch (cfg.type) {
                .ina226 => Driver.init(hw.I2C.get(.sensor), drivers.battery.ina226.DEFAULT_SLAVE_ADDRESS, hw.Clock.instance, .{
                    .shunt_resistance = cfg.shunt_resistance,
                    .current_lsb = cfg.max_current / 32768.0,
                }) catch |err| {
                    log.warn("failed to init battery sensor: {t}", .{err});
                    return;
                },
            },
        };

        hw.InterruptPin.battery.subscribe(
            *Battery,
            battery,
            tick_callback,
            scheduler,
        );
    }

    pub fn tick_callback(battery: *Battery) void {
        const now = hw.get_time_since_boot();
        const last_tick = battery.last_tick orelse now;
        const dt = now.diff(last_tick);
        battery.last_tick = now;

        const voltage, const current = switch (cfg.type) {
            .ina226 => blk: {
                const voltage = battery.driver.read_bus_voltage() catch |err| {
                    log.warn("failed to read voltage: {}", .{err});
                    return;
                };
                const current = battery.driver.read_current() catch |err| {
                    log.warn("failed to read voltage: {}", .{err});
                    return;
                };
                break :blk .{ voltage, current };
            },
        };

        {
            const dt_s = dt.to_secs_f32();
            const dt_h = dt_s / 3600.0;
            const instant_current_A = (battery.prev_current + current) / 2.0;
            const instant_current_mA = instant_current_A * 1000.0;
            battery.capacity_used += instant_current_mA * dt_h;
            battery.prev_current = current;
        }

        msg_data.publish(.{
            .voltage = voltage,
            .current = current,
            .capacity_used = battery.capacity_used,
        });
    }
};
