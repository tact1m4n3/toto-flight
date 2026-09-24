const std = @import("std");
const microzig = @import("microzig");

const actuator = @import("actuator.zig");
const battery = @import("battery.zig");
const control = @import("control.zig");
const hw = @import("hw.zig");
const imu = @import("imu.zig");
const receiver = @import("receiver.zig");
const Scheduler = @import("Scheduler.zig");
const status_led = @import("status_led.zig");
const storage = @import("storage.zig");

const log = std.log.scoped(.main);

pub const std_options = microzig.std_options(.{
    .log_level = .debug,
    .logFn = hw.chip.log_fn,
});
pub const microzig_options: microzig.Options = .{
    .interrupts = hw.chip.interrupts,
};
pub const panic = microzig.panic;
comptime {
    _ = microzig.export_startup();
}

pub var scheduler_realtime: Scheduler = .init(hw.chip.scheduler_get_pend_fn(.realtime));
pub var scheduler_high: Scheduler = .init(hw.chip.scheduler_get_pend_fn(.high));
pub var scheduler_mid: Scheduler = .init(hw.chip.scheduler_get_pend_fn(.mid));
pub var scheduler_low: Scheduler = .init(hw.chip.scheduler_get_pend_fn(.low));

// var task_storage: storage.Storage = undefined;

var task_imu: imu.Imu = undefined;
var task_control: control.Loop = undefined;
var task_actuator: actuator.Actuator = undefined;

var task_rx: receiver.Rx = undefined;
var task_channel_mapper: receiver.ChannelMapper = undefined;

var task_battery: battery.Battery = undefined;
var task_status_led: status_led.StatusLed = undefined;

var task_cpu_usage: hw.CPU_Usage = undefined;

pub fn main() void {
    {
        const cs = hw.enter_critical_section();
        defer cs.leave();

        hw.chip.init();

        log.info("initializing tasks", .{});

        // task_storage.init(&scheduler_low);

        task_cpu_usage.init(&scheduler_realtime);
        task_imu.init(&scheduler_realtime);
        task_control.init(&scheduler_realtime);
        task_actuator.init(&scheduler_realtime);

        task_rx.init(&scheduler_high);
        task_channel_mapper.init(&scheduler_high);

        // NOTE: all i2c sensors must be on the same scheduler
        if (hw.def.battery) |_| {
            task_battery.init(&scheduler_mid);
        }

        if (hw.def.led_strip) |_| {
            task_status_led.init(&scheduler_low);
        }
    }

    log.info("setup done. main going idle...", .{});

    while (true) {
        task_cpu_usage.on_idle();
    }
}
