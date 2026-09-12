const std = @import("std");

const hw = @import("hw.zig");
const control = @import("control.zig");
const Scheduler = @import("Scheduler.zig");
const Receiver = Scheduler.Receiver;
const time = @import("time.zig");
const drivers = @import("drivers.zig");

pub const StatusLed = struct {
    counter: usize = 0,
    lit: bool = true,
    rcv_tick: Receiver(time.Absolute) = undefined,

    pub fn init(status_led: *StatusLed, scheduler: *Scheduler) void {
        status_led.* = .{};

        hw.Ticker.@"10Hz".subscribe(
            &status_led.rcv_tick,
            *StatusLed,
            status_led,
            tick_callback,
            scheduler,
        );
    }

    pub fn tick_callback(status_led: *StatusLed, _: time.Absolute) void {
        const status = control.msg_status.get() orelse return;

        const color: drivers.Color = if (status.armed)
            .dark_red
        else
            .dark_blue;

        const period: usize = if (status.failsafe)
            2
        else
            5;

        if (status_led.counter % period == 0) {
            if (status_led.lit) {
                hw.led_strip.write(color);
            } else {
                hw.led_strip.write(.black);
            }
            status_led.lit = !status_led.lit;
        }

        status_led.counter += 1;
    }
};
