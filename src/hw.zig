const std = @import("std");
const Map = std.EnumMap;

const microzig = @import("microzig");
const cpu = microzig.cpu;
const options = @import("options");

const Scheduler = @import("Scheduler.zig");
const Task = Scheduler.Task;
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const Waker = Scheduler.Waker;
const time = @import("time.zig");

// TODO: motors, servos and led_strip still require some thought in terms of api

pub const is_sim = options.board == .simulator;

pub const chip = switch (options.board) {
    .madflight_rp2350 => @import("chips/rp2350.zig"),
    .simulator => @import("chips/simulator.zig"),
};
pub const board = switch (options.board) {
    .madflight_rp2350 => @import("boards/madflight_rp2350.zig"),
    .simulator => @compileError("simulator doesn't have a board"),
};

pub const enter_critical_section = chip.enter_critical_section;
pub const CriticalSection = chip.CriticalSection;

pub const get_time_since_boot = chip.get_time_since_boot;

const Pin = chip.Pin;

pub const SchedulerPriority = enum {
    realtime,
    high,
    mid,
    low,
};

pub const TickRate = enum(u32) {
    @"1Hz" = 1,
    @"10Hz" = 10,
    @"50Hz" = 50,
    @"100Hz" = 100,
    @"500Hz" = 500,
    @"1kHz" = 1000,

    pub fn get_period(self: TickRate) time.Duration {
        return .from_hz(@backingInt(self));
    }
};

var tickers: std.EnumArray(TickRate, Message(void)) = .initFill(.{});
pub fn ticker(rate: TickRate) *Message(void) {
    return tickers.getPtr(rate);
}

pub const InterruptPin = enum {
    imu,
    battery,

    pub const Config = union(enum) {
        hw: Pin,
        sw: TickRate,
    };

    var wakers: std.EnumArray(InterruptPin, Task.Waker) = .initFill(.uninited);

    pub fn get_config(periodic: InterruptPin) ?Config {
        return switch (periodic) {
            .imu => if (def.imu.pin_interrupt) |pin|
                return .{ .hw = pin }
            else
                return .{ .sw = def.imu.tick_rate },
            .battery => if (def.battery) |battery|
                .{ .sw = battery.tick_rate } // TODO: interrupt pin
            else
                null,
        };
    }

    pub inline fn sw_wake(rate: TickRate) void {
        inline for (std.enums.values(InterruptPin)) |interrupt_pin| {
            const cfg = comptime interrupt_pin.get_config() orelse continue;
            switch (cfg) {
                .sw => |current_rate| if (current_rate == rate) {
                    wakers.getPtr(interrupt_pin).wake();
                },
                .hw => {},
            }
        }
    }

    pub inline fn hw_wake(pin: Pin) void {
        inline for (std.enums.values(InterruptPin)) |interrupt_pin| {
            const cfg = comptime interrupt_pin.get_config() orelse continue;
            switch (cfg) {
                .sw => {},
                .hw => |current_pin| if (current_pin == pin) {
                    wakers.getPtr(interrupt_pin).wake();
                },
            }
        }
    }

    pub fn subscribe(
        interrupt_pin: InterruptPin,
        Context: type,
        context: Context,
        comptime callback: fn (Context) void,
        scheduler: *Scheduler,
    ) void {
        wakers.getPtr(interrupt_pin).register(Context, context, struct {
            pub fn wrapper(ctx: Context, _: *Task.Waker) void {
                callback(ctx);
            }
        }.wrapper, scheduler);
    }
};

pub const Clock = chip.Clock;
pub const Flash = chip.Flash;

pub const UART_Instance = enum {
    receiver,
    gps,

    pub fn get_config(instance: UART_Instance) ?UART.Config {
        return switch (instance) {
            .receiver => def.receiver.uart,
            .gps => null, // TODO: add gps support
        };
    }
};
pub const UART = chip.UART;

pub const SPI_Instance = enum {
    imu,
    sd_card,

    pub fn get_config(instance: SPI_Instance) ?SPI.Config {
        return switch (instance) {
            .imu => def.imu.spi,
            .sd_card => null, // TODO: add sd card support
        };
    }
};
pub const SPI = chip.SPI;

pub const I2C_Instance = enum {
    sensor,

    pub fn get_config(instance: I2C_Instance) ?I2C.Config {
        return switch (instance) {
            .sensor => def.i2c_sensor,
        };
    }
};
pub const I2C = chip.I2C;

pub const MotorConfig = chip.MotorConfig;
pub const motors = chip.motors;

pub const ServoConfig = chip.ServoConfig;
pub const servos = chip.servos;

pub const LedStripConfig = chip.LedStripConfig;
pub const led_strip = chip.led_strip;

pub const def: Definition = board.hw_def;

pub const Definition = struct {
    motors: struct {
        protocol: enum {
            dshot_300,
        },
        outputs: []const MotorConfig,
    },
    servos: []const ServoConfig,

    imu: struct {
        tick_rate: TickRate,
        type: enum {
            lsm6dsv,
        },
        spi: SPI.Config,
        pin_interrupt: ?Pin = null,
    },

    receiver: struct {
        protocol: enum {
            crsf,
        },
        uart: UART.Config,
    },

    i2c_sensor: I2C.Config,

    battery: ?struct {
        tick_rate: TickRate,
        type: enum {
            ina226,
        },
        shunt_resistance: f32,
        max_current: f32,
    } = null,

    led_strip: ?struct {
        count: usize,
        config: LedStripConfig,
    } = null,

    flash: Flash.Config,
};

pub const CPU_Usage = struct {
    last_tick_ticks: u32,
    idle_ticks: u32,
    rcv_tick: Receiver(void) = undefined,

    pub fn init(cpu_usage: *CPU_Usage, scheduler: *Scheduler) void {
        cpu_usage.* = .{
            .last_tick_ticks = 0,
            .idle_ticks = 0,
        };

        // Enable trace
        cpu.peripherals.dcb.DEMCR.modify(.{ .TRCENA = 1 });

        // Reset counters
        cpu.peripherals.dwt.CYCCNT = 0;
        cpu.peripherals.dwt.EXCCNT = 0;

        // Enable cycle counter
        cpu.peripherals.dwt.CTRL.modify(.{ .CYCCNTENA = 1 });

        // Set SEVONPEND flag so that interrupts set the event flag even inside
        // a critical section.
        cpu.peripherals.scb.SCR.modify(.{ .SEVONPEND = 1 });

        ticker(.@"1Hz").subscribe(&cpu_usage.rcv_tick, *CPU_Usage, cpu_usage, tick_callback, scheduler);
    }

    fn tick_callback(cpu_usage: *CPU_Usage, _: void) void {
        const ticks_now = cpu.peripherals.dwt.CYCCNT;
        const tick_diff = ticks_now -% cpu_usage.last_tick_ticks;

        if (tick_diff != 0) {
            const idle_u64: u64 = cpu_usage.idle_ticks;
            const diff_u64: u64 = tick_diff;

            const idle_percent = (idle_u64 * 100) / diff_u64;
            const usage_percent = @as(u64, 100) -| idle_percent;

            std.log.scoped(.usage).info("cpu usage: {d}%", .{usage_percent});
        }

        // NOTE: No need for a cs. This function is called from the highest
        // priority accessing these fields.
        cpu_usage.last_tick_ticks = ticks_now;
        cpu_usage.idle_ticks = 0;
    }

    /// Must only be called from a thread context.
    pub fn on_idle(cpu_usage: *CPU_Usage) void {
        const cs = enter_critical_section();
        defer cs.leave();

        const before = cpu.peripherals.dwt.CYCCNT;
        cpu.wfe();
        const after = cpu.peripherals.dwt.CYCCNT;
        cpu_usage.idle_ticks +%= (after -% before);
    }
};
