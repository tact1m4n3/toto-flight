const std = @import("std");

const microzig = @import("microzig");
const rp2xxx = microzig.hal;
pub const Pin = rp2xxx.gpio.Pin;

const hw = @import("../hw.zig");
const imu = @import("../imu.zig");
const receiver = @import("../receiver.zig");
const Scheduler = @import("../Scheduler.zig");
const Message = Scheduler.Message;
const Task = Scheduler.Task;
const time = @import("../time.zig");
const Duration = time.Duration;
const Absolute = time.Absolute;
const RingBuffer = @import("../utils/ring_buffer.zig").RingBuffer;

const log = std.log.scoped(.chip_rp2350);

pub const interrupts: microzig.InterruptOptions = .{
    .IO_IRQ_BANK0 = .{ .c = IO_IRQ_BANK0 },
    .UART0_IRQ = UART0_IRQ,
    .UART1_IRQ = UART1_IRQ,
    .SPAREIRQ_IRQ_0 = .{ .c = SPAREIRQ_IRQ_0 },
    .SPAREIRQ_IRQ_1 = .{ .c = SPAREIRQ_IRQ_1 },
    .SPAREIRQ_IRQ_2 = .{ .c = SPAREIRQ_IRQ_2 },
    .SPAREIRQ_IRQ_3 = .{ .c = SPAREIRQ_IRQ_3 },
};

var task_imu: imu.Imu = undefined;
var task_rx: receiver.Rx = undefined;

pub fn main() noreturn {
    RTT.init();
    rtt_logger = RTT.writer(0, &rtt_writer_buf);

    schedulers_init();

    InterruptPin.apply_all();
    UART.apply_all();
    SPI.apply_all();

    task_imu.init(&scheduler_realtime_priority);
    task_rx.init(&scheduler_high_priority);

    microzig.cpu.interrupt.enable_interrupts();

    while (true) {
        microzig.cpu.wfi();
    }
}

const RTT = microzig.cpu.rtt.RTT(.{});
var rtt_logger: ?RTT.Writer = null;
var rtt_writer_buf: [128]u8 = undefined;

pub fn log_fn(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    const level_prefix = comptime "[{}.{:0>6}] " ++ level.asText();
    const prefix = comptime level_prefix ++ switch (scope) {
        .default => ": ",
        else => " (" ++ @tagName(scope) ++ "): ",
    };
    if (rtt_logger) |*writer| {
        const current_time = hw.get_time_since_boot();
        const seconds = current_time.to_us() / std.time.us_per_s;
        const microseconds = current_time.to_us() % std.time.us_per_s;
        writer.interface.print(prefix ++ format ++ "\r\n", .{ seconds, microseconds } ++ args) catch {};
        writer.interface.flush() catch {};
    }
}

pub fn enter_critical_section() CriticalSection {
    const enable_on_leave = microzig.cpu.interrupt.globally_enabled();
    microzig.cpu.interrupt.disable_interrupts();
    return .{
        .enable_on_leave = enable_on_leave,
    };
}
pub const CriticalSection = struct {
    enable_on_leave: bool,
    pub fn leave(cs: CriticalSection) void {
        if (cs.enable_on_leave) {
            microzig.cpu.interrupt.enable_interrupts();
        }
    }
};

pub fn get_time_since_boot() Absolute {
    return .from_us(rp2xxx.time.get_time_since_boot().to_us());
}

const scheduler_priority_realtime: microzig.cpu.interrupt.Priority = @enumFromInt(0);
const scheduler_priority_high: microzig.cpu.interrupt.Priority = @enumFromInt(1);
const scheduler_priority_mid: microzig.cpu.interrupt.Priority = @enumFromInt(2);
const scheduler_priority_low: microzig.cpu.interrupt.Priority = @enumFromInt(3);

var scheduler_realtime_priority: Scheduler = .init(scheduler_pend_fn(.SPAREIRQ_IRQ_0));
var scheduler_high_priority: Scheduler = .init(scheduler_pend_fn(.SPAREIRQ_IRQ_1));
var scheduler_mid_priority: Scheduler = .init(scheduler_pend_fn(.SPAREIRQ_IRQ_2));
var scheduler_low_priority: Scheduler = .init(scheduler_pend_fn(.SPAREIRQ_IRQ_3));

fn schedulers_init() void {
    inline for (&.{
        .SPAREIRQ_IRQ_0,
        .SPAREIRQ_IRQ_1,
        .SPAREIRQ_IRQ_2,
        .SPAREIRQ_IRQ_3,
    }, &.{
        scheduler_priority_realtime,
        scheduler_priority_high,
        scheduler_priority_mid,
        scheduler_priority_low,
    }) |interrupt, priority| {
        microzig.cpu.interrupt.set_priority(interrupt, priority);
        microzig.cpu.interrupt.clear_pending(interrupt);
        microzig.cpu.interrupt.enable(interrupt);
    }
}

fn scheduler_pend_fn(comptime interrupt: microzig.cpu.ExternalInterrupt) fn () void {
    return struct {
        fn pend_fn() void {
            microzig.cpu.interrupt.set_pending(interrupt);
        }
    }.pend_fn;
}

fn SPAREIRQ_IRQ_0() callconv(.c) void {
    microzig.cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_0);
    scheduler_realtime_priority.run();
}

fn SPAREIRQ_IRQ_1() callconv(.c) void {
    microzig.cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_1);
    scheduler_high_priority.run();
}

fn SPAREIRQ_IRQ_2() callconv(.c) void {
    microzig.cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_2);
    scheduler_mid_priority.run();
}

fn SPAREIRQ_IRQ_3() callconv(.c) void {
    microzig.cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_3);
    scheduler_low_priority.run();
}

pub const InterruptPin = enum(u6) {
    imu = @intFromEnum(hw.def.imu.pin_interrupt),

    pub fn apply_all() void {
        inline for (std.enums.values(InterruptPin)) |interrupt_pin| {
            const pin: rp2xxx.gpio.Pin = @enumFromInt(@intFromEnum(interrupt_pin));
            pin.set_function(.sio);
            pin.set_direction(.in);
            pin.set_pull(.down);
            pin.set_irq_enabled(.{ .rise = 1 }, true);
            microzig.cpu.interrupt.clear_pending(.IO_IRQ_BANK0);
            microzig.cpu.interrupt.enable(.IO_IRQ_BANK0);
        }
    }

    pub fn subscribe(
        pin: InterruptPin,
        Context: type,
        context: Context,
        comptime callback: fn (context: Context) void,
        scheduler: *Scheduler,
    ) void {
        const task = interrupt_pin_task_map.getPtr(pin);
        task.init_late(Context, context, struct {
            pub fn wrapper(ctx: Context, _: *Task) void {
                callback(ctx);
            }
        }.wrapper, scheduler);
    }
};

var interrupt_pin_task_map: std.EnumArray(InterruptPin, Task) = .initFill(.uninitialized);

fn IO_IRQ_BANK0() linksection(".ram_text") callconv(.c) void {
    var it: rp2xxx.gpio.IrqEventIter = .{};
    while (it.next()) |trigger| {
        if (trigger.events.rise == 1) {
            if (std.enums.fromInt(InterruptPin, @intFromEnum(trigger.pin))) |pin| {
                interrupt_pin_task_map.getPtr(pin).ready();
            }
        }
    }
}

pub const UART_Config = struct {
    instance: Instance,
    baud_rate: u32,
    pin_tx: Pin,
    pin_rx: Pin,
    buf_size_tx: usize,
    buf_size_rx: usize,

    pub const Instance = union(enum) {
        uart: rp2xxx.uart.UART,
        // pio: struct {
        //     pio: rp2xxx.pio.Pio,
        //     sm: rp2xxx.pio.Pio,
        // },
    };
};

pub const UART = enum {
    receiver,

    fn apply_all() void {
        inline for (std.enums.values(UART)) |uart| {
            uart.get_state().apply();
        }
    }

    fn from_instance(comptime instance: UART_Config.Instance) ?UART {
        return inline for (std.enums.values(UART)) |uart| {
            if (std.meta.eql(uart.get_config().instance, instance)) {
                break uart;
            }
        } else null;
    }

    fn get_state(comptime uart: UART) *UART_State(uart.get_config()) {
        return &struct {
            var state: UART_State(uart.get_config()) = .{};
        }.state;
    }

    fn get_config(comptime uart: UART) UART_Config {
        return switch (uart) {
            .receiver => hw.def.receiver.uart,
        };
    }
};

pub const UART_TX = struct {
    inner: UART,

    pub fn write_byte(uart_tx: UART_TX, byte: u8) !void {
        try uart_tx.inner.get_state().write_byte(byte);
    }
};

pub const UART_RX = struct {
    inner: UART,

    pub const receiver: UART_RX = .{ .inner = .receiver };

    /// Thread safe. Idempotent.
    pub fn subscribe(
        uart_rx: UART_RX,
        Context: type,
        context: Context,
        comptime callback: fn (Context, u8) void,
        scheduler: *Scheduler,
    ) void {
        switch (uart_rx.inner) {
            inline else => |uart_comptime| uart_comptime.get_state().subscribe(
                Context,
                context,
                callback,
                scheduler,
            ),
        }
    }
};

pub fn UART_State(cfg: UART_Config) type {
    return struct {
        const Self = @This();

        // TODO: maybe just use interrupts instead of dma

        const RxError = rp2xxx.uart.ReceiveError;
        tx_ring_buf: RingBuffer(u8, cfg.buf_size_tx) = .{},
        rx_ring_buf: RingBuffer(u8, cfg.buf_size_rx) = .{},
        rx_task: Task = .uninitialized,

        pub fn apply(_: *Self) void {
            cfg.pin_tx.set_function(.uart);
            cfg.pin_rx.set_function(.uart);

            switch (cfg.instance) {
                .uart => |uart| {
                    uart.apply(.{
                        .clock_config = rp2xxx.clock_config,
                        .baud_rate = cfg.baud_rate,
                    });
                    uart.set_interrupts_enabled(.{
                        .rx = true,
                        .tx = true,
                        .fe = true,
                        .pe = true,
                        .be = true,
                        .oe = true,
                    });
                    switch (uart) {
                        .num(0) => {
                            microzig.cpu.interrupt.clear_pending(.UART0_IRQ);
                            microzig.cpu.interrupt.enable(.UART0_IRQ);
                        },
                        .num(1) => {
                            microzig.cpu.interrupt.clear_pending(.UART1_IRQ);
                            microzig.cpu.interrupt.enable(.UART1_IRQ);
                        },
                        _ => @compileError("invalid uart"),
                    }
                },
            }
        }

        /// Thread safe. Idempotent.
        pub fn subscribe(
            state: *Self,
            Context: type,
            context: Context,
            comptime callback: fn (Context, u8) void,
            scheduler: *Scheduler,
        ) void {
            state.rx_task.init_late(Context, context, struct {
                pub fn wrapper(ctx: Context, task: *Task) void {
                    const s: *Self = @fieldParentPtr("rx_task", task);
                    while (s.rx_ring_buf.pop()) |byte| {
                        callback(ctx, byte);
                    }
                }
            }.wrapper, scheduler);
        }

        pub fn write_byte(state: *Self, byte: u8) !void {
            try state.tx_ring_buf.push(byte);
        }

        // Inline because we want it in .ram_text
        pub inline fn on_interrupt(state: *Self) void {
            switch (cfg.instance) {
                .uart => |uart| {
                    var read_flag: bool = false;
                    while (true) {
                        const maybe_byte = uart.read_word() catch {
                            // log.warn("uart: failed to read byte: {t}", .{err});
                            uart.clear_errors();
                            continue;
                        };

                        if (maybe_byte) |byte| {
                            state.rx_ring_buf.push(byte) catch {};
                            read_flag = true;
                        } else {
                            break;
                        }
                    }
                    if (read_flag) state.rx_task.ready();

                    while (uart.is_writeable()) {
                        if (state.tx_ring_buf.pop()) |byte| {
                            uart.get_regs().UARTDR.write_raw(byte);
                        } else {
                            break;
                        }
                    }
                },
            }
        }
    };
}

pub const UART0_IRQ: ?microzig.interrupt.Handler = if (UART.from_instance(.{ .uart = .num(0) })) |uart|
    .{ .c = struct {
        fn handler() linksection(".ram_text") callconv(.c) void {
            uart.get_state().on_interrupt();
        }
    }.handler }
else
    null;
pub const UART1_IRQ: ?microzig.interrupt.Handler = if (UART.from_instance(.{ .uart = .num(1) })) |uart|
    .{ .c = struct {
        fn handler() linksection(".ram_text") callconv(.c) void {
            uart.get_state().on_interrupt();
        }
    }.handler }
else
    null;

pub const clock: Clock = .{};
pub const Clock = struct {
    pub fn sleep_ms(_: Clock, ms: u32) void {
        rp2xxx.time.sleep_ms(ms);
    }

    pub fn sleep_us(_: Clock, us: u64) void {
        rp2xxx.time.sleep_us(us);
    }
};

pub const SPI_Config = struct {
    instance: rp2xxx.spi.SPI,
    baud_rate: u32,
    pin_clk: Pin,
    pin_mosi: Pin,
    pin_miso: Pin,
    pin_cs: Pin,
};

pub const SPI = enum {
    imu,

    fn apply_all() void {
        inline for (std.enums.values(SPI)) |spi| {
            const cfg = comptime spi.get_config();

            cfg.pin_cs.set_function(.sio);
            cfg.pin_cs.set_direction(.out);
            cfg.pin_cs.put(1);

            inline for (&.{
                cfg.pin_clk,
                cfg.pin_mosi,
                cfg.pin_miso,
            }) |pin| {
                pin.set_function(.spi);
            }

            cfg.instance.apply(.{
                .clock_config = rp2xxx.clock_config,
                .baud_rate = cfg.baud_rate,
            });
        }
    }

    pub fn transceive(spi: SPI, buf: []u8) !void {
        switch (spi) {
            inline else => |spi_comptime| {
                const cfg = spi_comptime.get_config();

                cfg.pin_cs.put(0);
                defer cfg.pin_cs.put(1);

                cfg.instance.transceive_blocking(u8, buf, buf);
            },
        }
    }

    fn get_config(comptime spi: SPI) SPI_Config {
        return switch (spi) {
            .imu => hw.def.imu.spi,
        };
    }
};

pub const I2C_Config = struct {
    instance: rp2xxx.i2c.I2C,
    baud_rate: u32,
    pin_sda: Pin,
    pin_scl: Pin,
};
pub const PWM_Config = struct {
    pwm_slice: rp2xxx.pwm.Slice,
    pin_a: ?Pin = null,
    pin_b: ?Pin = null,
};
