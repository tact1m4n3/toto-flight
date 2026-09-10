const std = @import("std");
const assert = std.debug.assert;

const microzig = @import("microzig");
const rp2xxx = microzig.hal;
pub const Pin = rp2xxx.gpio.Pin;

const control = @import("../control.zig");
const hw = @import("../hw.zig");
const imu = @import("../imu.zig");
const receiver = @import("../receiver.zig");
const storage = @import("../storage.zig");
const Scheduler = @import("../Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const Task = Scheduler.Task;
const Waker = Scheduler.Waker;
const time = @import("../time.zig");
const Duration = time.Duration;
const Absolute = time.Absolute;
const RingBuffer = @import("../utils/ring_buffer.zig").RingBuffer;

const log = std.log.scoped(.chip_rp2350);

// TODO: maybe ticker and periodics can be merged into one enum, tasks
// subscribe to the corresponding ticker

pub const interrupts: microzig.InterruptOptions = .{
    .TIMER0_IRQ_0 = .{ .c = TIMER0_IRQ_0 },
    .IO_IRQ_BANK0 = .{ .c = IO_IRQ_BANK0 },
    .UART0_IRQ = UART0_IRQ,
    .UART1_IRQ = UART1_IRQ,
    .SPAREIRQ_IRQ_0 = .{ .c = SPAREIRQ_IRQ_0 },
    .SPAREIRQ_IRQ_1 = .{ .c = SPAREIRQ_IRQ_1 },
    .SPAREIRQ_IRQ_2 = .{ .c = SPAREIRQ_IRQ_2 },
    .SPAREIRQ_IRQ_3 = .{ .c = SPAREIRQ_IRQ_3 },
};

var task_storage: storage.StorageGeneric(.{
    .imu = &imu.param_table,
    .rate = &control.param_table_rate,
}) = undefined;

var task_imu: imu.Imu = undefined;
var task_rx: receiver.Rx = undefined;
var task_channel_mapper: receiver.ChannelMapper = undefined;

var task_control: control.Loop = undefined;

pub fn main() noreturn {
    microzig.cpu.interrupt.disable_interrupts();

    RTT.init();
    rtt_logger = RTT.writer(0, &rtt_writer_buf);

    log.info("booting", .{});

    schedulers_init();
    timer_init();

    // enable flash
    if (microzig.config.ram_image) {
        rp2xxx.rom.connect_internal_flash();
        rp2xxx.rom.flash_exit_xip();
        rp2xxx.flash.boot2.flash_enable_xip();
    }

    InterruptPin.apply_all();
    UART.apply_all();
    SPI.apply_all();

    log.info("initializing tasks", .{});

    task_storage.init(&scheduler_low_priority);

    task_imu.init(&scheduler_realtime_priority);
    task_rx.init(&scheduler_high_priority);
    task_channel_mapper.init(&scheduler_high_priority);

    task_control.init(&scheduler_realtime_priority);

    microzig.cpu.interrupt.enable_interrupts();

    log.info("initialization done", .{});

    while (true) {
        microzig.cpu.wfi();
    }
}

const RTT = microzig.cpu.rtt.RTT(.{});
var rtt_logger: ?RTT.Writer = null;
var rtt_writer_buf: [256]u8 = undefined;

pub fn log_fn(
    comptime level: std.log.Level,
    comptime scope: @EnumLiteral(),
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

const scheduler_priority_realtime: microzig.cpu.interrupt.Priority = @fromBackingInt(@intCast(0));
const scheduler_priority_high: microzig.cpu.interrupt.Priority = @fromBackingInt(@intCast(1));
const scheduler_priority_mid: microzig.cpu.interrupt.Priority = @fromBackingInt(@intCast(2));
const scheduler_priority_low: microzig.cpu.interrupt.Priority = @fromBackingInt(@intCast(3));

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

const timer = rp2xxx.system_timer.num(0);
const timer_period_us: u32 = 1_000;
var timer_next_tick: u32 = 0;

pub fn timer_init() void {
    timer.set_interrupt_enabled(.alarm0, true);
    timer_next_tick = timer.read_low() +% timer_period_us;
    timer.schedule_alarm(.alarm0, timer_next_tick);
    microzig.cpu.interrupt.enable(.TIMER0_IRQ_0);
}

// NOTE: this timer is already setup by the rp2xxx hal to tick every 1us
// TODO: this could be implemented better I think
pub fn TIMER0_IRQ_0() linksection(".ram_text") callconv(.c) void {
    while (true) {
        timer.clear_interrupt(.alarm0);
        timer_next_tick +%= timer_period_us;
        timer.schedule_alarm(.alarm0, timer_next_tick);

        const diff: i32 = @bitCast(timer.read_low() -% timer_next_tick);
        if (diff < 0) {
            break;
        } else {
            log.warn("missed tick!", .{});
        }
    }

    Ticker.tick_all();
}

pub const Ticker = enum(u32) {
    @"10Hz" = 10,
    @"50Hz" = 50,
    @"100Hz" = 100,
    @"250Hz" = 250,
    @"500Hz" = 500,
    @"1000Hz" = 1000,

    const State = struct {
        next_tick: time.Absolute = .from_us(0),
        message: Message(Absolute) = .{},
    };

    var states: std.EnumArray(Ticker, State) = .initFill(.{});

    /// Inline this into the interrupt handler.
    inline fn tick_all() void {
        const now = get_time_since_boot();
        inline for (std.enums.values(Ticker)) |ticker| {
            var state = states.getPtr(ticker);
            while (state.next_tick.is_reached_by(now)) {
                const ts = state.next_tick;
                state.message.publish(ts);
                state.next_tick = ts.add_duration(comptime ticker.get_period());
            }
        }
    }

    pub fn get_period(ticker: Ticker) Duration {
        return .from_hz(@backingInt(ticker));
    }

    pub fn subscribe(
        ticker: Ticker,
        rcv: *Receiver(Absolute),
        Context: type,
        context: Context,
        comptime callback: fn (context: Context, absolute: Absolute) void,
        scheduler: *Scheduler,
    ) void {
        states.getPtr(ticker).message.subscribe(rcv, Context, context, callback, scheduler);
    }
};

pub const InterruptPin = enum(u6) {
    imu = @backingInt(hw.def.imu.pin_interrupt),

    var message_map: std.EnumArray(InterruptPin, Message(Absolute)) = .initFill(.{});

    fn apply_all() void {
        inline for (std.enums.values(InterruptPin)) |interrupt_pin| {
            const pin: rp2xxx.gpio.Pin = @fromBackingInt(@intCast(@backingInt(interrupt_pin)));
            pin.set_function(.sio);
            pin.set_direction(.in);
            pin.set_pull(.up);
            pin.set_irq_enabled(.{ .rise = 1 }, true);
        }
        microzig.cpu.interrupt.clear_pending(.IO_IRQ_BANK0);
        microzig.cpu.interrupt.enable(.IO_IRQ_BANK0);
    }

    pub fn subscribe(
        pin: InterruptPin,
        rcv: *Receiver(Absolute),
        Context: type,
        context: Context,
        comptime callback: fn (Context, Absolute) void,
        scheduler: *Scheduler,
    ) void {
        message_map.getPtr(pin).subscribe(rcv, Context, context, callback, scheduler);
    }
};

fn IO_IRQ_BANK0() linksection(".ram_text") callconv(.c) void {
    const ts = get_time_since_boot();
    var it: rp2xxx.gpio.IrqEventIter = .{};
    while (it.next()) |trigger| {
        if (trigger.events.rise == 1) {
            if (std.enums.fromInt(InterruptPin, @backingInt(trigger.pin))) |pin| {
                InterruptPin.message_map.getPtr(pin).publish(ts);
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

    pub fn subscribe(
        uart_rx: UART_RX,
        task: *Task,
        Context: type,
        context: Context,
        comptime callback: fn (Context, u8) void,
        scheduler: *Scheduler,
    ) void {
        switch (uart_rx.inner) {
            inline else => |uart_comptime| uart_comptime.get_state().subscribe(
                task,
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

        const RxError = rp2xxx.uart.ReceiveError;
        tx_ring_buf: RingBuffer(u8, cfg.buf_size_tx) = .{},
        rx_ring_buf: RingBuffer(u8, cfg.buf_size_rx) = .{},
        rx_waker: Waker = .{},

        pub fn apply(_: *Self) void {
            cfg.pin_tx.set_function(.uart);
            cfg.pin_rx.set_function(.uart);

            switch (cfg.instance) {
                .uart => |uart| {
                    uart.apply(.{
                        .clock_config = rp2xxx.clock_config,
                        .baud_rate = cfg.baud_rate,
                    });

                    uart.get_regs().UARTIFLS.write(.{ .RXIFLSEL = 0, .TXIFLSEL = 0 });

                    uart.get_regs().UARTIMSC.write(.{
                        .TXIM = 1,
                        .RXIM = 1,
                        .RTIM = 1,
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

        pub fn subscribe(
            state: *Self,
            task: *Task,
            Context: type,
            context: Context,
            comptime callback: fn (Context, u8) void,
            scheduler: *Scheduler,
        ) void {
            task.* = .init(Context, context, struct {
                pub fn wrapper(ctx: Context, _: *Task) void {
                    const uart_enum = comptime UART.from_instance(cfg.instance).?;
                    const s = uart_enum.get_state();
                    while (s.rx_ring_buf.pop()) |byte| {
                        callback(ctx, byte);
                    }
                    // Ring has room again: reopen RX.
                    switch (cfg.instance) {
                        .uart => |uart| uart.get_regs().UARTIMSC.write(.{
                            .TXIM = 1,
                            .RXIM = 1,
                            .RTIM = 1,
                        }),
                    }
                }
            }.wrapper, scheduler);
            state.rx_waker.register(task);
        }

        pub fn write_byte(state: *Self, byte: u8) !void {
            try state.tx_ring_buf.push(byte);
            switch (cfg.instance) {
                .uart => |uart| {
                    switch (uart) {
                        .num(0) => microzig.cpu.interrupt.set_pending(.UART0_IRQ),
                        .num(1) => microzig.cpu.interrupt.set_pending(.UART1_IRQ),
                        _ => @compileError("invalid uart"),
                    }
                },
            }
        }

        // Inline because we want it in .ram_text
        pub inline fn on_interrupt(state: *Self) void {
            switch (cfg.instance) {
                .uart => |uart| {
                    // TX fires on transition through the level, not the level
                    // itself. Clear the latched TX edge so an idle empty FIFO
                    // doesn't re-fire. RX flags are cleared by draining the
                    // FIFO below.
                    uart.get_regs().UARTICR.write(.{ .TXIC = 1 });

                    var rx_full = false;
                    while (true) {
                        const maybe_byte = uart.read_word() catch |err| {
                            log.warn("uart: failed to read byte: {t}", .{err});
                            uart.clear_errors();
                            continue;
                        };

                        const byte = maybe_byte orelse break;

                        state.rx_ring_buf.push(byte) catch {
                            rx_full = true;
                            break;
                        };
                    }
                    if (!state.rx_ring_buf.is_empty()) state.rx_waker.wake();
                    if (rx_full) {
                        // Ring full: mask RX/RT so the still-full HW FIFO
                        // can't re-fire forever. Task wrapper reopens after
                        // draining.
                        uart.get_regs().UARTIMSC.write(.{
                            .TXIM = 1,
                        });
                    }

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

pub var clock: Clock = .{};
pub const Clock = struct {
    pub fn sleep_ms(_: *Clock, ms: u32) void {
        rp2xxx.time.sleep_ms(ms);
    }

    pub fn sleep_us(_: *Clock, us: u64) void {
        rp2xxx.time.sleep_us(us);
    }
};

pub const FlashConfig = struct {
    size: u32,
    storage_start: u32,
    storage_end: u32,
};

pub var flash: Flash = .{};

pub const Flash = struct {
    // TODO: if we ever do flash dma transfers we should wait for those to
    // finish in addition to critical sections

    const BASE = rp2xxx.flash.XIP_BASE;
    const SIZE = hw.def.flash.size;

    pub const WRITE_SIZE = 1;
    pub const ERASE_SIZE = rp2xxx.flash.SECTOR_SIZE;

    pub fn erase(_: Flash, offset: u32, size: u32) error{EraseFailed}!void {
        assert(std.mem.isAlignedGeneric(u32, offset, ERASE_SIZE));
        assert(std.mem.isAlignedGeneric(u32, size, ERASE_SIZE));
        assert(offset + size <= SIZE);

        const cs = enter_critical_section();
        defer cs.leave();
        rp2xxx.flash.range_erase(BASE + offset, size);
    }

    pub fn read_buf(_: Flash, start: u32, end: u32) error{ ReadFailed, Corrupted }!?[]const u8 {
        assert(start <= SIZE);
        assert(end <= SIZE);
        if (start != end) {
            return @as([*]u8, @ptrFromInt(BASE))[start..end];
        } else {
            return null;
        }
    }

    pub fn read(_: Flash, offset: u32, data: []u8) error{ ReadFailed, Corrupted }!void {
        assert(offset + @as(u32, @truncate(data.len)) <= SIZE);
        std.mem.copyForwards(u8, data, @as([*]u8, @ptrFromInt(BASE + offset))[0..data.len]);
    }

    pub fn write(_: Flash, offset: u32, data: []const u8) error{ WriteFailed, PageAlreadyProgrammed }!void {
        assert(offset + @as(u32, @truncate(data.len)) <= SIZE);

        const PAGE_SIZE = rp2xxx.flash.PAGE_SIZE;

        const cs = enter_critical_section();
        defer cs.leave();

        var current_offset = offset;
        var remaining_data = data;

        while (remaining_data.len > 0) {
            var buffer: [PAGE_SIZE]u8 = @splat(0xFF);

            const page_offset = current_offset & ~@as(u32, PAGE_SIZE - 1);
            const offset_in_page = current_offset & @as(u32, PAGE_SIZE - 1);
            const count = @min(remaining_data.len, PAGE_SIZE - offset_in_page);
            std.mem.copyForwards(u8, buffer[offset_in_page..][0..count], remaining_data[0..count]);

            rp2xxx.flash.range_program(page_offset, &buffer);

            remaining_data = remaining_data[count..];
            current_offset += count;
        }
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
