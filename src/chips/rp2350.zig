const std = @import("std");
const root = @import("root");
const assert = std.debug.assert;

const microzig = @import("microzig");
const cpu = microzig.cpu;
const rp2xxx = microzig.hal;
pub const Pin = rp2xxx.gpio.Pin;

const hw = @import("../hw.zig");
const Scheduler = @import("../Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const Task = Scheduler.Task;
const drivers = @import("../drivers.zig");
const time = @import("../time.zig");
const Duration = time.Duration;
const Absolute = time.Absolute;
const RingBuffer = @import("../utils/ring_buffer.zig").RingBuffer;

const log = std.log.scoped(.chip_rp2350);

pub const interrupts: microzig.InterruptOptions = .{
    .TIMER0_IRQ_0 = .{ .c = TIMER0_IRQ_0 },
    .IO_IRQ_BANK0 = .{ .c = IO_IRQ_BANK0 },
    .UART0_IRQ = .{ .c = UART0_IRQ },
    .UART1_IRQ = .{ .c = UART1_IRQ },
    .SPAREIRQ_IRQ_0 = .{ .c = SPAREIRQ_IRQ_0 },
    .SPAREIRQ_IRQ_1 = .{ .c = SPAREIRQ_IRQ_1 },
    .SPAREIRQ_IRQ_2 = .{ .c = SPAREIRQ_IRQ_2 },
    .SPAREIRQ_IRQ_3 = .{ .c = SPAREIRQ_IRQ_3 },
};

pub var spi_imu: SPI = undefined;

pub fn init() void {
    RTT.init();

    log.info("initializing hardware", .{});

    schedulers_init();
    periodics_init();

    UART.apply_all();
    SPI.apply_all();
    I2C.apply_all();

    motors.apply();
    servos.apply();

    if (hw.def.led_strip) |_| {
        led_strip.apply();
    }

    log.info("hardware initialization done", .{});
}

const RTT = cpu.rtt.RTT(.{
    // TODO: the default locks do something weird with priorities, look into it
    // .exclusive_access = null,
});
var rtt_log_writer = RTT.writer(0, &.{});

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

    const current_time = get_time_since_boot();
    const seconds = current_time.to_us() / std.time.us_per_s;
    const microseconds = current_time.to_us() % std.time.us_per_s;

    const cs = enter_critical_section();
    defer cs.leave();
    rtt_log_writer.interface.print(prefix ++ format ++ "\r\n", .{ seconds, microseconds } ++ args) catch {};
}

pub fn enter_critical_section() CriticalSection {
    const enable_on_leave = cpu.interrupt.globally_enabled();
    cpu.interrupt.disable_interrupts();
    return .{
        .enable_on_leave = enable_on_leave,
    };
}
pub const CriticalSection = struct {
    enable_on_leave: bool,
    pub fn leave(cs: CriticalSection) void {
        if (cs.enable_on_leave) {
            cpu.interrupt.enable_interrupts();
        }
    }
};

pub fn get_time_since_boot() Absolute {
    return .from_us(rp2xxx.time.get_time_since_boot().to_us());
}

const schedulers_info: std.EnumArray(hw.SchedulerPriority, struct {
    cpu_prio: cpu.interrupt.Priority,
    interrupt: microzig.cpu.Interrupt,
}) = .init(.{
    .realtime = .{
        .cpu_prio = @fromBackingInt(0),
        .interrupt = .SPAREIRQ_IRQ_0,
    },
    .high = .{
        .cpu_prio = @fromBackingInt(1),
        .interrupt = .SPAREIRQ_IRQ_1,
    },
    .mid = .{
        .cpu_prio = @fromBackingInt(2),
        .interrupt = .SPAREIRQ_IRQ_2,
    },
    .low = .{
        .cpu_prio = @fromBackingInt(3),
        .interrupt = .SPAREIRQ_IRQ_3,
    },
});

fn schedulers_init() void {
    inline for (std.enums.values(hw.SchedulerPriority)) |prio| {
        const info = comptime schedulers_info.get(prio);
        cpu.interrupt.set_priority(info.interrupt, info.cpu_prio);
        cpu.interrupt.clear_pending(info.interrupt);
        cpu.interrupt.enable(info.interrupt);
    }
}

pub fn scheduler_get_pend_fn(comptime prio: hw.SchedulerPriority) *const fn () void {
    return struct {
        fn pend_fn() void {
            cpu.interrupt.set_pending(comptime schedulers_info.get(prio).interrupt);
        }
    }.pend_fn;
}

fn SPAREIRQ_IRQ_0() linksection(".ram_text") callconv(.c) void {
    cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_0);
    root.scheduler_realtime.run();
}

fn SPAREIRQ_IRQ_1() linksection(".ram_text") callconv(.c) void {
    cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_1);
    root.scheduler_high.run();
}

fn SPAREIRQ_IRQ_2() linksection(".ram_text") callconv(.c) void {
    cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_2);
    root.scheduler_mid.run();
}

fn SPAREIRQ_IRQ_3() linksection(".ram_text") callconv(.c) void {
    cpu.interrupt.clear_pending(.SPAREIRQ_IRQ_3);
    root.scheduler_low.run();
}

const timer = rp2xxx.system_timer.num(0);
const timer_period_us: u32 = 1_000;
var timer_next_tick: u32 = 0;

fn periodics_init() void {
    timer.set_interrupt_enabled(.alarm0, true);
    timer_next_tick = timer.read_low() +% timer_period_us;
    timer.schedule_alarm(.alarm0, timer_next_tick);
    cpu.interrupt.enable(.TIMER0_IRQ_0);

    cpu.interrupt.clear_pending(.IO_IRQ_BANK0);
    cpu.interrupt.enable(.IO_IRQ_BANK0);

    inline for (std.enums.values(hw.InterruptPin)) |interrupt_pin| {
        const cfg = comptime interrupt_pin.get_config() orelse continue;
        switch (cfg) {
            .hw => |pin| {
                pin.set_function(.sio);
                pin.set_direction(.in);
                pin.set_pull(.up);
                pin.set_irq_enabled(.{ .rise = 1 }, true);
            },
            .sw => {},
        }
    }
}

const TickerState = struct {
    next_tick: ?u32,
    period_us: u32,

    var map: std.EnumArray(hw.TickRate, TickerState) = blk: {
        var ret: std.EnumArray(hw.TickRate, TickerState) = .initUndefined();
        for (std.enums.values(hw.TickRate)) |periodic| {
            const state = ret.getPtr(periodic);
            state.* = .{
                .next_tick = null,
                .period_us = @intCast(periodic.get_period().to_us()),
            };
        }
        break :blk ret;
    };

    pub fn tick(state: *TickerState, now: u32) bool {
        const next_tick = state.next_tick orelse now;
        const late = now -% next_tick;
        if (@as(i32, @bitCast(late)) >= 0) {
            const periods_missed = late / state.period_us + 1;
            state.next_tick = next_tick +% periods_missed * state.period_us;
            return true;
        } else {
            return false;
        }
    }
};

fn TIMER0_IRQ_0() linksection(".ram_text") callconv(.c) void {
    for (0..5) |_| {
        timer.clear_interrupt(.alarm0);
        timer_next_tick +%= timer_period_us;
        timer.schedule_alarm(.alarm0, timer_next_tick);

        if (@as(i32, @bitCast(timer.read_low() -% timer_next_tick)) < 0) {
            break;
        }
    } else {
        log.warn("missed too many ticks!", .{});
        timer.clear_interrupt(.alarm0);
        timer_next_tick = timer.read_low() +% timer_period_us;
        timer.schedule_alarm(.alarm0, timer_next_tick);
    }

    const now = timer.read_low();
    for (std.enums.values(hw.TickRate)) |rate| {
        const state = TickerState.map.getPtr(rate);
        if (state.tick(now)) {
            hw.InterruptPin.sw_wake(rate);
            hw.ticker(rate).publish({});
        }
    }
}

fn IO_IRQ_BANK0() linksection(".ram_text") callconv(.c) void {
    var it: rp2xxx.gpio.IrqEventIter = .{};
    while (it.next()) |trigger| {
        if (trigger.events.rise == 1) {
            hw.InterruptPin.hw_wake(trigger.pin);
        }
    }
}

pub const UART = struct {
    instance: HAL_Instance,
    state: *State,

    pub const HAL_Instance = union(enum) {
        uart: rp2xxx.uart.UART,
        // pio: struct {
        //     pio: rp2xxx.pio.Pio,
        //     sm: rp2xxx.pio.Pio,
        // },

        fn to_uart_instance(comptime hal_instance: HAL_Instance) ?hw.UART_Instance {
            return inline for (std.enums.values(hw.UART_Instance)) |instance| {
                const cfg = comptime instance.get_config() orelse continue;
                if (std.meta.eql(hal_instance, cfg.instance)) {
                    break instance;
                }
            } else null;
        }
    };

    pub const Config = struct {
        instance: HAL_Instance,
        baud_rate: u32,
        pin_tx: Pin,
        pin_rx: Pin,
        buf_size_tx: usize,
        buf_size_rx: usize,
    };

    const State = struct {
        tx_ring_buf: RingBuffer(u8),
        rx_ring_buf: RingBuffer(u8),
        rx_waker: Task.Waker = .uninited,
    };

    fn apply_all() void {
        inline for (std.enums.values(hw.UART_Instance)) |instance| {
            const cfg = comptime instance.get_config() orelse continue;

            cfg.pin_tx.set_function(.uart);
            cfg.pin_rx.set_function(.uart);

            switch (cfg.instance) {
                .uart => |uart| {
                    uart.apply(.{
                        .clock_config = rp2xxx.clock_config,
                        .baud_rate = cfg.baud_rate,
                    });

                    uart.get_regs().UARTIFLS.write(.{ .RXIFLSEL = 0, .TXIFLSEL = 0 });

                    uart.set_interrupts_enabled(.{
                        .rx = true,
                        .tx = true,
                        .rt = true,
                    });

                    switch (uart) {
                        .num(0) => {
                            cpu.interrupt.clear_pending(.UART0_IRQ);
                            cpu.interrupt.enable(.UART0_IRQ);
                        },
                        .num(1) => {
                            cpu.interrupt.clear_pending(.UART1_IRQ);
                            cpu.interrupt.enable(.UART1_IRQ);
                        },
                        _ => @compileError("invalid uart"),
                    }
                },
            }
        }
    }

    pub fn get(comptime instance: hw.UART_Instance) UART {
        const cfg = comptime instance.get_config().?;

        return .{
            .instance = cfg.instance,
            .state = comptime get_state(instance),
        };
    }

    pub fn write_byte(uart: UART, byte: u8) !void {
        try uart.state.tx_ring_buf.push(byte);
        switch (uart.instance) {
            .uart => |hw_uart_instance| switch (hw_uart_instance) {
                .num(0) => cpu.interrupt.set_pending(.UART0_IRQ),
                .num(1) => cpu.interrupt.set_pending(.UART1_IRQ),
                _ => @compileError("invalid uart"),
            },
        }
    }

    pub fn read_byte(uart: UART) ?u8 {
        return uart.state.rx_ring_buf.pop();
    }

    pub fn subscribe(
        uart: UART,
        Context: type,
        context: Context,
        comptime callback: fn (Context, u8) void,
        scheduler: *Scheduler,
    ) void {
        uart.state.rx_waker.register(Context, context, struct {
            pub fn wrapper(ctx: Context, waker: *Task.Waker) void {
                const state: *State = @fieldParentPtr("rx_waker", waker);
                while (state.rx_ring_buf.pop()) |byte| {
                    callback(ctx, byte);
                }
            }
        }.wrapper, scheduler);
    }

    fn get_state(comptime instance: hw.UART_Instance) *State {
        const cfg = comptime instance.get_config().?;

        const S = struct {
            var buf_tx: [cfg.buf_size_tx]u8 = undefined;
            var buf_rx: [cfg.buf_size_rx]u8 = undefined;
            var state: State = .{
                .tx_ring_buf = .init(&buf_tx),
                .rx_ring_buf = .init(&buf_rx),
            };
        };

        return &S.state;
    }
};

pub fn hardware_uart_interrupt_common(uart: rp2xxx.uart.UART, state: *UART.State) linksection(".ram_text") void {
    // TX fires on transition through the level, not the level
    // itself. Clear the latched TX edge so an idle empty FIFO
    // doesn't re-fire. RX flags are cleared by draining the
    // FIFO below.
    uart.get_regs().UARTICR.write(.{ .TXIC = 1 });

    while (true) {
        const maybe_byte = uart.read_word() catch |err| {
            log.warn("uart: failed to read byte: {t}", .{err});
            uart.clear_errors();
            continue;
        };

        const byte = maybe_byte orelse break;

        state.rx_ring_buf.push(byte) catch {};
    }
    if (!state.rx_ring_buf.is_empty()) state.rx_waker.wake();

    while (uart.is_writeable()) {
        if (state.tx_ring_buf.pop()) |byte| {
            uart.get_regs().UARTDR.write_raw(byte);
        } else {
            break;
        }
    }
}

pub fn UART0_IRQ() linksection(".ram_text") callconv(.c) void {
    const uart: rp2xxx.uart.UART = comptime .num(0);
    if (comptime (UART.HAL_Instance{ .uart = uart }).to_uart_instance()) |instance| {
        const state = UART.get_state(instance);
        hardware_uart_interrupt_common(.num(0), state);
    } else {
        @panic("UART0_IRQ: no instance found");
    }
}

pub fn UART1_IRQ() linksection(".ram_text") callconv(.c) void {
    const uart: rp2xxx.uart.UART = comptime .num(1);
    if (comptime (UART.HAL_Instance{ .uart = uart }).to_uart_instance()) |instance| {
        const state = UART.get_state(instance);
        hardware_uart_interrupt_common(.num(1), state);
    } else {
        @panic("UART1_IRQ: no instance found");
    }
}

pub const Clock = struct {
    pub var instance: Clock = .{};

    pub fn sleep_ms(_: Clock, ms: u32) void {
        rp2xxx.time.sleep_ms(ms);
    }

    pub fn sleep_us(_: Clock, us: u64) void {
        rp2xxx.time.sleep_us(us);
    }
};

pub const Flash = struct {
    // TODO: we must also wait for flash dma transfers to finish in addition to
    // critical sections

    const BASE = rp2xxx.flash.XIP_BASE;
    const SIZE = hw.def.flash.size;

    pub const Config = struct {
        size: u32,
        storage_start: u32,
        storage_end: u32,
    };

    pub const WRITE_SIZE = 1;
    pub const ERASE_SIZE = rp2xxx.flash.SECTOR_SIZE;

    pub var instance: Flash = .{};

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

pub const SPI = struct {
    pin_cs: Pin,
    instance: rp2xxx.spi.SPI,

    pub const Config = struct {
        instance: rp2xxx.spi.SPI,
        baud_rate: u32,
        pin_clk: Pin,
        pin_mosi: Pin,
        pin_miso: Pin,
        pin_cs: Pin,
    };

    fn apply_all() void {
        inline for (std.enums.values(hw.SPI_Instance)) |instance| {
            const cfg = comptime instance.get_config() orelse continue;

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

    pub fn get(comptime instance: hw.SPI_Instance) SPI {
        const cfg = comptime instance.get_config().?;
        return .{
            .pin_cs = cfg.pin_cs,
            .instance = cfg.instance,
        };
    }

    pub fn reconfigure(spi: SPI, baud_rate: u32) void {
        // TODO: ideally we should only update the baud rate
        spi.instance.apply(.{
            .clock_config = rp2xxx.clock_config,
            .baud_rate = baud_rate,
        });
    }

    pub fn write(spi: SPI, buf: []const u8) !void {
        spi.pin_cs.put(0);
        defer spi.pin_cs.put(1);

        spi.instance.write_blocking(u8, buf);
    }

    pub fn transceive(spi: SPI, write_buf: []const u8, read_buf: []u8) !void {
        spi.pin_cs.put(0);
        defer spi.pin_cs.put(1);

        spi.instance.transceive_blocking(u8, write_buf, read_buf);
    }

    pub fn transceive_in_place(spi: SPI, buf: []u8) !void {
        try spi.transceive(buf, buf);
    }
};

pub const I2C = struct {
    instance: rp2xxx.i2c.I2C,
    mutex: *Scheduler.SafetyMutex,

    const timeout: Duration = .from_ms(100);

    var mutexes: std.EnumArray(hw.I2C_Instance, Scheduler.SafetyMutex) = .initFill(.{});

    pub const Config = struct {
        instance: rp2xxx.i2c.I2C,
        baud_rate: u32,
        pin_sda: Pin,
        pin_scl: Pin,
    };

    fn apply_all() void {
        inline for (std.enums.values(hw.I2C_Instance)) |instance| {
            const cfg = comptime instance.get_config() orelse continue;

            inline for (&.{
                cfg.pin_sda,
                cfg.pin_scl,
            }) |pin| {
                pin.set_function(.i2c);
            }

            cfg.instance.apply(.{
                .clock_config = rp2xxx.clock_config,
                .baud_rate = cfg.baud_rate,
            });
        }
    }

    pub fn get(comptime instance: hw.I2C_Instance) I2C {
        const cfg = comptime instance.get_config().?;
        return .{
            .instance = cfg.instance,
            .mutex = mutexes.getPtr(instance),
        };
    }

    pub fn read(i2c: I2C, addr: u7, buf: []u8) !void {
        i2c.mutex.lock();
        defer i2c.mutex.unlock();

        try i2c.instance.read_blocking(@fromBackingInt(addr), buf, .from_us(timeout.to_us()));
    }

    pub fn write(i2c: I2C, addr: u7, buf: []const u8) !void {
        i2c.mutex.lock();
        defer i2c.mutex.unlock();

        try i2c.instance.write_blocking(@fromBackingInt(addr), buf, .from_us(timeout.to_us()));
    }

    pub fn write_than_read(i2c: I2C, addr: u7, write_buf: []const u8, read_buf: []u8) !void {
        i2c.mutex.lock();
        defer i2c.mutex.unlock();

        try i2c.instance.write_then_read_blocking(@fromBackingInt(addr), write_buf, read_buf, .from_us(timeout.to_us()));
    }
};

pub const MotorConfig = struct {
    pio: rp2xxx.pio.Pio,
    sm: rp2xxx.pio.StateMachine,
    pin: Pin,
};

pub const motors = struct {
    pub const count = hw.def.motors.outputs.len;

    fn apply() void {
        const dshot_speed = switch (hw.def.motors.protocol) {
            .dshot_300 => 300,
        };

        const clkdiv = comptime rp2xxx.pio.ClkDivOptions.from_ratio(
            rp2xxx.clock_config.sys.?.frequency(),
            dshot_speed * 8_000,
        );

        var offsets: std.EnumArray(rp2xxx.pio.Pio, u5) = .initFill(0);
        comptime var loaded: std.EnumSet(rp2xxx.pio.Pio) = .empty;

        inline for (hw.def.motors.outputs) |output| {
            if (comptime !loaded.contains(output.pio)) {
                comptime loaded.setPresent(output.pio, true);

                const offset = output.pio.add_program(dshot_program) catch unreachable;
                offsets.set(output.pio, offset);
            }

            output.pio.gpio_init(output.pin);
            const offset = offsets.get(output.pio);
            output.pio.sm_init(output.sm, offset, .{
                .clkdiv = clkdiv,
                .pin_mappings = .{ .set = .single(output.pin) },
                .exec = .{
                    .wrap_target = offset,
                    .wrap = offset + @as(u5, dshot_program.instructions.len),
                },
                .shift = .{ .out_shiftdir = .left },
            }) catch unreachable;
            output.pio.sm_exec_set_pindir(output.sm, 1);
            output.pio.sm_set_enabled(output.sm, true);
        }
    }

    /// values are 0.0..=1.0
    pub fn write(values: *const [count]f32) void {
        for (hw.def.motors.outputs, values) |output, value| {
            std.debug.assert(0.0 <= value and value <= 1.0);
            output.pio.sm_write(output.sm, encode_throttle(value));
        }
    }

    pub fn disarm() void {
        for (hw.def.motors.outputs) |output| {
            output.pio.sm_write(output.sm, encode_command(.stop, false));
        }
    }

    // ESCs ignore further commands for ~260ms after a beep
    pub fn beep() void {
        for (hw.def.motors.outputs) |output| {
            output.pio.sm_write(output.sm, encode_command(.beep_1, false));
        }
    }

    pub fn set_spin_direction(direction: SpinDirection) void {
        const command: Command = switch (direction) {
            .normal => .spin_direction_normal,
            .reversed => .spin_direction_reversed,
        };
        // command must be received 6 times
        for (0..6) |_| {
            for (hw.def.motors.outputs) |output| {
                // block so that we ensure commands are sent
                output.pio.sm_write_blocking(output.sm, encode_command(command, false));
            }
        }
    }

    const dshot_program = blk: {
        @setEvalBranchQuota(10_000);
        break :blk rp2xxx.pio.assemble(
            \\.program dshot
            \\entry:
            \\    pull
            \\    out null, 16
            \\    set x, 15
            \\loop:
            \\    set pins, 1
            \\    out y, 1
            \\    jmp !y zero
            \\    nop [2]
            \\one:
            \\    set pins, 0
            \\    jmp x-- loop
            \\    jmp reset
            \\zero:
            \\    set pins, 0 [3]
            \\    jmp x-- loop
            \\    jmp reset
            \\reset: ; Blank frame
            \\    nop [31]
            \\    nop [31]
            \\    nop [31]
            \\    jmp entry [31]
        , .{}).get_program_by_name("dshot");
    };

    // TODO: should be common to all dshot implementations, move some place else
    const Command = enum(u16) {
        stop = 0,
        beep_1 = 1,
        beep_2 = 2,
        beep_3 = 3,
        beep_4 = 4,
        beep_5 = 5,
        spin_direction_normal = 20,
        spin_direction_reversed = 21,
        throttle_min = 48,
    };

    pub const THROTTLE_MIN: u16 = @backingInt(Command.throttle_min);
    pub const THROTTLE_MAX: u16 = 2047;

    pub const SpinDirection = enum {
        normal,
        reversed,
    };

    fn encode(command: u16, telemetry: bool) u16 {
        const packet = (command << 1) | @intFromBool(telemetry);
        const crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
        return ((packet & 0xFFF) << 4) | crc;
    }

    fn encode_command(command: Command, telemetry: bool) u16 {
        return encode(@backingInt(command), telemetry);
    }

    // 0.0..=1.0 -> THROTTLE_MIN..=THROTTLE_MAX
    fn encode_throttle(value: f32) u16 {
        const throttle: u16 = @round(value * @as(f32, @floatFromInt(THROTTLE_MAX - THROTTLE_MIN)));
        return encode(THROTTLE_MIN + throttle, false);
    }
};

pub const ServoConfig = Pin;

pub const servos = struct {
    pub const count = hw.def.servos.len;

    fn apply() void {
        // Make pwm counter increment once every microsecond
        const div: rp2xxx.pwm.FractionalDivider = comptime .from_ratio(rp2xxx.clock_config.sys.?.frequency(), 1_000_000);
        // The frequency of the pwm signal is 50Hz.
        const wrap = 20_000 - 1;

        comptime var is_slice_configured: std.EnumArray(rp2xxx.pwm.Slice, bool) = .initFill(false);

        inline for (hw.def.servos) |pin| {
            const pwm = comptime rp2xxx.pwm.get_pwm(@backingInt(pin));

            pwm.set_level(1_500);

            const slice = comptime pwm.slice();
            if (comptime !is_slice_configured.get(slice)) {
                slice.set_clk_div(div);
                slice.set_phase_correct(false);
                slice.set_wrap(wrap);
                slice.enable();
                comptime is_slice_configured.set(slice, true);
            }

            pin.set_function(.pwm);
        }
    }

    /// values are 1000..=2000 us
    pub fn write(values: *const [count]u16) void {
        inline for (hw.def.servos, values) |pin, value| {
            std.debug.assert(1000 <= value and value <= 2000);
            const pwm = comptime rp2xxx.pwm.get_pwm(@backingInt(pin));
            pwm.set_level(value);
        }
    }
};

pub const LedStripConfig = struct {
    pio: rp2xxx.pio.Pio,
    sm: rp2xxx.pio.StateMachine,
    pin: Pin,
};

pub const led_strip = if (hw.def.led_strip) |def_led_strip| struct {
    const cfg = def_led_strip.config;

    fn apply() void {
        // TODO: the hal is a bit akward when using pins over 32
        if (@backingInt(cfg.pin) >= 32) {
            cfg.pio.get_regs().GPIOBASE.write_raw(16);
        }

        cfg.pio.gpio_init(cfg.pin);
        cfg.pio.sm_set_pindir(cfg.sm, cfg.pin, 1, .out) catch unreachable;

        const cycles_per_bit: comptime_int = ws2812_program.defines[0].value + //T1
            ws2812_program.defines[1].value + //T2
            ws2812_program.defines[2].value; //T3
        const div = @as(f32, @floatFromInt(rp2xxx.clock_config.sys.?.frequency())) /
            (800_000 * cycles_per_bit);

        cfg.pio.sm_load_and_start_program(cfg.sm, ws2812_program, .{
            .clkdiv = .from_float(div),
            .pin_mappings = .{
                .side_set = .single(cfg.pin),
            },
            .shift = .{
                .out_shiftdir = .left,
                .autopull = true,
                .pull_threshold = 24,
                .join_tx = true,
            },
        }) catch unreachable;
        cfg.pio.sm_set_enabled(cfg.sm, true);
    }

    pub fn write(color: drivers.Color) void {
        // zig fmt: off
        const code = @as(u32, color.b) <<  8 |
                     @as(u32, color.r) << 16 |
                     @as(u32, color.g) << 24;
        // zig fmt: on
        cfg.pio.sm_write(cfg.sm, code);
    }

    const ws2812_program = blk: {
        @setEvalBranchQuota(10_000);
        break :blk rp2xxx.pio.assemble(
            \\;
            \\; Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
            \\;
            \\; SPDX-License-Identifier: BSD-3-Clause
            \\;
            \\.program ws2812
            \\.side_set 1
            \\
            \\.define public T1 2
            \\.define public T2 5
            \\.define public T3 3
            \\
            \\.wrap_target
            \\bitloop:
            \\    out x, 1       side 0 [T3 - 1] ; Side-set still takes place when instruction stalls
            \\    jmp !x do_zero side 1 [T1 - 1] ; Branch on the bit we shifted out. Positive pulse
            \\do_one:
            \\    jmp  bitloop   side 1 [T2 - 1] ; Continue driving high, for a long pulse
            \\do_zero:
            \\    nop            side 0 [T2 - 1] ; Or drive low, for a short pulse
            \\.wrap
        , .{}).get_program_by_name("ws2812");
    };
} else @compileError("led strip not available in this configuration");
