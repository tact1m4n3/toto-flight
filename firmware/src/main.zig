const std = @import("std");
const Allocator = std.mem.Allocator;
const microzig = @import("microzig");
const Absolute = microzig.drivers.time.Absolute;
const peripherals = microzig.chip.peripherals;
const rp2xxx = microzig.hal;

const CRSF_Parser = @import("CRSF_Parser.zig");

pub const microzig_options: microzig.Options = .{
    .log_level = .info,
    .logFn = microzig.hal.uart.log,
    .interrupts = .{
        .UART0_IRQ = .{ .c = UART0_IRQ },
    },
};

pub const config = struct {
    const rx_uart = rp2xxx.uart.instance.num(0);
    const rx_uart_baud_rate = 115_200;
    const rx_uart_tx_pin = rp2xxx.gpio.num(0);
    const rx_uart_rx_pin = rp2xxx.gpio.num(1);

    const debug_uart = rp2xxx.uart.instance.num(1);
    const debug_uart_baud_rate = 115_200;
    const debug_uart_tx_pin = rp2xxx.gpio.num(8);

    const PWM_DIV = 50;
    const PWM_WRAP = 50_000;
    const PWM_PERIOD_MS = 20;

    const ESC_PWM_MIN_LEVEL: u16 = @trunc(@as(f32, 1.0) / PWM_PERIOD_MS * PWM_WRAP);
    const ESC_PWM_MAX_LEVEL: u16 = @trunc(@as(f32, 2.0) / PWM_PERIOD_MS * PWM_WRAP);

    const ELEVON_PWM_MIN_LEVEL: u16 = @trunc(@as(f32, 0.5) / PWM_PERIOD_MS * PWM_WRAP);
    const ELEVON_PWM_MID_LEVEL: u16 = @trunc(@as(f32, 1.5) / PWM_PERIOD_MS * PWM_WRAP);
    const ELEVON_PWM_MAX_LEVEL: u16 = @trunc(@as(f32, 2.5) / PWM_PERIOD_MS * PWM_WRAP);

    const esc_pwm: rp2xxx.pwm.Pwm = .{
        .slice_number = 1,
        .channel = .a,
    };
    const esc_pwm_pin = rp2xxx.gpio.num(2);

    const right_elevon_pwm: rp2xxx.pwm.Pwm = .{
        .slice_number = 5,
        .channel = .a,
    };
    const right_elevon_pwm_pin = rp2xxx.gpio.num(10);

    const left_elevon_pwm: rp2xxx.pwm.Pwm = .{
        .slice_number = 5,
        .channel = .b,
    };
    const left_elevon_pwm_pin = rp2xxx.gpio.num(11);
};

pub const state = struct {
    var channels: [16]u16 = @splat(1000);
    var last_received_channels_time: Absolute = .from_us(0);
};

pub fn main() !void {
    state.last_received_channels_time = rp2xxx.time.get_time_since_boot();

    // debugging
    config.debug_uart_tx_pin.set_function(.uart);
    config.debug_uart.apply(.{
        .baud_rate = config.debug_uart_baud_rate,
        .clock_config = rp2xxx.clock_config,
    });
    rp2xxx.uart.init_logger(config.debug_uart);

    // receiver
    inline for (&.{ config.rx_uart_tx_pin, config.rx_uart_rx_pin }) |pin| {
        pin.set_function(.uart);
    }
    config.rx_uart.apply(.{
        .baud_rate = config.rx_uart_baud_rate,
        .clock_config = rp2xxx.clock_config,
    });
    config.rx_uart.set_interrupts_enabled(.{ .rx = true });

    // configure pwm
    inline for (&.{ config.esc_pwm_pin, config.left_elevon_pwm_pin, config.right_elevon_pwm_pin }) |pin| {
        pin.set_function(.pwm);
    }

    const esc_slice = config.esc_pwm.slice();
    esc_slice.set_clk_div(config.PWM_DIV, 0);
    esc_slice.set_phase_correct(false);
    esc_slice.set_wrap(config.PWM_WRAP - 1);
    esc_slice.enable();

    const elevon_slice = config.left_elevon_pwm.slice();
    elevon_slice.set_clk_div(config.PWM_DIV, 0);
    elevon_slice.set_phase_correct(false);
    elevon_slice.set_wrap(config.PWM_WRAP - 1);
    elevon_slice.enable();

    microzig.cpu.interrupt.set_priority(.UART0_IRQ, .highest);
    microzig.cpu.interrupt.clear_pending(.UART0_IRQ);
    microzig.cpu.interrupt.enable(.UART0_IRQ);

    microzig.cpu.interrupt.enable_interrupts();

    var armed: bool = false;

    while (true) {
        // don't get interrupted during this critical section
        {
            const cs = microzig.interrupt.enter_critical_section();
            defer cs.leave();

            const channels = state.channels;

            // are we in failsafe?
            const now = rp2xxx.time.get_time_since_boot();
            const failsafe = !now.diff(state.last_received_channels_time).less_than(.from_ms(1_000));

            const throttle = channels[2];
            // const yaw = channels[3];
            const roll = channels[0];
            const pitch = channels[1];

            // update armed flag
            const arm_switch_value = channels[4];
            if (!armed and arm_switch_value >= 1500 and throttle < 1100) {
                std.log.info("armed", .{});
                armed = true;
            } else if (armed and arm_switch_value < 1500) {
                std.log.info("disarmed", .{});
                armed = false;
            }

            // send default values during failsafe and when not armed
            if (failsafe or !armed) {
                config.esc_pwm.set_level(config.ESC_PWM_MIN_LEVEL);
                config.left_elevon_pwm.set_level(config.ELEVON_PWM_MID_LEVEL);
                config.right_elevon_pwm.set_level(config.ELEVON_PWM_MID_LEVEL);
                continue;
            }

            // output control
            const roll_fp = roll_pitch_mapping.map(@floatFromInt(roll)); // map between -1 and 1
            const pitch_fp = roll_pitch_mapping.map(@floatFromInt(pitch)); // map between -1 and 1

            const left_elevon = (roll_fp - pitch_fp) / @as(f32, @floatFromInt(2)); // (roll_fp - pitch_fp) / 2
            const right_elevon = (roll_fp - pitch_fp) / @as(f32, @floatFromInt(2)); // (roll_fp + pitch_fp) / 2
            config.esc_pwm.set_level(@intFromFloat(throttle_mapping.map(@floatFromInt(throttle))));
            config.left_elevon_pwm.set_level(@intFromFloat(elevon_mapping.map(left_elevon)));
            config.right_elevon_pwm.set_level(@intFromFloat(elevon_mapping.map(right_elevon)));
        }

        rp2xxx.time.sleep_ms(10);
    }
}

const Mapping = struct {
    start_min: f32,
    start_max: f32,
    end_min: f32,
    end_max: f32,

    pub fn map(self: Mapping, x: f32) f32 {
        // std.debug.assert(x.greaterOrEqual(self.start_min) and x.lessOrEqual(self.start_max));

        return self.end_min + (x - self.start_min) * (self.end_max - self.end_min) / (self.start_max - self.start_min);
    }
};

const throttle_mapping: Mapping = .{
    .start_min = 1000,
    .start_max = 2000,
    .end_min = config.ESC_PWM_MIN_LEVEL,
    .end_max = config.ESC_PWM_MAX_LEVEL,
};

const roll_pitch_mapping: Mapping = .{
    .start_min = 1000,
    .start_max = 2000,
    .end_min = -1,
    .end_max = 1,
};

const elevon_mapping: Mapping = .{
    .start_min = -1,
    .start_max = 1,
    .end_min = config.ELEVON_PWM_MIN_LEVEL,
    .end_max = config.ELEVON_PWM_MAX_LEVEL,
};

const channel_to_us_mapping: Mapping = .{
    .start_min = 191,
    .start_max = 1792,
    .end_min = 1000,
    .end_max = 2000,
};

test "mapping" {
    try std.testing.expectEqual(throttle_mapping.map(.fromInt(1000)).toInt(), config.ESC_PWM_MIN_LEVEL);
    try std.testing.expectEqual(throttle_mapping.map(.fromInt(2000)).toInt(), config.ESC_PWM_MAX_LEVEL);
    try std.testing.expectEqual(elevon_mapping.map(.fromInt(-1)).toInt(), config.ELEVON_PWM_MIN_LEVEL);
    try std.testing.expectEqual(elevon_mapping.map(.fromInt(1)).toInt(), config.ELEVON_PWM_MAX_LEVEL);
}

pub const Control = struct {
    pub const idle: Control = .{
        .throttle = 1000,
        .yaw = 1500,
        .pitch = 1500,
        .roll = 1500,
    };

    throttle: u16,
    yaw: u16,
    pitch: u16,
    roll: u16,
};

fn UART0_IRQ() callconv(.c) void {
    const static = struct {
        var crsf_parser: CRSF_Parser = .{};
    };

    while (true) {
        const maybe_byte = config.rx_uart.read_word() catch |err| {
            config.rx_uart.clear_errors();
            std.log.info("receiver uart read error: {}", .{err});
            continue;
        };

        const byte = maybe_byte orelse break;

        {
            const cs = microzig.interrupt.enter_critical_section();
            defer cs.leave();

            const maybe_packet = static.crsf_parser.push_byte(byte) catch |err| {
                std.log.err("crsf parser error: {}", .{err});
                continue;
            };

            if (maybe_packet) |packet| switch (packet) {
                .rc_channels_packed => |rc_channels_packed| {
                    // SAFETY: aren't modified in other interrupts with greater priority

                    for (rc_channels_packed, &state.channels) |in, *out| {
                        out.* = @intFromFloat(channel_to_us_mapping.map(@floatFromInt(in)));
                    }

                    state.last_received_channels_time = rp2xxx.time.get_time_since_boot();
                },
                else => {},
            };
        }
    }
}

test "CRSF_Parser" {
    _ = CRSF_Parser;
}
