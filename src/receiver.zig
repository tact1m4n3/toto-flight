const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;

const persistent = @import("persistent.zig");
const control = @import("control.zig");
const hw = @import("hw.zig");
const imu = @import("imu.zig");
const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const Task = Scheduler.Task;

const log = std.log.scoped(.receiver);

pub var msg_channels: Message(Channels) = .{};
pub var msg_command: Message(control.Command) = .{};

pub const Rx = struct {
    parser: switch (hw.def.receiver.protocol) {
        .crsf => @import("parsers/CRSF.zig"),
    } = .{},

    pub fn init(rx: *Rx, scheduler: *Scheduler) void {
        rx.* = .{};
        hw.UART_RX.receiver.subscribe(*Rx, rx, switch (hw.def.receiver.protocol) {
            .crsf => receive_byte_crsf,
        }, scheduler);
    }

    fn receive_byte_crsf(rx: *Rx, byte: u8) void {
        const maybe_packet = rx.parser.push_byte(byte) catch |err| {
            log.warn("failed to parse crsf packet: {t}", .{err});
            return;
        };

        if (maybe_packet) |packet| {
            switch (packet) {
                .rc_channels_packed => |channels_crsf| {
                    var channels_us: [16]u16 = undefined;
                    for (&channels_us, channels_crsf) |*value_us, value_crsf| {
                        value_us.* = @truncate(@as(u32, value_crsf) * 1024 / 1639 + 881);
                    }
                    log.info("received channels: {any}", .{channels_us});
                    const channels: Channels = .init(.aetr1234, &channels_us);
                    msg_channels.publish(channels);
                },
                .link_statistics => |ls| {
                    log.info("received link stats: {any}", .{ls});
                },
            }
        }
    }
};

pub const ChannelMapper = struct {
    arm_switch: ChannelCondition,
    rate_mode_enable: ChannelCondition,
    angle_mode_enable: ChannelCondition,
    calibrate_imu_action: ChannelDetector,
    save_config_action: ChannelDetector,

    rcv_channels: Receiver(Channels) = undefined,

    pub fn init(mapper: *ChannelMapper, scheduler: *Scheduler) void {
        mapper.* = .{
            .arm_switch = .{
                .ident = .{ .index = 4 },
                .range = .{ .start = 1700, .end = 2100 },
            },
            .rate_mode_enable = .{
                .ident = .{ .index = 6 },
                .range = .{ .start = 1300, .end = 1700 },
            },
            .angle_mode_enable = .{
                .ident = .{ .index = 6 },
                .range = .{ .start = 1800, .end = 2100 },
            },
            // TEMP
            .calibrate_imu_action = .{ .cond = .{
                .ident = .{ .index = 7 },
                .range = .{ .start = 1300, .end = 1700 },
            } },
            .save_config_action = .{ .cond = .{
                .ident = .{ .index = 7 },
                .range = .{ .start = 1800, .end = 2100 },
            } },
        };

        msg_channels.subscribe(&mapper.rcv_channels, *ChannelMapper, mapper, channels_callback, scheduler);
    }

    fn channels_callback(mapper: *ChannelMapper, channels: Channels) void {
        const RATE_MODE_MULT = math.degrees_to_radians(50.0);
        const ANGLE_MODE_MULT = math.degrees_to_radians(50.0);

        const arm_state = mapper.arm_switch.get(&channels);
        const rate_mode = mapper.rate_mode_enable.get(&channels);
        const angle_mode = mapper.angle_mode_enable.get(&channels);

        if (mapper.calibrate_imu_action.detect(&channels)) |calibration_request| {
            if (calibration_request) {
                log.info("imu calibration requested", .{});
                imu.msg_calibrate.publish({});
            }
        }

        if (mapper.save_config_action.detect(&channels)) |store_request| {
            if (store_request) {
                log.info("config save requested", .{});
                persistent.msg_save.publish({});
            }
        }

        const throttle = us_to_0_1(channels.get(.throttle));
        const roll = us_to_neg1_1(channels.get(.roll));
        const pitch = us_to_neg1_1(channels.get(.pitch));
        const yaw = us_to_neg1_1(channels.get(.yaw));

        const command: control.Command =
            if (arm_state)
                if (angle_mode) .{
                    .angle = .{
                        .throttle = throttle * ANGLE_MODE_MULT,
                        .roll = roll * ANGLE_MODE_MULT,
                        .pitch = pitch * ANGLE_MODE_MULT,
                        // .yaw = yaw * ANGLE_MODE_MULT,
                    },
                } else if (rate_mode) .{ .rate = .{
                    .throttle = throttle * RATE_MODE_MULT,
                    .roll = roll * RATE_MODE_MULT,
                    .pitch = pitch * RATE_MODE_MULT,
                    .yaw = yaw * RATE_MODE_MULT,
                } } else .{ .manual = .{
                    .throttle = throttle,
                    .roll = roll,
                    .pitch = pitch,
                    .yaw = yaw,
                } }
            else
                .disarm;

        msg_command.publish(command);
    }
};

const CHANNEL_MIN_VALUE = 1000;
const CHANNEL_MID_VALUE = 1500;
const CHANNEL_MAX_VALUE = 2000;

fn us_to_neg1_1(us: u16) f32 {
    return @as(f32, @floatFromInt(us - CHANNEL_MIN_VALUE)) / ((CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE) / 2.0) - 1.0;
}

fn us_to_0_1(us: u16) f32 {
    return @as(f32, @floatFromInt(us - CHANNEL_MIN_VALUE)) / (CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE);
}

pub const ChannelIndex = u4;

pub const ChannelIdent = union(enum) {
    throttle,
    roll,
    pitch,
    yaw,
    index: ChannelIndex,
};

pub const Channels = struct {
    // throttle, roll, pitch, yaw, aux1, aux2, ...
    values: [16]u16,

    pub const default: Channels = .{
        .values = .{1000} ++ @as([15]u16, @splat(1500)),
    };

    pub fn init(format: enum {
        aetr1234,
    }, raw: *const [16]u16) Channels {
        var values: [16]u16 = undefined;
        const taer = switch (format) {
            .aetr1234 => &.{ raw[2], raw[0], raw[1], raw[3] },
        };
        std.mem.copyForwards(u16, values[0..4], taer);
        std.mem.copyForwards(u16, values[4..16], raw[4..16]);
        for (&values) |*value| {
            value.* = std.math.clamp(value.*, CHANNEL_MIN_VALUE, CHANNEL_MAX_VALUE);
        }
        return .{ .values = values };
    }

    pub fn get(self: *const Channels, ident: ChannelIdent) u16 {
        return switch (ident) {
            .throttle => self.values[0],
            .roll => self.values[1],
            .pitch => self.values[2],
            .yaw => self.values[3],
            // NOTE: we don't need to assert index is less than 16 because we
            // use u4 as data type
            .index => |index| self.values[index],
        };
    }
};

const ChannelCondition = struct {
    ident: ChannelIdent,
    range: ChannelRange,

    pub fn get(cond: *const ChannelCondition, channels: *const Channels) bool {
        const value = channels.get(cond.ident);
        return cond.range.contains(value);
    }
};

const ChannelDetector = struct {
    cond: ChannelCondition,
    state: bool = false,

    pub fn detect(detector: *ChannelDetector, channels: *const Channels) ?bool {
        const new_state = detector.cond.get(channels);
        if (detector.state != new_state) {
            detector.state = new_state;
            return new_state;
        }
        return null;
    }
};

pub const ChannelRange = struct {
    start: u16,
    end: u16,

    pub fn contains(range: ChannelRange, value: u16) bool {
        return range.start <= value and value <= range.end;
    }
};

test "ChannelRange.contains" {
    const range: ChannelRange = .{ .start = 1000, .end = 2000 };
    try testing.expect(range.contains(1500));
    try testing.expect(range.contains(1000));
    try testing.expect(range.contains(2000));
    try testing.expect(!range.contains(999));
    try testing.expect(!range.contains(2001));
}

test "Channels.init" {
    const raw: [16]u16 = .{ 500, 1200, 1100, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };

    {
        const channels: Channels = .init(.aetr1234, &raw);
        try testing.expectEqual(1100, channels.get(.throttle));
        try testing.expectEqual(1000, channels.get(.roll)); // gets clamped
        try testing.expectEqual(1200, channels.get(.pitch));
        try testing.expectEqual(1500, channels.get(.yaw));
        try testing.expectEqual(1500, channels.get(.{ .index = 5 }));
    }
}

test "ChannelCondition.get" {
    var cond: ChannelCondition = .{ .ident = .{ .index = 4 }, .range = .{ .start = 1700, .end = 2000 } };
    const raw_off: [16]u16 = @splat(1500);
    const raw_on: [16]u16 = blk: {
        var r: [16]u16 = @splat(1500);
        r[4] = 1800;
        break :blk r;
    };
    const ch_off = Channels.init(.aetr1234, &raw_off);
    const ch_on = Channels.init(.aetr1234, &raw_on);

    // the state doesn't change because it starts as false
    try testing.expectEqual(false, cond.update(&ch_off));
    try testing.expectEqual(true, cond.update(&ch_on));
    try testing.expectEqual(false, cond.update(&ch_off));
}
