const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;

const time = @import("time.zig");
const storage = @import("storage.zig");
const control = @import("control.zig");
const hw = @import("hw.zig");
const imu = @import("imu.zig");
const math = @import("math.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const Task = Scheduler.Task;
const crsf = @import("protocols/crsf.zig");
const mavlink = @import("protocols/mavlink.zig");

const log = std.log.scoped(.receiver);

pub var msg_channels: Message(Channels) = .{};
pub var msg_command: Message(control.Command) = .{};

pub const Rx = struct {
    inner: switch (hw.def.receiver.protocol) {
        .crsf => Rx_CRSF,
        .mavlink => RxMavlink,
    } = undefined,

    pub fn init(rx: *Rx, scheduler: *Scheduler) void {
        rx.inner.init(scheduler);
    }
};

pub const Rx_CRSF = struct {
    rcv_tick: Receiver(void) = undefined,

    parser: crsf.Parser = .{},

    pub fn init(rx: *Rx_CRSF, scheduler: *Scheduler) void {
        rx.* = .{};

        hw.UART.get(.receiver).subscribe(*Rx_CRSF, rx, receive_callback, scheduler);
    }

    fn receive_callback(rx: *Rx_CRSF, byte: u8) void {
        const maybe_packet = rx.parser.push_byte(byte) catch |err| {
            log.warn("failed to parse crsf packet: {t}", .{err});
            return;
        };

        if (maybe_packet) |packet| {
            switch (packet) {
                .rc_channels_packed => |channels_crsf| {
                    var channels_us: [16]u16 = undefined;
                    for (&channels_us, channels_crsf) |*value_us, value_crsf| {
                        // NOTE: can't overflow: 2 ^ 11 * 2 ^ 10 < 2 ^ 32
                        value_us.* = @truncate(@as(u32, value_crsf) * 1024 / 1639 + 881);
                    }
                    // log.info("received channels: {any}", .{channels_us});
                    const channels: Channels = .init(.aetr1234, &channels_us);
                    msg_channels.publish(channels);
                },
                .link_statistics => |ls| {
                    _ = ls;
                    // log.info("received link stats: {any}", .{ls});
                },
            }
        }
    }
};

pub const RxMavlink = struct {
    // TODO: PARAM_REQUEST_LIST
    //
    // IDK if we need:
    // TODO: COMPONENT_INFORMATION
    // TODO: GIMBAL_MANAGER_INFORMATION
    //
    // TODO: separate code into microservices

    const uart = hw.UART.get(.receiver);
    const send_tick_rate: hw.TickRate = .@"100Hz";

    const sysid = 1;
    const capabilities: mavlink.enums.MAV_PROTOCOL_CAPABILITY = .{
        .MAV_PROTOCOL_CAPABILITY_MAVLINK2 = true,
        .MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE = true,
    };
    const flight_modes: []const struct {
        name: ?[]const u8 = null,
        standard: mavlink.enums.MAV_STANDARD_MODE = .MAV_STANDARD_MODE_NON_STANDARD,
        properties: mavlink.enums.MAV_MODE_PROPERTY,
    } = &.{
        .{ .name = "manual", .properties = .{ .MAV_MODE_PROPERTY_ADVANCED = true } },
        .{ .name = "rate", .properties = .{ .MAV_MODE_PROPERTY_ADVANCED = true } },
        .{ .name = "angle", .properties = .{ .MAV_MODE_PROPERTY_ADVANCED = true } },
        .{ .standard = .MAV_STANDARD_MODE_CRUISE, .properties = .{ .MAV_MODE_PROPERTY_AUTO_MODE = true } },
    };

    rcv_tick: Receiver(void) = undefined,

    parser: mavlink.Parser = .{},

    tick_count: u32 = 0,
    gcs_connected: bool = false,
    heartbeat_seq: u8 = 0,
    one_shot: packed struct {
        autopilot_version: bool = false,
        mission_count: bool = false,
    } = .{},

    send_available_modes: SendAvailableModes = .idle,

    const SendAvailableModes = struct {
        specific: bool,
        current_index: u8,
        state: enum {
            idle,
            ack,
            send,
        },

        pub const idle: SendAvailableModes = .{
            .specific = false,
            .current_index = 0,
            .state = .idle,
        };
    };

    pub fn init(rx: *RxMavlink, scheduler: *Scheduler) void {
        rx.* = .{};

        uart.subscribe(*RxMavlink, rx, receive_callback, scheduler);
        hw.ticker(send_tick_rate).subscribe(&rx.rcv_tick, *RxMavlink, rx, send_callback, scheduler);
    }

    fn receive_callback(rx: *RxMavlink, byte: u8) void {
        const maybe_packet = rx.parser.push_byte(byte) catch |err| {
            switch (err) {
                error.UnknownMessageId, error.UnimplementedMessage => {
                    log.warn("unknown message id: 0x{x:0>6}", .{rx.parser.packet.get_msgid()});
                },
                else => log.warn("failed to parse mavlink packet: {t}", .{err}),
            }
            return;
        };

        if (maybe_packet) |packet| {
            switch (packet) {
                .HEARTBEAT => |heartbeat| {
                    if (heartbeat.type == .MAV_TYPE_GCS and !rx.gcs_connected) {
                        log.info("gcs connected", .{});
                        rx.gcs_connected = true;
                    }
                },
                .COMMAND_LONG => |command| switch (command.command) {
                    .MAV_CMD_REQUEST_MESSAGE => {
                        if (std.enums.fromInt(mavlink.MessageId, std.math.lossyCast(u24, command.param1))) |id| {
                            switch (id) {
                                .AUTOPILOT_VERSION => rx.one_shot.autopilot_version = true,
                                .AVAILABLE_MODES => if (rx.send_available_modes.state == .idle) {
                                    const index = std.math.lossyCast(u8, command.param2);
                                    rx.send_available_modes = .{
                                        .specific = index != 0,
                                        .current_index = if (index != 0) index else 1,
                                        .state = .ack,
                                    };
                                },
                                else => log.warn("unknown message request: {t}", .{id}),
                            }
                        }
                    },
                    else => std.log.warn("unknown command: {t}", .{command.command}),
                },
                .MISSION_REQUEST_LIST => {
                    // TODO: state machine for mission download
                    rx.one_shot.mission_count = true;
                },
                else => {},
            }
        }
    }

    fn send_callback(rx: *RxMavlink, _: void) void {
        if (!rx.gcs_connected) return;

        defer rx.tick_count +%= 1;

        if (rx.tick_count % 100 == 0) {
            send_message(&.{ .HEARTBEAT = .{
                .custom_mode = 0,
                .type = .MAV_TYPE_FIXED_WING,
                .autopilot = .MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
                .base_mode = .{},
                .system_status = .MAV_STATE_STANDBY,
                .mavlink_version = 3,
            } }, .{
                .seq = rx.heartbeat_seq,
            }) catch return;
            rx.heartbeat_seq +%= 1;
        }

        if (rx.one_shot.autopilot_version) {
            send_message(&.{
                .AUTOPILOT_VERSION = .{
                    .capabilities = capabilities,

                    // Unused for now
                    .flight_custom_version = @splat(0),
                    .middleware_custom_version = @splat(0),
                    .os_custom_version = @splat(0),
                    .uid = 0,
                    .flight_sw_version = 0,
                    .middleware_sw_version = 0,
                    .os_sw_version = 0,
                    .board_version = 0,
                    .vendor_id = 0,
                    .product_id = 0,
                },
            }, .{}) catch return;
            rx.one_shot.autopilot_version = false;
        }

        if (rx.one_shot.mission_count) {
            send_message(&.{ .MISSION_COUNT = .{
                .count = 0,
                .target_system = sysid,
                .target_component = @backingInt(mavlink.enums.MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1),
            } }, .{}) catch return;
            rx.one_shot.mission_count = false;
        }

        send_available_modes: switch (rx.send_available_modes.state) {
            .idle => {},
            .ack => {
                if (rx.send_available_modes.current_index == 0 or rx.send_available_modes.current_index > flight_modes.len) {
                    send_message(&.{ .COMMAND_ACK = .{
                        .command = .MAV_CMD_REQUEST_MESSAGE,
                        .result = .MAV_RESULT_DENIED,
                    } }, .{}) catch return;
                    log.warn("invalid available modes request: {d}", .{rx.send_available_modes.current_index});
                    rx.send_available_modes.state = .idle;
                    break :send_available_modes;
                } else {
                    send_message(&.{ .COMMAND_ACK = .{
                        .command = .MAV_CMD_REQUEST_MESSAGE,
                        .result = .MAV_RESULT_ACCEPTED,
                    } }, .{}) catch return;
                    rx.send_available_modes.state = .send;
                }
            },
            .send => {
                assert(rx.send_available_modes.current_index > 0);
                assert(rx.send_available_modes.current_index <= flight_modes.len);

                const mode = &flight_modes[rx.send_available_modes.current_index - 1];
                var name: [35]u8 = @splat(0);
                {
                    const src_name = mode.name orelse "";
                    std.mem.copyForwards(u8, name[0..src_name.len], src_name);
                }
                send_message(&.{
                    .AVAILABLE_MODES = .{
                        .mode_name = @bitCast(name),
                        .custom_mode = 0,
                        .properties = mode.properties,
                        .number_modes = @truncate(flight_modes.len),
                        .mode_index = rx.send_available_modes.current_index,
                        .standard_mode = mode.standard,
                    },
                }, .{}) catch return;

                if (rx.send_available_modes.current_index == flight_modes.len or
                    rx.send_available_modes.specific)
                {
                    rx.send_available_modes.state = .idle;
                } else {
                    rx.send_available_modes.current_index += 1;
                    continue :send_available_modes .send;
                }
            },
        }
    }

    fn send_message(message: *const mavlink.OutgoingMessage, options: struct {
        seq: u8 = 0,
    }) !void {
        var buf: [mavlink.MAX_PACKET_LEN]u8 = undefined;
        const len = mavlink.serialize(
            &buf,
            message,
            .{
                .seq = options.seq,
                .sysid = sysid,
                .compid = @backingInt(mavlink.enums.MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1),
            },
        ) catch unreachable;
        try uart.write(buf[0..len]);
    }
};

// pub const Periodic = struct {
//     const max_messages = 10;
//     const Item = struct {
//         offset: u32,
//         period_in_ticks: u32,
//         message_id: mavlink.MessageId,
//         ready: bool = false,
//     };
//
//     tick_period: time.Duration,
//     count: u32 = 0,
//     messages: bounded.Array(Item, max_messages) = .empty,
//
//     pub fn add(periodic: *Periodic, message_id: mavlink.MessageId, period: time.Duration) !void {
//         const period_in_ticks: u32 = @truncate(period.to_us() / periodic.tick_period.to_us());
//         if (period_in_ticks == 0) {
//             return error.PeriodTooSmall;
//         }
//
//         // trick to sparse the messages over the period, so they don't all
//         // get sent at the same time
//         const offset = periodic.count % period_in_ticks;
//         periodic.count +%= 1;
//
//         for (periodic.messages.items()) |*item| {
//             if (item.message_id == message_id) {
//                 item.offset = offset;
//                 item.period_in_ticks = period_in_ticks;
//                 break;
//             }
//         } else try periodic.messages.append(.{
//             .offset = offset,
//             .period_in_ticks = period_in_ticks,
//             .message_id = message_id,
//         });
//     }
//
//     pub fn tick(periodic: *Periodic) void {
//         for (periodic.messages.items()) |*item| {
//             if ((periodic.count +% item.offset) % item.period_in_ticks == 0) {
//                 item.ready = true;
//             }
//         }
//         periodic.count +%= 1;
//     }
// };

pub const ChannelMapper = struct {
    arm_switch: ChannelCondition,
    rate_mode_enable: ChannelCondition,
    angle_mode_enable: ChannelCondition,
    action_calibrate_imu: ChannelDetector,
    action_save_config: ChannelDetector,

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
            .action_calibrate_imu = .{
                .cond = .{
                    .ident = .{ .index = 7 },
                    .range = .{ .start = 1300, .end = 1700 },
                },
                .debounce = 10,
            },
            .action_save_config = .{
                .cond = .{
                    .ident = .{ .index = 7 },
                    .range = .{ .start = 1800, .end = 2100 },
                },
                .debounce = 10,
            },
        };

        msg_channels.subscribe(&mapper.rcv_channels, *ChannelMapper, mapper, channels_callback, scheduler);
    }

    fn channels_callback(mapper: *ChannelMapper, channels: Channels) void {
        const RATE_MODE_MULT = math.radians_from_degrees(50.0);
        const ANGLE_MODE_MULT = math.radians_from_degrees(50.0);

        const arm_state = mapper.arm_switch.get(&channels);
        const rate_mode = mapper.rate_mode_enable.get(&channels);
        const angle_mode = mapper.angle_mode_enable.get(&channels);

        if (mapper.action_calibrate_imu.detect(&channels)) |calibration_request| {
            if (calibration_request) {
                log.info("imu calibration requested", .{});
                // imu.msg_calibrate.publish({});
            }
        }

        if (mapper.action_save_config.detect(&channels)) |store_request| {
            if (store_request) {
                log.info("config save requested", .{});
                // storage.msg_save.publish({});
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
                        .throttle = throttle,
                        .angle_roll = roll * ANGLE_MODE_MULT,
                        .angle_pitch = pitch * ANGLE_MODE_MULT,
                        // .yaw = yaw * ANGLE_MODE_MULT,
                    },
                } else if (rate_mode) .{ .rate = .{
                    .throttle = throttle,
                    .rate = .{
                        .x = roll * RATE_MODE_MULT,
                        .y = pitch * RATE_MODE_MULT,
                        .z = yaw * RATE_MODE_MULT,
                    },
                } } else .{ .manual = .{
                    .throttle = throttle,
                    .throw = .{
                        .x = roll,
                        .y = pitch,
                        .z = yaw,
                    },
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
    const clamped = std.math.clamp(us, CHANNEL_MIN_VALUE, CHANNEL_MAX_VALUE);
    return @as(f32, @floatFromInt(clamped - CHANNEL_MIN_VALUE)) / ((CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE) / 2.0) - 1.0;
}

fn us_to_0_1(us: u16) f32 {
    const clamped = std.math.clamp(us, CHANNEL_MIN_VALUE, CHANNEL_MAX_VALUE);
    return @as(f32, @floatFromInt(clamped - CHANNEL_MIN_VALUE)) / (CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE);
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
    debounce: u16 = 0,
    state: bool = false,

    debounce_remaining: ?u16 = null,

    pub fn detect(detector: *ChannelDetector, channels: *const Channels) ?bool {
        const new_state = detector.cond.get(channels);
        if (detector.state != new_state) {
            detector.debounce_remaining = detector.debounce;
            detector.state = new_state;
        }
        if (detector.debounce_remaining) |*debounce_remaining| {
            if (debounce_remaining.* == 0) {
                detector.debounce_remaining = null;
                return detector.state;
            } else {
                debounce_remaining.* -= 1;
            }
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

    try testing.expectEqual(false, cond.get(&ch_off));
    try testing.expectEqual(true, cond.get(&ch_on));
    try testing.expectEqual(false, cond.get(&ch_off));
}

test "ChannelDetector.detect" {
    var detector: ChannelDetector = .{ .cond = .{
        .ident = .{ .index = 4 },
        .range = .{ .start = 1700, .end = 2000 },
    } };
    const raw_off: [16]u16 = @splat(1500);
    const raw_on: [16]u16 = blk: {
        var r: [16]u16 = @splat(1500);
        r[4] = 1800;
        break :blk r;
    };
    const ch_off = Channels.init(.aetr1234, &raw_off);
    const ch_on = Channels.init(.aetr1234, &raw_on);

    try testing.expectEqual(null, detector.detect(&ch_off));
    try testing.expectEqual(true, detector.detect(&ch_on));
    try testing.expectEqual(null, detector.detect(&ch_on));
    try testing.expectEqual(false, detector.detect(&ch_off));
    try testing.expectEqual(null, detector.detect(&ch_off));

    detector.debounce = 4;
    try testing.expectEqual(null, detector.detect(&ch_on));
    try testing.expectEqual(null, detector.detect(&ch_on));
    try testing.expectEqual(null, detector.detect(&ch_on));
    try testing.expectEqual(null, detector.detect(&ch_on));
    try testing.expectEqual(true, detector.detect(&ch_on));
}
