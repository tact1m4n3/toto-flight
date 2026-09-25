const std = @import("std");
const assert = std.debug.assert;
const testing = std.testing;

const bounded = @import("utils/bounded.zig");
const time = @import("time.zig");
const storage = @import("storage.zig");
const control = @import("control.zig");
const parameter = @import("parameter.zig");
const hw = @import("hw.zig");
const fusion = @import("fusion.zig");
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
    // TODO: SYSTEM_TIME
    // TODO: MISSION_ACK
    //
    // IDK if we need:
    // TODO: COMPONENT_INFORMATION
    // TODO: GIMBAL_MANAGER_INFORMATION
    //
    // TODO: configurable at comptime/runtime

    const uart = hw.UART.get(.receiver);
    const send_tick_rate: hw.TickRate = .@"100Hz";

    const min_txbuf = 30;
    const max_bitrate = 100; // bytes per second
    const bitrate_inc_per_tick = @max(max_bitrate * send_tick_rate.get_period().to_ms() / 1_000, 1);

    const sysid = 1;
    const compid = @backingInt(mavlink.enums.MAV_COMPONENT.MAV_COMP_ID_AUTOPILOT1);
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
    const command_ack_queue_capacity = 10;

    rcv_tick: Receiver(void) = undefined,
    param_watcher: parameter.Watcher = .{},

    parser: mavlink.Parser = .{},

    txbuf: u8 = 100,
    rate_limiter: u32 = max_bitrate,

    tick_count: u32 = 0,
    gcs_connected: bool = false,
    heartbeat_seq: u8 = 0,
    periodic: Periodic = .{ .tick_period = send_tick_rate.get_period() },
    one_shot: packed struct {
        autopilot_version: bool = false,
        mission_count: bool = false,
    } = .{},
    command_ack: bounded.Queue(CommandAck, command_ack_queue_capacity) = .empty,
    send_available_modes: ?SendAvailableModes = null,

    const CommandAck = struct {
        command: mavlink.enums.MAV_CMD,
        result: mavlink.enums.MAV_RESULT,
    };

    const SendAvailableModes = struct {
        specific: bool,
        current_index: u8,
    };

    const ParamId = struct {
        buf: [16]u8,

        pub fn from_raw(raw: [16]i8) ParamId {
            return .{ .buf = @bitCast(raw) };
        }

        pub fn to_raw(id: ParamId) [16]i8 {
            return @bitCast(id.buf);
        }

        pub fn from_slice(id: []const u8) ?ParamId {
            if (id.len > 16) return null;
            var buf: [16]u8 = @splat(0);
            std.mem.copyForwards(u8, buf[0..id.len], id);
            return .{ .buf = buf };
        }

        pub fn as_slice(id: *const ParamId) []const u8 {
            const end = std.mem.findScalar(u8, &id.buf, 0) orelse id.buf.len;
            return id.buf[0..end];
        }
    };

    pub fn init(rx: *RxMavlink, scheduler: *Scheduler) void {
        rx.* = .{};

        uart.subscribe(*RxMavlink, rx, receive_callback, scheduler);
        hw.ticker(send_tick_rate).subscribe(&rx.rcv_tick, *RxMavlink, rx, send_callback, scheduler);

        parameter.register_watcher(&rx.param_watcher);

        rx.periodic.add(.HEARTBEAT, comptime .from_hz(1)) catch unreachable;
        rx.periodic.add(.ATTITUDE_QUATERNION, comptime .from_hz(1)) catch unreachable;
    }

    fn receive_callback(rx: *RxMavlink, byte: u8) void {
        const maybe_message = rx.parser.push_byte(byte) catch |err| {
            switch (err) {
                error.UnknownMessageId, error.UnimplementedMessage => {
                    log.warn("unknown message id: 0x{x:0>6}", .{rx.parser.packet.get_msgid()});
                },
                else => log.warn("failed to parse mavlink packet: {t}", .{err}),
            }
            return;
        };

        if (maybe_message) |packet| {
            // if (rx.parser.packet.sysid != sysid or rx.parser.packet.compid != compid) {
            //     return;
            // }

            const armed = control.get_status().armed;

            switch (packet) {
                .RADIO_STATUS => |status| {
                    rx.txbuf = status.txbuf;
                },
                .RC_CHANNELS_OVERRIDE => |rc| {
                    const S = struct {
                        fn process_channel(raw: u16) u16 {
                            if (raw != 0 and raw != std.math.maxInt(u16)) {
                                return raw;
                            } else {
                                return 1500;
                            }
                        }
                    };
                    const raw_channels: [16]u16 = .{
                        S.process_channel(rc.chan1_raw),
                        S.process_channel(rc.chan2_raw),
                        S.process_channel(rc.chan3_raw),
                        S.process_channel(rc.chan4_raw),
                        S.process_channel(rc.chan5_raw),
                        S.process_channel(rc.chan6_raw),
                        S.process_channel(rc.chan7_raw),
                        S.process_channel(rc.chan8_raw),
                        S.process_channel(rc.chan9_raw),
                        S.process_channel(rc.chan10_raw),
                        S.process_channel(rc.chan11_raw),
                        S.process_channel(rc.chan12_raw),
                        S.process_channel(rc.chan13_raw),
                        S.process_channel(rc.chan14_raw),
                        S.process_channel(rc.chan15_raw),
                        S.process_channel(rc.chan16_raw),
                    };
                    const channels: Channels = .init(.aetr1234, &raw_channels);
                    msg_channels.publish(channels);
                },
                .HEARTBEAT => |heartbeat| {
                    if (heartbeat.type == .MAV_TYPE_GCS and !rx.gcs_connected) {
                        log.info("gcs connected", .{});
                        rx.gcs_connected = true;
                    }
                },
                .COMMAND_LONG => |command| {
                    var ack_result: mavlink.enums.MAV_RESULT = .MAV_RESULT_UNSUPPORTED;

                    switch (command.command) {
                        .MAV_CMD_REQUEST_MESSAGE => {
                            if (std.enums.fromInt(mavlink.MessageId, std.math.lossyCast(u24, command.param1))) |id| {
                                switch (id) {
                                    .AUTOPILOT_VERSION => {
                                        rx.one_shot.autopilot_version = true;
                                        ack_result = .MAV_RESULT_ACCEPTED;
                                    },
                                    .AVAILABLE_MODES => if (rx.send_available_modes == null) {
                                        const raw_index = std.math.lossyCast(u8, command.param2);
                                        const index = if (raw_index != 0) raw_index else 1;
                                        if (index <= flight_modes.len) {
                                            rx.send_available_modes = .{
                                                .specific = index != 0,
                                                .current_index = if (index != 0) index else 1,
                                            };
                                            ack_result = .MAV_RESULT_ACCEPTED;
                                        } else {
                                            log.warn("invalid available modes request: {d}", .{index});
                                            ack_result = .MAV_RESULT_DENIED;
                                        }
                                    } else {
                                        ack_result = .MAV_RESULT_IN_PROGRESS;
                                    },
                                    else => log.warn("unknown message request: {t}", .{id}),
                                }
                            }
                        },
                        .MAV_CMD_PREFLIGHT_CALIBRATION => if (!armed) {
                            if (command.param1 == 1.0 or command.param5 == 1.0) {
                                log.info("imu calibration requested", .{});
                                imu.msg_calibrate.publish({});
                                ack_result = .MAV_RESULT_ACCEPTED;
                            }
                        },
                        .MAV_CMD_PREFLIGHT_STORAGE => if (!armed) {
                            if (command.param1 == 1.0) {
                                log.info("config save to flash requested", .{});
                                storage.msg_save.publish({});
                                ack_result = .MAV_RESULT_ACCEPTED;
                            } else if (command.param1 == 0.0) {
                                log.info("config load from flash requested", .{});
                                storage.msg_load.publish({});
                                ack_result = .MAV_RESULT_ACCEPTED;
                            } else if (command.param1 == 2.0) {
                                log.info("reset config to default", .{});
                                parameter.reset_to_default();
                                ack_result = .MAV_RESULT_ACCEPTED;
                            }
                        },
                        else => std.log.warn("unknown command: {t}", .{command.command}),
                    }

                    rx.command_ack.append(.{
                        .command = command.command,
                        .result = ack_result,
                    }) catch {
                        log.warn("too many command ack...", .{});
                    };
                },
                .MISSION_REQUEST_LIST => {
                    // TODO: state machine for mission download
                    rx.one_shot.mission_count = true;
                    log.info("mission request list received, sending mission count", .{});
                },

                .PARAM_REQUEST_LIST => {
                    // we set all bits so that the param sender thinks all
                    // params have changed and will send them all
                    rx.param_watcher.set(.full);
                    log.info("param request list received, sending all params", .{});
                },
                .PARAM_REQUEST_READ => |request| {
                    const maybe_index = std.math.cast(u16, request.param_index) orelse blk: {
                        const param_id: [16]u8 = @bitCast(request.param_id);
                        break :blk parameter.id_to_index.get(&param_id);
                    };

                    log.info("param request read received, index: {d}", .{maybe_index orelse 0});
                    if (maybe_index) |index| {
                        // we set a specific bit so that the param sender thinks
                        // this param changed and it will send it
                        rx.param_watcher.set_one(index);
                    } else {
                        log.warn("read param not found", .{});
                    }
                },
                .PARAM_SET => |set| {
                    const param_id: ParamId = .from_raw(set.param_id);
                    const value: parameter.AnyValue = switch (set.param_type) {
                        .MAV_PARAM_TYPE_INT8 => .{ .i8 = std.math.lossyCast(i8, set.param_value) },
                        .MAV_PARAM_TYPE_INT16 => .{ .i16 = std.math.lossyCast(i16, set.param_value) },
                        .MAV_PARAM_TYPE_INT32 => .{ .i32 = std.math.lossyCast(i32, set.param_value) },
                        .MAV_PARAM_TYPE_INT64 => .{ .i64 = std.math.lossyCast(i64, set.param_value) },
                        .MAV_PARAM_TYPE_UINT8 => .{ .u8 = std.math.lossyCast(u8, set.param_value) },
                        .MAV_PARAM_TYPE_UINT16 => .{ .u16 = std.math.lossyCast(u16, set.param_value) },
                        .MAV_PARAM_TYPE_UINT32 => .{ .u32 = std.math.lossyCast(u32, set.param_value) },
                        .MAV_PARAM_TYPE_UINT64 => .{ .u64 = std.math.lossyCast(u64, set.param_value) },
                        .MAV_PARAM_TYPE_REAL32 => .{ .f32 = set.param_value },
                        .MAV_PARAM_TYPE_REAL64 => .{ .f64 = set.param_value },
                    };
                    log.info("param set received, id: {s}, value: {any}", .{ param_id.as_slice(), value });
                    parameter.modify_dyn_with_id(param_id.as_slice(), value) catch |err| {
                        log.warn("failed to set param {s}: {}", .{ param_id.as_slice(), err });
                    };
                },
            }
        }
    }

    fn send_callback(rx: *RxMavlink, _: void) void {
        if (!rx.gcs_connected) return;

        rx.rate_limiter += bitrate_inc_per_tick;
        rx.rate_limiter = @min(rx.rate_limiter, max_bitrate);

        defer rx.tick_count +%= 1;

        {
            var limit: usize = rx.command_ack.len;
            while (limit > 0) : (limit -= 1) {
                const command = rx.command_ack.peek() orelse break;
                rx.send_message(&.{ .COMMAND_ACK = .{
                    .command = command.command,
                    .result = command.result,
                } }, .{}) catch return;
                rx.command_ack.discard();
            }
        }

        {
            rx.periodic.tick();
            for (rx.periodic.messages.items()) |*item| {
                if (!item.ready) continue;

                switch (item.message_id) {
                    .HEARTBEAT => {
                        rx.send_message(&.{ .HEARTBEAT = .{
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
                    },
                    .ATTITUDE_QUATERNION => if (fusion.msg_attitude.get()) |attitude| {
                        rx.send_message(&.{ .ATTITUDE_QUATERNION = .{
                            .time_boot_ms = @truncate(hw.get_time_since_boot().to_ms()),
                            .q1 = attitude.w,
                            .q2 = attitude.x,
                            .q3 = attitude.y,
                            .q4 = attitude.z,
                            .rollspeed = 0.0,
                            .pitchspeed = 0.0,
                            .yawspeed = 0.0,
                        } }, .{}) catch return;
                    },
                    else => {},
                }

                item.ready = false;
            }
        }

        if (rx.one_shot.autopilot_version) {
            rx.send_message(&.{
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
            rx.send_message(&.{ .MISSION_COUNT = .{
                .count = 0,
                .target_system = sysid,
                .target_component = compid,
            } }, .{}) catch return;
            rx.one_shot.mission_count = false;
        }

        if (rx.send_available_modes) |*state| {
            assert(state.current_index > 0);

            while (state.current_index <= flight_modes.len) : (state.current_index += 1) {
                const mode = &flight_modes[state.current_index - 1];
                var name: [35]u8 = @splat(0);
                {
                    const src_name = mode.name orelse "";
                    std.mem.copyForwards(u8, name[0..src_name.len], src_name);
                }
                rx.send_message(&.{
                    .AVAILABLE_MODES = .{
                        .mode_name = @bitCast(name),
                        .custom_mode = 0,
                        .properties = mode.properties,
                        .number_modes = @truncate(flight_modes.len),
                        .mode_index = state.current_index,
                        .standard_mode = mode.standard,
                    },
                }, .{}) catch return;

                if (state.specific) {
                    break;
                }
            }

            rx.send_available_modes = null;
        }

        {
            // whatever we don't send right away we mark as changed again
            var changed_params = rx.param_watcher.get_and_clear();
            defer rx.param_watcher.set(changed_params);

            var it = changed_params.iterator(.{});
            while (it.next()) |index| {
                const value = parameter.get_dyn(index).?; // this can't fail since the bitset so many parameters
                const id_str = parameter.ids[index];
                const id = ParamId.from_slice(id_str) orelse {
                    log.warn("can't send param {s}: id too big", .{id_str});
                    continue;
                };
                rx.send_message(&.{
                    .PARAM_VALUE = .{
                        .param_id = id.to_raw(),
                        .param_value = switch (value) {
                            inline else => |val| std.math.lossyCast(f32, val),
                        },
                        .param_count = std.math.cast(u16, parameter.count) orelse break,
                        .param_index = std.math.cast(u16, index) orelse break,
                        .param_type = switch (value) {
                            .i8 => .MAV_PARAM_TYPE_INT8,
                            .i16 => .MAV_PARAM_TYPE_INT16,
                            .i32 => .MAV_PARAM_TYPE_INT32,
                            .i64 => .MAV_PARAM_TYPE_INT64,
                            .u8 => .MAV_PARAM_TYPE_UINT8,
                            .u16 => .MAV_PARAM_TYPE_UINT16,
                            .u32 => .MAV_PARAM_TYPE_UINT32,
                            .u64 => .MAV_PARAM_TYPE_UINT64,
                            .f32 => .MAV_PARAM_TYPE_REAL32,
                            .f64 => .MAV_PARAM_TYPE_REAL64,
                        },
                    },
                }, .{}) catch return;
                changed_params.unset(index);
            }
        }
    }

    fn send_message(rx: *RxMavlink, message: *const mavlink.OutgoingMessage, options: struct {
        seq: u8 = 0,
    }) !void {
        if (rx.txbuf < min_txbuf) {
            return error.LinkSaturated;
        }

        var buf: [mavlink.MAX_PACKET_LEN]u8 = undefined;
        const len = mavlink.serialize(
            &buf,
            message,
            .{
                .seq = options.seq,
                .sysid = sysid,
                .compid = compid,
            },
        ) catch unreachable;

        rx.rate_limiter = std.math.sub(u32, rx.rate_limiter, len) catch return error.LinkSaturated;

        try uart.write(buf[0..len]);
    }
};

pub const Periodic = struct {
    const max_messages = 10;
    const Item = struct {
        offset: u32,
        period_in_ticks: u32,
        message_id: mavlink.MessageId,
        ready: bool = false,
    };

    tick_period: time.Duration,
    count: u32 = 0,
    messages: bounded.Array(Item, max_messages) = .empty,

    pub fn add(periodic: *Periodic, message_id: mavlink.MessageId, period: time.Duration) !void {
        const period_in_ticks: u32 = @truncate(period.to_us() / periodic.tick_period.to_us());
        if (period_in_ticks == 0) {
            return error.PeriodTooSmall;
        }

        // trick to sparse the messages over the period, so they don't all
        // get sent at the same time
        const offset = periodic.count % period_in_ticks;
        periodic.count +%= 1;

        for (periodic.messages.items()) |*item| {
            if (item.message_id == message_id) {
                item.offset = offset;
                item.period_in_ticks = period_in_ticks;
                break;
            }
        } else try periodic.messages.append(.{
            .offset = offset,
            .period_in_ticks = period_in_ticks,
            .message_id = message_id,
        });
    }

    pub fn tick(periodic: *Periodic) void {
        for (periodic.messages.items()) |*item| {
            if ((periodic.count +% item.offset) % item.period_in_ticks == 0) {
                item.ready = true;
            }
        }
        periodic.count +%= 1;
    }
};

pub const ChannelMapper = struct {
    arm_switch: ChannelCondition,
    rate_mode_enable: ChannelCondition,
    angle_mode_enable: ChannelCondition,

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
        };

        msg_channels.subscribe(&mapper.rcv_channels, *ChannelMapper, mapper, channels_callback, scheduler);
    }

    fn channels_callback(mapper: *ChannelMapper, channels: Channels) void {
        const RATE_MODE_MULT = math.radians_from_degrees(50.0);
        const ANGLE_MODE_MULT = math.radians_from_degrees(50.0);

        const arm_state = mapper.arm_switch.get(&channels);
        const rate_mode = mapper.rate_mode_enable.get(&channels);
        const angle_mode = mapper.angle_mode_enable.get(&channels);

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
                        .rate_yaw = yaw * RATE_MODE_MULT,
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
