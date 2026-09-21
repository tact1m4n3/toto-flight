const builtin = @import("builtin");
const std = @import("std");
const assert = std.debug.assert;

const defs = @import("mavlink/generated.zig");
pub const MessageId = defs.MessageId;
pub const messages = defs.messages;
pub const enums = defs.enums;

const log = std.log.scoped(.mavlink);
const native_endian = builtin.cpu.arch.endian();
const CRC = std.hash.crc.@"CRC-16/MCRF4XX";

const MAGIC_BYTE_V2: u8 = 0xFD;
const EXTRA_PACKET_LEN = 12;
pub const MAX_PAYLOAD_LEN = std.math.maxInt(u8);
pub const MAX_PACKET_LEN = EXTRA_PACKET_LEN + MAX_PAYLOAD_LEN;

pub const IncomingMessage = union(enum) {
    HEARTBEAT: messages.HEARTBEAT,

    RADIO_STATUS: messages.RADIO_STATUS,
    RC_CHANNELS_OVERRIDE: messages.RC_CHANNELS_OVERRIDE,

    COMMAND_LONG: messages.COMMAND_LONG,

    MISSION_REQUEST_LIST: messages.MISSION_REQUEST_LIST,
};

pub const OutgoingMessage = union(enum) {
    HEARTBEAT: messages.HEARTBEAT,

    AUTOPILOT_VERSION: messages.AUTOPILOT_VERSION,

    COMMAND_ACK: messages.COMMAND_ACK,
    AVAILABLE_MODES: messages.AVAILABLE_MODES,

    MISSION_COUNT: messages.MISSION_COUNT,
};

pub const Parser = struct {
    state: State = .wait_for_magic,
    packet: Packet = undefined,
    crc: CRC = .init(),
    payload_buf: [MAX_PAYLOAD_LEN]u8 = undefined,

    pub const State = union(enum) {
        wait_for_magic,
        read_len,
        read_incflags,
        read_cmpflags,
        read_seq,
        read_sysid,
        read_compid,
        read_msgid: usize,
        read_payload: usize,
        read_crc: usize,
    };

    pub const Packet = struct {
        len: u8,
        incflags: u8,
        cmpflags: u8,
        seq: u8,
        sysid: u8,
        compid: u8,
        msgid: [3]u8,
        checksum: [2]u8,

        pub fn get_msgid(self: *const Packet) u24 {
            return std.mem.readInt(u24, &self.msgid, .little);
        }

        pub fn get_crc(self: *const Packet) u16 {
            return std.mem.readInt(u16, &self.checksum, .little);
        }
    };

    pub fn push_byte(p: *Parser, byte: u8) !?IncomingMessage {
        switch (p.state) {
            .wait_for_magic => if (byte == MAGIC_BYTE_V2) {
                p.crc = .init(); // reset crc
                p.state = .read_len;
            } else {
                p.state = .wait_for_magic;
            },
            .read_len => {
                p.crc.update(&.{byte});
                p.packet.len = byte;
                p.state = .read_incflags;
            },
            .read_incflags => {
                p.crc.update(&.{byte});
                p.packet.incflags = byte;
                p.state = .read_cmpflags;
            },
            .read_cmpflags => {
                p.crc.update(&.{byte});
                p.packet.cmpflags = byte;
                p.state = .read_seq;
            },
            .read_seq => {
                p.crc.update(&.{byte});
                p.packet.seq = byte;
                p.state = .read_sysid;
            },
            .read_sysid => {
                p.crc.update(&.{byte});
                p.packet.sysid = byte;
                p.state = .read_compid;
            },
            .read_compid => {
                p.crc.update(&.{byte});
                p.packet.compid = byte;
                p.state = .{ .read_msgid = 0 };
            },
            .read_msgid => |index| {
                assert(index < 3);

                p.crc.update(&.{byte});
                p.packet.msgid[index] = byte;

                if (index + 1 == 3) {
                    if (p.packet.len > 0) {
                        p.state = .{ .read_payload = 0 };
                    } else {
                        p.state = .{ .read_crc = 0 };
                    }
                } else {
                    p.state = .{ .read_msgid = index + 1 };
                }
            },
            .read_payload => |index| {
                assert(index < p.packet.len);
                p.payload_buf[index] = byte;
                p.crc.update(&.{byte});
                if (index + 1 == p.packet.len) {
                    p.state = .{ .read_crc = 0 };
                } else {
                    p.state = .{ .read_payload = index + 1 };
                }
            },
            .read_crc => |index| {
                assert(index < 2);

                if (index == 0) {
                    p.packet.checksum[0] = byte;
                    p.state = .{ .read_crc = 1 };
                } else {
                    p.packet.checksum[1] = byte;
                    p.state = .wait_for_magic;

                    if (p.packet.incflags != 0) {
                        return error.IncompatiblePacket;
                    }

                    const msgid_raw = p.packet.get_msgid();
                    const msgid = std.enums.fromInt(MessageId, msgid_raw) orelse
                        return error.UnknownMessageId;
                    p.crc.update(&.{msgid.get_crc_extra()});

                    const actual_checksum = p.crc.final();

                    const expected_checksum = p.packet.get_crc();
                    if (actual_checksum != expected_checksum) {
                        return error.InvalidChecksum;
                    }

                    const payload = p.payload_buf[0..p.packet.len];
                    const message = try parse_payload(msgid, payload);

                    return message;
                }
            },
        }
        return null;
    }
};

fn parse_payload(message_id: MessageId, payload: []const u8) !IncomingMessage {
    switch (message_id) {
        inline else => |tag| {
            if (!@hasField(IncomingMessage, @tagName(tag))) {
                return error.UnimplementedMessage;
            }
            const MessageType = @FieldType(IncomingMessage, @tagName(tag));
            const message_info = @typeInfo(MessageType).@"struct";

            var message: MessageType = undefined;

            var index: usize = 0;
            inline for (message_info.field_names, message_info.field_types) |name, FieldType| {
                const remaining = payload.len - index;
                var buffer: [@sizeOf(FieldType)]u8 = @splat(0);
                const count = @min(buffer.len, remaining);
                std.mem.copyForwards(u8, buffer[0..count], payload[index..][0..count]);
                read_type(FieldType, &@field(message, name), &buffer) catch return error.InvalidPayload;
                index += count;
            }
            return @unionInit(IncomingMessage, @tagName(tag), message);
        },
    }
}

fn read_type(comptime T: type, result: *T, buffer: *const [@sizeOf(T)]u8) !void {
    switch (@typeInfo(T)) {
        .int, .float => {
            const value: T = @bitCast(buffer.*);
            result.* = if (native_endian == .little) value else @byteSwap(value);
        },
        .@"enum" => |@"enum"| {
            var value: @"enum".tag_type = undefined;
            try read_type(@"enum".tag_type, &value, buffer);
            result.* = std.enums.fromInt(T, value) orelse return error.InvalidEnumVariant;
        },
        .@"struct" => |@"struct"| {
            if (@"struct".layout != .@"packed") {
                @compileError("only packed structs allowed in messages");
            }
            const BackingInt = @"struct".backing_integer.?;
            var value: BackingInt = undefined;
            try read_type(BackingInt, &value, buffer);
            result.* = @bitCast(value);
        },
        .array => |array| {
            for (0..array.len) |i| {
                try read_type(array.child, &result[i], buffer[i * @sizeOf(array.child) ..][0..@sizeOf(array.child)]);
            }
        },
        else => |tag| @compileError("message field type not allowed: " ++ @tagName(tag)),
    }
}

pub const SerializeOptions = struct {
    seq: u8,
    sysid: u8,
    compid: u8,
};

pub fn serialize(
    buf: []u8,
    any_message: *const OutgoingMessage,
    options: SerializeOptions,
) !usize {
    const PAYLOAD_OFFSET = 10;

    const max_among_all_payload_len: usize = comptime blk: {
        var max_payload_len = 1;
        for (@typeInfo(OutgoingMessage).@"union".field_types) |MessageType| {
            const message_info = @typeInfo(MessageType).@"struct";
            var current_max_payload_len: u8 = 0;
            for (message_info.field_names) |name| {
                current_max_payload_len += @sizeOf(@FieldType(MessageType, name));
            }
            max_payload_len = @max(max_payload_len, current_max_payload_len);
        }
        break :blk max_payload_len;
    };
    if (EXTRA_PACKET_LEN + max_among_all_payload_len > buf.len) {
        return error.BufferTooSmall;
    }

    buf[0] = MAGIC_BYTE_V2;
    // TODO: incflags, cmpflags
    buf[2] = 0;
    buf[3] = 0;
    buf[4] = options.seq;
    buf[5] = options.sysid;
    buf[6] = options.compid;

    const message_id: MessageId, const max_payload_len = switch (any_message.*) {
        inline else => |message, tag| blk: {
            const message_id = @field(MessageId, @tagName(tag));
            const MessageType = @TypeOf(message);
            const message_info = @typeInfo(MessageType).@"struct";

            var index: usize = PAYLOAD_OFFSET;
            inline for (message_info.field_names) |name| {
                var field = @field(message, name);
                if (native_endian != .little) std.mem.byteSwap(@FieldType(MessageType, name), &field);
                const raw_data: []const u8 = @ptrCast(&field);
                std.mem.copyForwards(u8, buf[index..][0..raw_data.len], raw_data);
                index += raw_data.len;
            }
            break :blk .{ message_id, index - PAYLOAD_OFFSET };
        },
    };

    std.mem.writeInt(u24, buf[7..10], @backingInt(message_id), .little);

    // compute the length dinamically based on the last non-zero byte in the payload
    const payload_len: usize = if (std.mem.findLastNone(u8, buf[PAYLOAD_OFFSET..][0..max_payload_len], &.{0})) |pos|
        pos + 1
    else
        1;
    buf[1] = @intCast(payload_len);

    var crc: CRC = .init();
    crc.update(buf[1 .. PAYLOAD_OFFSET + payload_len]);
    crc.update(&.{message_id.get_crc_extra()});
    std.mem.writeInt(u16, buf[PAYLOAD_OFFSET + payload_len ..][0..2], crc.final(), .little);

    return EXTRA_PACKET_LEN + payload_len;
}
