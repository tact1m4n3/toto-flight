const std = @import("std");
const assert = std.debug.assert;

const defs = @import("mavlink/generated.zig");
pub const MessageId = defs.MessageId;

const log = std.log.scoped(.mavlink);

const MAX_PAYLOAD_LEN = std.math.maxInt(u8);

pub const Parser = struct {
    state: State = .wait_for_magic,
    packet: Packet = undefined,
    crc: std.hash.crc.@"CRC-16/MCRF4XX" = .init(),
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
        incompat_flags: u8,
        compat_flags: u8,
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

    pub fn push_byte(p: *Parser, byte: u8) !?Message {
        switch (p.state) {
            .wait_for_magic => if (byte == 0xFD) {
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
                p.packet.seq = byte;
                p.state = .read_cmpflags;
            },
            .read_cmpflags => {
                p.crc.update(&.{byte});
                p.packet.seq = byte;
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

                    const msgid_raw = p.packet.get_msgid();
                    const msgid = std.enums.fromInt(MessageId, msgid_raw) orelse
                        return error.UnknownMessageId;
                    p.crc.update(&.{msgid.get_crc_extra()});

                    const actual_checksum = p.crc.final();
                    p.crc = .init(); // reset crc

                    const expected_checksum = p.packet.get_crc();
                    if (actual_checksum != expected_checksum) {
                        return error.InvalidChecksum;
                    }

                    const payload = p.payload_buf[0..p.packet.len];
                    const message: Message = try .parse(msgid, payload);

                    return message;
                }
            },
        }
        return null;
    }
};

pub const Message = union(enum) {
    RADIO_STATUS: defs.messages.RADIO_STATUS,
    RC_CHANNELS_OVERRIDE: defs.messages.RC_CHANNELS_OVERRIDE,

    pub fn parse(message_id: MessageId, payload: []const u8) !Message {
        var index: usize = 0;
        switch (message_id) {
            inline else => |tag| {
                if (!@hasField(Message, @tagName(tag))) {
                    return error.UnimplementedMessage;
                }
                const MessageType = @FieldType(Message, @tagName(tag));
                const message_info = @typeInfo(MessageType).@"struct";
                var message = std.mem.zeroes(MessageType);
                inline for (message_info.field_names) |name| {
                    const remaining = payload.len - index;
                    if (remaining == 0) {
                        return @unionInit(Message, @tagName(tag), message);
                    }

                    const raw_data = std.mem.asBytes(&@field(message, name));
                    const count = @min(raw_data.len, remaining);

                    std.mem.copyForwards(u8, raw_data[0..count], payload[index..][0..count]);

                    index += count;
                }
                return @unionInit(Message, @tagName(tag), message);
            },
        }
    }
};
