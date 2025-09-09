const std = @import("std");

const CRSF_Parser = @This();

state: State = .wait_for_sync,
crc: std.hash.crc.Crc8DvbS2 = .init(),
payload_buf: [MAX_PAYLOAD_LEN]u8 = undefined,

const SYNC = 0xC8;
const MAX_PACKET_LEN = 64;

const MIN_LEN_BYTE = 2;
const MAX_LEN_BYTE = MAX_PACKET_LEN - 2;

const MAX_PAYLOAD_LEN = MAX_PACKET_LEN - 4;

pub const State = union(enum) {
    /// Wait for the sync byte.
    wait_for_sync,
    /// Read length byte.
    read_len,
    /// Read type byte.
    read_type: struct {
        payload_len: u8,
    },
    /// Read payload and checksum.
    read_payload: struct {
        len: u8,
        type: PacketType,
        index: u8,
    },
};

pub const PayloadError = error{
    InvalidPayloadLength,
    InvalidPayloadData,
    UnimplementedPacket,
};

pub const Error = error{
    InvalidChecksum,
    InvalidLengthByte,
    InvalidTypeByte,
} || PayloadError;

pub fn push_byte(parser: *CRSF_Parser, byte: u8) Error!?Packet {
    errdefer parser.state = .wait_for_sync;

    switch (parser.state) {
        .wait_for_sync => if (byte == SYNC) {
            parser.state = .read_len;
        },
        .read_len => if (byte >= MIN_LEN_BYTE and byte <= MAX_LEN_BYTE) {
            parser.state = .{ .read_type = .{ .payload_len = byte - 2 } };
        } else {
            return error.InvalidLengthByte;
        },
        .read_type => |read_type_state| {
            parser.crc.update(&.{byte});
            parser.state = .{ .read_payload = .{
                .len = read_type_state.payload_len,
                .type = std.meta.intToEnum(PacketType, byte) catch {
                    return error.InvalidTypeByte;
                },
                .index = 0,
            } };
        },
        .read_payload => |read_payload_state| if (read_payload_state.index < read_payload_state.len) {
            // std.log.debug("payload index: {} len: {}", .{read_payload_state.index, read_payload_state.len});
            parser.payload_buf[read_payload_state.index] = byte;
            parser.crc.update(&.{byte});
            parser.state = .{.read_payload = .{
                .index = read_payload_state.index + 1,
                .len = read_payload_state.len,
                .type = read_payload_state.type,
            } };
        } else {
            const checksum = parser.crc.final();
            parser.crc = .init();

            if (byte != checksum) {
                return error.InvalidChecksum;
            }

            const packet: Packet = try .parse(read_payload_state.type, parser.payload_buf[0..read_payload_state.len]);

            parser.state = .wait_for_sync;
            return packet;
        },
    }

    return null;
}

pub const PacketType = enum(u8) {
    gps = 0x02,
    vario = 0x07,
    battery_sensor = 0x08,
    baro_altitude = 0x09,
    heartbeat = 0x0b,
    link_statistics = 0x14,
    rc_channels_packed = 0x16,
    subset_rc_channels_packed = 0x17,
    link_rx_id = 0x1c,
    link_tx_id = 0x1d,
    attitude = 0x1e,
    flight_mode = 0x21,
    device_ping = 0x28,
    device_info = 0x29,
    parameter_settings_entry = 0x2b,
    parameter_read = 0x2c,
    parameter_write = 0x2d,
    elrs_status = 0x2e,
    command = 0x32,
    radio_id = 0x3a,
    kiss_request = 0x78,
    kiss_response = 0x79,
    msp_request = 0x7a,
    msp_response = 0x7b,
    msp_write = 0x7c,
    ardupilot_response = 0x80,

    // TODO: payload length function
};

pub const Packet = union(enum) {
    rc_channels_packed: [16]u11,
    link_statistics: LinkStatistics,

    pub fn parse(typ: PacketType, payload: []const u8) PayloadError!Packet {
        return switch (typ) {
            .rc_channels_packed => blk: {
                if (payload.len < 16 * @bitSizeOf(u11) / 8) return error.InvalidPayloadLength;

                var rc_channels: [16]u11 = undefined;
                inline for (&rc_channels, 0..) |*rc_channel, i| {
                    rc_channel.* = std.mem.readPackedInt(u11, payload, 11 * i, .little);
                }
                break :blk .{ .rc_channels_packed = rc_channels };
            },
            .link_statistics => blk: {
                if (payload.len < 10) return error.InvalidPayloadLength;

                break :blk .{ .link_statistics = .{
                    .uplink_rssi_1 = payload[0],
                    .uplink_rssi_2 = payload[1],
                    .uplink_link_quality = payload[2],
                    .uplink_snr = std.math.cast(i8, payload[3]) orelse return error.InvalidPayloadData,
                    .active_antenna = payload[4],
                    .rf_mode = payload[5],
                    .uplink_tx_power = std.meta.intToEnum(LinkStatistics.Uplink_TX_Power, payload[6]) catch return error.InvalidPayloadData,
                    .downlink_rssi = payload[7],
                    .downlink_link_quality = payload[8],
                    .downlink_snr = std.math.cast(i8, payload[9]) orelse return error.InvalidPayloadData,
                } };
            },
            else => return error.UnimplementedPacket,
        };
    }

    pub const LinkStatistics = struct {
        uplink_rssi_1: u8,
        uplink_rssi_2: u8,
        uplink_link_quality: u8,
        uplink_snr: i8,
        active_antenna: u8,
        rf_mode: u8,
        uplink_tx_power: Uplink_TX_Power,
        downlink_rssi: u8,
        downlink_link_quality: u8,
        downlink_snr: i8,

        pub const Uplink_TX_Power = enum(u8) {
            @"0mW" = 0,
            @"10mW",
            @"25mW",
            @"100mW",
            @"500mW",
            @"1000mW",
            @"2000mW",
            @"250mW",
            @"50mW",
        };
    };
};

fn test_parse_packet(expected: Packet, data: []const u8) !void {
    var parser: CRSF_Parser = .{};

    for (data) |byte| {
        try std.testing.expectEqual(null, parser.push_byte(byte));
    }

    try std.testing.expectEqual(
        expected,
        parser.push_byte(std.hash.crc.Crc8DvbS2.hash(data[2..])),
    );
}

test "rc_channels_packed" {
    const data: []const u8 = &.{
        SYNC, 0x18, @intFromEnum(PacketType.rc_channels_packed),
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00, 0x00, 0x00,
        0x00,
    };
    try test_parse_packet(Packet{ .rc_channels_packed = @splat(.{ .value = 0 }) }, data);
}

test "link_statistics" {
    const data: []const u8 = &.{
        SYNC, 0x0c, @intFromEnum(PacketType.link_statistics),
        100,  100,  100,
        0,    0,    2,
        1,    100,  100,
        0,
    };
    try test_parse_packet(Packet{ .link_statistics = .{
        .uplink_rssi_1 = 100,
        .uplink_rssi_2 = 100,
        .uplink_link_quality = 100,
        .uplink_snr = 0,
        .active_antenna = 0,
        .rf_mode = 2,
        .uplink_tx_power = .@"10mW",
        .downlink_rssi = 100,
        .downlink_link_quality = 100,
        .downlink_snr = 0,
    } }, data);
}
