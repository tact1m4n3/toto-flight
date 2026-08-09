const std = @import("std");
const log = std.log.scoped(.receiver);

const hw = @import("hw.zig");
const Scheduler = @import("Scheduler.zig");
const PubSub = Scheduler.PubSub;
const Subscriber = Scheduler.Subscriber;
const Task = Scheduler.Task;
const CRSF_Parser = @import("parsers/CRSF.zig");

// TODO: make it generic over parser

pub const Rx = struct {
    uart_rx: hw.UART_RX,
    msg_channels: *PubSub([16]u16),
    parser: CRSF_Parser = .{},

    pub const Resources = struct {
        uart_rx: hw.UART_RX,
        msg_channels: *PubSub([16]u16),
    };

    pub fn init(rx: *Rx, scheduler: *Scheduler, resources: Resources) void {
        rx.* = .{
            .uart_rx = resources.uart_rx,
            .msg_channels = resources.msg_channels,
            .parser = .{},
        };

        rx.uart_rx.subscribe(*Rx, rx, receive_byte, scheduler);
    }

    pub fn receive_byte(rx: *Rx, byte: u8) void {
        const maybe_packet = rx.parser.push_byte(byte) catch |err| {
            log.warn("failed to parse packet: {t}", .{err});
            return;
        };

        if (maybe_packet) |packet| {
            switch (packet) {
                .rc_channels_packed => |raw_channels| {
                    var channels: [16]u16 = undefined;
                    for (&channels, &raw_channels) |*channel, raw_channel| {
                        channel.* = raw_channel;
                    }
                    log.info("received channels: {}", .{channels});
                    rx.msg_channels.publish(channels);
                },
                .link_statistics => |ls| {
                    log.info("received link stats: {any}", .{ls});
                },
            }
        }
    }
};

pub const ChannelsToCommand = struct {
    channels_sub: Subscriber([16]u16),
    command_msg: *PubSub(Command),
};
