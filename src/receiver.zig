const std = @import("std");

const Command = @import("control.zig").Command;
const hw = @import("hw.zig");
const CRSF_Parser = @import("parsers/CRSF.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const Task = Scheduler.Task;

const log = std.log.scoped(.receiver);

pub var msg_channels: Message([16]u16) = .{};
pub var msg_command: Message(Command) = .{};

// TODO: make it generic over parser
pub const Rx = struct {
    parser: CRSF_Parser = .{},

    pub fn init(rx: *Rx, scheduler: *Scheduler) void {
        rx.* = .{};
        hw.UART_RX.receiver.subscribe(*Rx, rx, receive_byte, scheduler);
    }

    fn receive_byte(rx: *Rx, byte: u8) void {
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
                    log.info("received channels: {any}", .{channels});
                    msg_channels.publish(channels);
                },
                .link_statistics => |ls| {
                    log.info("received link stats: {any}", .{ls});
                },
            }
        }
    }
};

pub const ChannelsToCommand = struct {
    rcv_channels: Receiver([16]u16) = undefined,

    pub fn init(channels_to_command: *ChannelsToCommand, scheduler: *Scheduler) void {
        channels_to_command.* = .{};
        msg_channels.subscribe(&channels_to_command.rcv_channels, ChannelsToCommand, channels_to_command, channels_callback, scheduler);
    }

    fn channels_callback(channels_to_command: *ChannelsToCommand, channels: [16]u16) void {
        _ = channels_to_command; // autofix
        _ = channels; // autofix
    }
};
