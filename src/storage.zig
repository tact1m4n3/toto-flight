const std = @import("std");
const assert = std.debug.assert;

const hw = @import("hw.zig");
const control = @import("control.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const parameter = @import("parameter.zig");
const drivers = @import("drivers.zig");

const log = std.log.scoped(.storage);

pub var msg_save: Message(void) = .{};
pub var msg_load: Message(void) = .{};

pub const Storage = struct {
    const Driver = drivers.storage.StorageGeneric(hw.Flash, Key, .{});

    driver: Driver,
    rcv_save: Receiver(void) = undefined,
    param_watcher: parameter.Watcher = .{},

    pub fn init(storage: *Storage, scheduler: *Scheduler) void {
        // log.info("erasing", .{});
        // hw.flash.erase(hw.def.flash.storage_start, hw.def.flash.storage_end - hw.def.flash.storage_start) catch {};

        storage.* = .{
            .driver = Driver.init(
                hw.Flash.instance,
                hw.def.flash.storage_start,
                hw.def.flash.storage_end,
            ) catch @panic("failed to init storage"),
        };

        parameter.register_watcher(&storage.param_watcher);

        storage.load();

        msg_save.subscribe(&storage.rcv_save, *Storage, storage, save_callback, scheduler);
    }

    fn save_callback(storage: *Storage, _: void) void {
        const arm_state = if (control.msg_status.get()) |status| status.armed else false;
        if (arm_state) {
            log.warn("skipping config save because system is armed", .{});
            return;
        }

        const changed = storage.param_watcher.get_and_clear();
        if (changed != .empty) {
            log.info("saving config", .{});
            storage.driver.store(.{ .kind = .params }, parameter.dump()) catch |err| {
                log.warn("failed to save config: {}", .{err});
            };
        }
    }

    fn load(storage: *Storage) !void {
        const table = storage.driver.fetch(.{ .kind = .params }) catch |err| {
            log.warn("failed to save config: {}", .{err});
        };
        parameter.load(table);
    }
};

pub const Key = packed struct(u8) {
    kind: enum(u2) {
        params = 0,
        mission = 1,
    },
    index: u6,
};
