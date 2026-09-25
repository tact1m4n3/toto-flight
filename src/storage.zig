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

pub var arm_block: control.ArmBlock = .init;
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

        if (storage.driver.fetch(.params, parameter.Table)) |maybe_table| {
            if (maybe_table) |table| {
                parameter.load(table);
            }
        } else |err| {
            log.warn("failed to load parameters: {}", .{err});
        }

        msg_save.subscribe(&storage.rcv_save, *Storage, storage, save_callback, scheduler);
    }

    fn save_callback(storage: *Storage, _: void) void {
        arm_block.acquire() catch {
            log.warn("skipped parameter save... maybe armed", .{});
            return;
        };
        defer arm_block.release();

        const changed = storage.param_watcher.get_and_clear();
        if (changed.count() == 0) {
            return;
        }

        log.info("saving config", .{});
        storage.driver.store(.params, parameter.dump()) catch |err| {
            log.warn("failed to save config: {}", .{err});
        };
    }

    fn load_callback(storage: *Storage, _: void) void {
        arm_block.acquire() orelse {
            log.warn("skipped parameter load... maybe armed", .{});
            return;
        };
        defer arm_block.release();

        const maybe_table = storage.driver.fetch(.params, parameter.Table) catch |err| {
            log.warn("failed to load parameters: {}", .{err});
            return;
        };
        if (maybe_table) |table| {
            parameter.load(table);
        }
    }
};

pub const Key = enum(u8) {
    params = 0,
    mission = 1,
};
