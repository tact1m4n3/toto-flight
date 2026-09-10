const std = @import("std");
const assert = std.debug.assert;

const hw = @import("hw.zig");
const control = @import("control.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const ParamTable = Scheduler.ParamTable;
const drivers = @import("drivers.zig");

const log = std.log.scoped(.storage);

pub var msg_save: Message(void) = .{};

pub fn StorageGeneric(tables: anytype) type {
    const info = @typeInfo(@TypeOf(tables)).@"struct";

    return struct {
        const Self = @This();

        const Driver = drivers.storage.StorageGeneric(hw.Flash, Key, .{});
        const Versions = GenerateVersionsStruct(tables);

        driver: Driver,
        rcv_save: Receiver(void) = undefined,
        versions: Versions = undefined,

        pub fn init(storage: *Self, scheduler: *Scheduler) void {
            // log.info("erasing", .{});
            // hw.flash.erase(hw.def.flash.storage_start, hw.def.flash.storage_end - hw.def.flash.storage_start) catch {};

            storage.* = .{
                .driver = Driver.init(
                    hw.flash,
                    hw.def.flash.storage_start,
                    hw.def.flash.storage_end,
                ) catch @panic("failed to init storage"),
            };

            storage.read_all() catch |err| {
                log.warn("failed to publish config: {}", .{err});
                return;
            };

            inline for (info.field_names) |field_name| {
                _, const version = @field(tables, field_name).get_with_version();
                @field(storage.versions, field_name) = version;
            }

            msg_save.subscribe(&storage.rcv_save, *Self, storage, save_callback, scheduler);
        }

        fn save_callback(store: *Self, _: void) void {
            const arm_state = if (control.msg_status.get()) |status| status.arm else false;
            if (arm_state) {
                log.warn("skipping config save because system is armed", .{});
                return;
            }

            inline for (info.field_names) |field_name| {
                const maybe_params, const version = @field(tables, field_name).get_with_version();
                if (@field(store.versions, field_name) != version) {
                    if (maybe_params) |params| {
                        std.log.info("storing something", .{});
                        const key = comptime generate_key(field_name);
                        store.driver.store(key, params) catch |err| {
                            log.warn("failed to store {s} params: {}", .{ field_name, err });
                        };
                        @field(store.versions, field_name) = version;
                    }
                }
            }
        }

        fn read_all(storage: *Self) !void {
            inline for (info.field_names) |field_name| {
                const key = comptime generate_key(field_name);
                const Table = @TypeOf(@field(tables, field_name));
                const ParamsType = @typeInfo(Table).pointer.child.Type;
                if (try storage.driver.fetch(key, ParamsType)) |params| {
                    @field(tables, field_name).update(params);
                }
            }
        }
    };
}

const key_len = 8;
const Key = [key_len]u8;

fn generate_key(comptime name: []const u8) Key {
    comptime assert(name.len <= key_len);
    var key: Key = @splat(0);
    std.mem.copyForwards(u8, key[0..name.len], name);
    return key;
}

fn GenerateVersionsStruct(messages: anytype) type {
    const info = @typeInfo(@TypeOf(messages)).@"struct";
    const field_types: [info.field_names.len]type = @splat(u32);
    const field_attrs: [info.field_names.len]std.lang.Type.Struct.FieldAttributes = @splat(.{});
    return @Struct(
        .auto,
        null,
        info.field_names,
        &field_types,
        &field_attrs,
    );
}
