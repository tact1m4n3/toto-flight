const std = @import("std");

const hw = @import("hw.zig");
const control = @import("control.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const imu = @import("imu.zig");
const drivers = @import("drivers.zig");

const log = std.log.scoped(.persistent);

pub var msg_save: Message(void) = .{};

pub fn StoreGeneric(messages: anytype) type {
    const info = @typeInfo(@TypeOf(messages)).@"struct";

    return struct {
        const Store = @This();

        const Storage = drivers.storage.StorageGeneric(hw.Flash, Key, .{});
        const Versions = GenerateVersionsStruct(messages);

        storage: Storage,
        rcv_save: Receiver(void) = undefined,
        versions: Versions = undefined,

        pub fn init(store: *Store, scheduler: *Scheduler) void {
            // log.info("erasing", .{});
            // hw.flash.erase(hw.def.flash.storage_start, hw.def.flash.storage_end - hw.def.flash.storage_start) catch {};

            store.* = .{
                .storage = Storage.init(
                    &hw.flash,
                    hw.def.flash.storage_start,
                    hw.def.flash.storage_end,
                ) catch @panic("failed to init storage"),
            };

            store.storage.print_all_items() catch @panic("failed to print storage items");

            store.read_all() catch |err| {
                log.warn("failed to publish config: {}", .{err});
                return;
            };

            inline for (info.field_names) |field_name| {
                _, const current_version = @field(messages, field_name).get_with_version();
                @field(store.versions, field_name) = current_version;
            }

            msg_save.subscribe(&store.rcv_save, *Store, store, save_callback, scheduler);
        }

        fn save_callback(store: *Store, _: void) void {
            const arm_state = if (control.msg_status.get()) |status| status.arm else false;
            if (arm_state) {
                log.warn("skipping config save because system is armed", .{});
                return;
            }

            inline for (info.field_names) |field_name| {
                const maybe_params, const current_version = @field(messages, field_name).get_with_version();
                if (@field(store.versions, field_name) != current_version) {
                    if (maybe_params) |params| {
                        const key = comptime generate_key(field_name);
                        store.storage.store(key, params) catch |err| {
                            log.warn("failed to store {s} params: {}", .{ field_name, err });
                        };
                    }
                    @field(store.versions, field_name) = current_version;
                }
            }
        }

        fn read_all(store: *Store) !void {
            inline for (info.field_names) |field_name| {
                const key = comptime generate_key(field_name);
                if (try store.storage.fetch(key, imu.Params)) |params| {
                    @field(messages, field_name).publish(params);
                }
            }
        }
    };
}

const key_len = 8;
const Key = [key_len]u8;

fn generate_key(name: []const u8) Key {
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
