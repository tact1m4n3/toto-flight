const std = @import("std");

const hw = @import("hw.zig");
const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const Receiver = Scheduler.Receiver;
const drivers = @import("drivers.zig");

const imu = @import("imu.zig");

const log = std.log.scoped(.persistent);

const key_len = 8;
const Key = [key_len]u8;

fn generate_key(name: []const u8) Key {
    var key: Key = @splat(0);
    std.mem.copyForwards(u8, key[0..name.len], name);
    return key;
}

fn GenerateReceiversStruct(messages: anytype) type {
    const info = @typeInfo(@TypeOf(messages)).@"struct";
    var field_types: [info.field_names.len]type = undefined;
    const field_attrs: [info.field_names.len]std.lang.Type.Struct.FieldAttributes = @splat(.{});
    for (info.field_types, &field_types) |msg_ptr_type, *rcv_type| {
        const Value = @typeInfo(msg_ptr_type).pointer.child.Value;
        if (msg_ptr_type != *Message(Value)) @compileError("`messages` should contain mutable pointers to Message");
        rcv_type.* = Receiver(Value);
    }
    return @Struct(
        .auto,
        null,
        info.field_names,
        &field_types,
        &field_attrs,
    );
}

pub fn StoreGeneric(messages: anytype) type {
    const info = @typeInfo(@TypeOf(messages)).@"struct";

    return struct {
        const Store = @This();

        const Storage = drivers.storage.StorageGeneric(hw.chip.Flash, Key, .{});
        const Receivers = GenerateReceiversStruct(messages);

        storage: Storage,
        rcvs: Receivers = undefined,

        pub fn init(store: *Store, scheduler: *Scheduler) void {
            hw.chip.flash.erase(hw.def.flash.storage_start, hw.def.flash.storage_end) catch {};

            store.* = .{
                .storage = Storage.init(
                    &hw.chip.flash,
                    hw.def.flash.storage_start,
                    hw.def.flash.storage_end,
                ) catch @panic("failed to init storage"),
            };

            store.read_all() catch |err| {
                log.warn("failed to publish config: {}", .{err});
                return;
            };

            inline for (info.field_names) |field_name| {
                @field(messages, field_name).subscribe(
                    &@field(store.rcvs, field_name),
                    *Store,
                    store,
                    struct {
                        fn callback(s: *Store, params: imu.Params) void {
                            const key = comptime generate_key(field_name);
                            s.storage.store(key, params) catch |err| {
                                log.warn("failed to store {s} params: {}", .{ field_name, err });
                            };
                        }
                    }.callback,
                    scheduler,
                );
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
