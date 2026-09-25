const std = @import("std");

const Scheduler = @import("Scheduler.zig");
const Message = Scheduler.Message;
const hw = @import("hw.zig");
const math = @import("math.zig");

/// Must only be accessed through a critical section.
var table: Table = .default;
/// Must only be accessed through a critical section.
var watchers: std.SinglyLinkedList = .{};

pub const ids = collect_param_ids(Table, "");
pub const count = ids.len;
pub const id_to_index: std.StaticStringMap(usize) = blk: {
    const KV = struct { []const u8, usize };
    var kvs: [ids.len]KV = undefined;
    for (&kvs, ids, 0..) |*kv, param_id, i| {
        if (param_id.len > 16) @compileLog("param id too long: ", param_id);
        kv.* = .{ param_id, i };
    }
    break :blk .initComptime(kvs);
};

pub const Table = extern struct {
    cor: extern struct {
        fwd_angl: f32,
    },
    imu: @import("imu.zig").Params,
    rate: @import("control.zig").RateParams,
    act: @import("actuator.zig").Params,

    pub const default: Table = .{
        .cor = .{
            .fwd_angl = 0.0,
        },
        .imu = .default,
        .rate = .default,
        .act = .default,
    };
};

pub const ChangedSet = std.bit_set.Static(count);
pub const Watcher = struct {
    /// Must only be accessed in through a critical section.
    changed: ChangedSet = .empty,
    node: std.SinglyLinkedList.Node = .{},

    pub fn set(watcher: *Watcher, changed: ChangedSet) void {
        const cs = hw.enter_critical_section();
        defer cs.leave();
        watcher.changed.setUnion(changed);
    }

    pub fn set_one(watcher: *Watcher, index: usize) void {
        var changed: ChangedSet = .empty;
        changed.set(index);
        watcher.set(changed);
    }

    pub fn get_and_clear(watcher: *Watcher) ChangedSet {
        const cs = hw.enter_critical_section();
        defer cs.leave();
        const changed = watcher.changed;
        watcher.changed = .empty;
        return changed;
    }

    pub fn clear(watcher: *Watcher) void {
        const cs = hw.enter_critical_section();
        defer cs.leave();
        watcher.changed = .empty;
    }
};

pub fn register_watcher(watcher: *Watcher) void {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    watcher.* = .{};
    watchers.prepend(&watcher.node);
}

/// Must be called only while on the ground (long cs).
pub fn reset_to_default() void {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    table = .default;
}

/// Must be called only while on the ground (long cs).
pub fn dump() Table {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    return table;
}

/// Must be called only while on the ground (long cs).
pub fn load(override: Table) void {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    table = override;
}

pub fn get(comptime R: type) R {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    var dst: R = undefined;
    get_params_comptime_inner(R, &dst, Table, &table);
    return dst;
}

pub fn modify(value: anytype) void {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    comptime var index: usize = 0;
    comptime var changed: ChangedSet = .empty;
    modify_params_comptime_inner(Table, &table, value, &index, &changed);
    publish_changed_bits(changed);
}

pub fn get_dyn(index: usize) ?AnyValue {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    const value_ptr = get_param(Table, &table, index) orelse return null;
    return value_ptr.get();
}

pub fn get_dyn_with_id(id: []const u8) ?AnyValue {
    const index = id_to_index.get(id) orelse return null;
    return get_dyn(index) orelse unreachable;
}

pub const DynUpdateError = error{ NotFound, TypeMismatch };

pub fn modify_dyn(index: usize, value: AnyValue) DynUpdateError!void {
    const cs = hw.enter_critical_section();
    defer cs.leave();
    const value_ptr = get_param(Table, &table, index) orelse return error.NotFound;
    try value_ptr.set(value);
    var changed: ChangedSet = .empty;
    changed.set(index);
    publish_changed_bits(changed);
}

pub fn modify_dyn_with_id(id: []const u8, value: AnyValue) DynUpdateError!void {
    const index = id_to_index.get(id) orelse return error.NotFound;
    modify_dyn(index, value) catch unreachable;
}

/// Must be called inside critical section.
fn publish_changed_bits(changed: ChangedSet) void {
    var it = watchers.first;
    while (it) |node| : (it = node.next) {
        const watcher: *Watcher = @fieldParentPtr("node", node);
        watcher.changed.setUnion(changed);
    }
}

pub const AnyValueType = enum {
    u8,
    u16,
    u32,
    u64,
    i8,
    i16,
    i32,
    i64,
    f32,
    f64,

    pub fn from_type(T: type) ?AnyValueType {
        return switch (T) {
            u8 => .u8,
            u16 => .u16,
            u32 => .u32,
            u64 => .u64,
            i8 => .i8,
            i16 => .i16,
            i32 => .i32,
            i64 => .i32,
            f32 => .f32,
            f64 => .f64,
            else => null,
        };
    }
};

pub const AnyValue = union(AnyValueType) {
    u8: u8,
    u16: u16,
    u32: u32,
    u64: u64,
    i8: i8,
    i16: i16,
    i32: i32,
    i64: i64,
    f32: f32,
    f64: f64,
};

pub const AnyValuePtr = union(AnyValueType) {
    u8: *u8,
    u16: *u16,
    u32: *u32,
    u64: *u64,
    i8: *i8,
    i16: *i16,
    i32: *i32,
    i64: *i64,
    f32: *f32,
    f64: *f64,

    fn from_ptr(ptr: anytype) ?AnyValuePtr {
        if (comptime AnyValueType.from_type(@typeInfo(@TypeOf(ptr)).pointer.child)) |kind| {
            return @unionInit(AnyValuePtr, @tagName(kind), ptr);
        } else {
            return null;
        }
    }

    fn get(any_ptr: AnyValuePtr) AnyValue {
        return switch (any_ptr) {
            inline else => |ptr, tag| return @unionInit(AnyValue, @tagName(tag), ptr.*),
        };
    }

    fn set(any_ptr: AnyValuePtr, any_value: AnyValue) error{TypeMismatch}!void {
        switch (any_ptr) {
            inline else => |ptr, tag| if (std.meta.activeTag(any_value) == tag) {
                ptr.* = @field(any_value, @tagName(tag));
            } else return error.TypeMismatch,
        }
    }
};

inline fn get_params_comptime_inner(
    comptime Dst: type,
    dst: *Dst,
    comptime Src: type,
    src: *const Src,
) void {
    if (comptime AnyValueType.from_type(Dst)) |_| {
        dst.* = src.*;
        return;
    }
    switch (@typeInfo(Dst)) {
        .@"struct" => |info| {
            inline for (info.field_names, info.field_types) |field_name, FieldType| {
                get_params_comptime_inner(FieldType, &@field(dst.*, field_name), @FieldType(Src, field_name), &@field(src.*, field_name));
            }
        },
        .array => |info| {
            inline for (0..info.len) |i| {
                get_params_comptime_inner(info.child, &dst.*[i], @typeInfo(Src).array.child, &src.*[i]);
            }
        },
        else => @compileError("invalid"),
    }
}

inline fn modify_params_comptime_inner(comptime T: type, ptr: *T, value: anytype, index: *usize, changed: anytype) void {
    if (comptime AnyValueType.from_type(T)) |_| {
        if (@TypeOf(value) != void) {
            ptr.* = value;
            comptime changed.set(index.*);
        }
        comptime index.* += 1;
        return;
    }

    switch (@typeInfo(T)) {
        .@"struct" => |info| {
            inline for (info.field_names, info.field_types) |field_name, FieldType| {
                if (field_name[0] == '_') continue;
                const next_value = if (@TypeOf(value) != void and @hasField(@TypeOf(value), field_name))
                    @field(value, field_name)
                else {};
                modify_params_comptime_inner(FieldType, &@field(ptr.*, field_name), next_value, index, changed);
            }
        },
        .array => |info| {
            inline for (0..info.len) |i| {
                const next_value = if (@TypeOf(value) != void and i < value.len)
                    value[i]
                else {};
                modify_params_comptime_inner(info.child, &ptr.*[i], next_value, index, changed);
            }
        },
        else => @compileError("invalid"),
    }
}

fn get_param(comptime T: type, ptr: *T, index: usize) ?AnyValuePtr {
    var remaining: usize = index;
    return get_param_inner(T, ptr, &remaining);
}

inline fn get_param_inner(comptime T: type, ptr: *T, remaining: *usize) ?AnyValuePtr {
    if (comptime AnyValueType.from_type(T)) |_| {
        if (remaining.* == 0) {
            return .from_ptr(ptr);
        } else {
            remaining.* -= 1;
            return null;
        }
    }

    switch (@typeInfo(T)) {
        .@"struct" => |info| {
            inline for (info.field_names, info.field_types) |field_name, FieldType| {
                if (field_name[0] == '_') continue;
                if (get_param_inner(FieldType, &@field(ptr.*, field_name), remaining)) |value| {
                    return value;
                }
            } else return null;
        },
        .array => |info| {
            for (0..info.len) |i| {
                if (get_param_inner(info.child, &ptr.*[i], remaining)) |value| {
                    return value;
                }
            } else return null;
        },
        else => return null,
    }
}

fn collect_param_ids(comptime T: type, comptime prefix: []const u8) []const []const u8 {
    if (AnyValueType.from_type(T)) |_| {
        return &.{prefix};
    }

    switch (@typeInfo(T)) {
        .@"struct" => |info| {
            var result: []const []const u8 = &.{};
            inline for (info.field_names, info.field_types) |field_name, FieldType| {
                if (field_name[0] == '_') continue;
                const new_prefix = if (prefix.len == 0) field_name else prefix ++ "." ++ field_name;
                result = result ++ collect_param_ids(FieldType, new_prefix);
            }
            return result;
        },
        .array => |info| {
            var result: []const []const u8 = &.{};
            inline for (0..info.len) |i| {
                const i_str = std.fmt.comptimePrint("{d}", .{i});
                const new_prefix = if (prefix.len == 0) i_str else prefix ++ "." ++ i_str;
                result = result ++ collect_param_ids(info.child, new_prefix);
            }
            return result;
        },
        else => {},
    }
}

const testing = std.testing;

test "comptime helpers" {
    const Params = struct {
        a: u32,
        b: f32,
        _hidden: u32 = 0,
        c: [2]struct {
            d: u8,
            e: u16,
        },
    };

    const param_ids = comptime collect_param_ids(Params, "");
    try testing.expectEqualSlices([]const u8, &.{
        "a",
        "b",
        "c.0.d",
        "c.0.e",
        "c.1.d",
        "c.1.e",
    }, param_ids);

    const id_map: std.StaticStringMap(usize) = comptime blk: {
        const KV = struct { []const u8, usize };
        var kvs: [param_ids.len]KV = undefined;
        for (&kvs, param_ids, 0..) |*kv, param_id, i| {
            kv.* = .{ param_id, i };
        }
        break :blk .initComptime(kvs);
    };

    var params: Params = .{
        .a = 42,
        .b = 3.14,
        .c = .{
            .{ .d = 1, .e = 2 },
            .{ .d = 3, .e = 4 },
        },
    };

    try testing.expectEqual(42, get_param(Params, &params, id_map.get("a").?).?.get().u32);
    try testing.expectApproxEqAbs(3.14, get_param(Params, &params, id_map.get("b").?).?.get().f32, 0.01);
    try testing.expectEqual(3, get_param(Params, &params, id_map.get("c.1.d").?).?.get().u8);
    try testing.expectEqual(4, get_param(Params, &params, id_map.get("c.1.e").?).?.get().u16);

    comptime var index: usize = 0;
    comptime var bitset: std.bit_set.Static(param_ids.len) = .empty;
    modify_params_comptime_inner(Params, &params, .{
        .a = 100,
        .c = .{
            .{},
            .{ .d = 30, .e = 40 },
        },
    }, &index, &bitset);

    try testing.expectEqual(100, get_param(Params, &params, id_map.get("a").?).?.get().u32);
    try testing.expectEqual(30, get_param(Params, &params, id_map.get("c.1.d").?).?.get().u8);
    try testing.expectEqual(40, get_param(Params, &params, id_map.get("c.1.e").?).?.get().u16);

    try testing.expect(bitset.isSet(id_map.get("a").?));
    try testing.expect(!bitset.isSet(id_map.get("b").?));
    try testing.expect(!bitset.isSet(id_map.get("c.0.d").?));
    try testing.expect(!bitset.isSet(id_map.get("c.0.e").?));
    try testing.expect(bitset.isSet(id_map.get("c.1.d").?));
    try testing.expect(bitset.isSet(id_map.get("c.1.e").?));

    var dst: struct {
        a: u32,
        c: [2]struct {
            d: u8,
        },
    } = undefined;
    get_params_comptime_inner(@TypeOf(dst), &dst, Params, &params);
    try testing.expectEqual(100, dst.a);
    try testing.expectEqual(1, dst.c[0].d);
    try testing.expectEqual(30, dst.c[1].d);
}
