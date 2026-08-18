const builtin = @import("builtin");
const std = @import("std");
const assert = std.debug.assert;
const alignForward = std.mem.alignForward;
const alignBackward = std.mem.alignBackward;

const log = std.log.scoped(.storage);

// TODO: handle write/erase errors gracefully (an erase-verify failure, a worn-out block)
// TODO: power loss simulation
// TODO: mark items as freed if newer versions are found while fetching

// Storage layout:
//
// [locked_sector] [locked_sector] [active_sector] [erased_sector] [locked_sector] [erased_or_locked_sectors...]
//
// Sector layout:
//
// [sector_tag] [item_tag] [item_header_padded] [key_padded] [value_padded] [next_items ...]
//  0x00||0xFF  0x00||0xFF
//
//  * padded to a write page
//
pub fn StorageGeneric(Flash: type, Key: type) type {
    // TODO: check key type
    return struct {
        const Storage = @This();

        const WRITE_SIZE = Flash.WRITE_SIZE;
        const ERASE_SIZE = Flash.ERASE_SIZE;

        const TAG_OFFSET = 0;
        const HEADER_OFFSET = TAG_OFFSET + WRITE_SIZE;
        const KEY_OFFSET = HEADER_OFFSET + alignForward(u16, @sizeOf(ItemHeader), WRITE_SIZE);
        const VALUE_OFFSET = KEY_OFFSET + alignForward(u16, @sizeOf(Key), WRITE_SIZE);

        flash: *Flash,
        range_start: u32,
        range_end: u32,
        current_offset: u32,
        active_sector_offset: u32,
        last_sector_offset: u32,

        pub fn init(flash: *Flash, range_start: u32, range_end: u32) !Storage {
            // TODO: verify bounds and start/size are multiples of ERASE_SIZE

            // find the first non locked page this is where we left off
            var storage: Storage = .{
                .flash = flash,
                .range_start = range_start,
                .range_end = range_end,
                .current_offset = undefined,
                .active_sector_offset = undefined,
                .last_sector_offset = undefined,
            };

            var sector_offset = range_start;
            var active_sector_offset: u32 = find_active_sector: while (sector_offset < range_end) : (sector_offset += ERASE_SIZE) {
                if (try storage.read_tag(sector_offset) == .in_use) {
                    break :find_active_sector sector_offset;
                }
            } else return error.Corrupted;

            // we got interrupted while advancing to the next sector at wraparound! ik.. one in a million
            if (active_sector_offset == range_start) {
                const prev_sector_offset = storage.get_prev_sector(active_sector_offset);
                if (try storage.read_tag(prev_sector_offset) == .in_use and !try storage.is_sector_erased(prev_sector_offset)) {
                    active_sector_offset = prev_sector_offset;
                }
            }

            storage.active_sector_offset = active_sector_offset;

            // verify the reserved sector. it must be erased!
            const reserved_sector_offset = storage.get_next_sector(active_sector_offset);
            storage.ensure_sector_erased(reserved_sector_offset) catch {
                log.warn("failed to erase reserved sector at 0x{x}", .{reserved_sector_offset});
            };

            // if the sector after the reserved one is erased, we haven't wrapped around
            const last_sector_offset = storage.get_next_sector(reserved_sector_offset);
            storage.last_sector_offset = if (try storage.is_sector_erased(last_sector_offset))
                range_start
            else
                last_sector_offset;

            var item_it: ItemIterator = .init(&storage, active_sector_offset);

            while (try item_it.next()) |item| {
                if (item_it.corrupted_data_flag) {
                    try storage.clear_tag(item.offset);
                    // catch {
                    //     log.warn("failed to free corrupted item at 0x{x}", .{item.offset});
                    // };
                }
            }
            storage.current_offset = item_it.next_offset;

            if (item_it.corrupted_header_flag) {
                try storage.advance_to_next_sector();
            }

            return storage;
        }

        pub fn fetch(storage: *Storage, key: Key, comptime T: type) !?T {
            if (try storage.fetch_item_internal(key, storage.last_sector_offset)) |item| {
                var value: T = undefined;
                try storage.flash.read(item.offset + VALUE_OFFSET, std.mem.asBytes(&value));
                return value;
            } else return null;
        }

        // TODO: add remove method

        pub fn store(storage: *Storage, key: Key, value: anytype) !void {
            // ATTENTION: item length is not aligned
            const item_len = VALUE_OFFSET + @sizeOf(@TypeOf(value));
            if (item_len > ERASE_SIZE - WRITE_SIZE) { // don't forget about the sector tag
                return error.ItemTooBig;
            }

            // TODO: check limit calculation
            var recursive_limit: u32 = (storage.range_end - storage.range_start) / ERASE_SIZE;
            while (storage.current_offset + item_len > storage.active_sector_offset + ERASE_SIZE and recursive_limit > 0) : (recursive_limit -= 1) {
                try storage.advance_to_next_sector();
            }
            if (recursive_limit == 0) return error.OutOfMemory;

            // we haven't written the new item yet.
            const maybe_existing_item = try storage.fetch_item_internal(key, storage.last_sector_offset);

            var crc: std.hash.crc.@"CRC-32/CKSUM" = .init();
            crc.update(std.mem.asBytes(&key));
            crc.update(std.mem.asBytes(&value));

            var header: ItemHeader = .init(crc.final(), item_len);

            {
                defer storage.current_offset += alignForward(u16, item_len, WRITE_SIZE);
                try storage.write_aligned(storage.current_offset + HEADER_OFFSET, std.mem.asBytes(&header));
                try storage.write_aligned(storage.current_offset + KEY_OFFSET, std.mem.asBytes(&key));
                try storage.write_aligned(storage.current_offset + VALUE_OFFSET, std.mem.asBytes(&value));
            }

            if (maybe_existing_item) |existing_item| {
                try storage.clear_tag(existing_item.offset);
            }
        }

        fn advance_to_next_sector(storage: *Storage) !void {
            // if there is only one sector for space, there is nothing we can do
            if ((storage.range_end - storage.range_start) / ERASE_SIZE == 1) {
                return error.OutOfMemory;
            }

            const active_sector_offset = storage.active_sector_offset;
            const reserved_sector_offset = storage.get_next_sector(active_sector_offset);
            const to_be_erased_sector_offset = storage.get_next_sector(reserved_sector_offset);

            var new_sector_offset: u32 = reserved_sector_offset + WRITE_SIZE;

            try storage.ensure_sector_erased(reserved_sector_offset);

            const last_sector_offset =
                if (storage.last_sector_offset == to_be_erased_sector_offset)
                    storage.get_next_sector(storage.last_sector_offset)
                else
                    storage.last_sector_offset;

            var item_it: ItemIterator = .init(storage, to_be_erased_sector_offset);
            while (try item_it.next()) |item| {
                if (item.tag == .freed) continue;

                if (try storage.fetch_item_internal(item.key, last_sector_offset) == null) {
                    var src_offset = item.offset;
                    const src_end = item_it.next_offset;

                    while (try storage.flash.read_buf(src_offset, src_end)) |data| : ({
                        src_offset += @truncate(data.len);
                        new_sector_offset += @truncate(data.len);
                    }) {
                        try storage.write_aligned(new_sector_offset, data);
                    }
                }
            }

            // lock the current sector
            try storage.clear_tag(active_sector_offset);

            // we are done with the critical parts, update state
            storage.current_offset = new_sector_offset;
            storage.active_sector_offset = reserved_sector_offset;
            storage.last_sector_offset = last_sector_offset;

            try storage.ensure_sector_erased(to_be_erased_sector_offset);
        }

        pub fn print_all_items(storage: *Storage) !void {
            var current_sector_offset = storage.active_sector_offset;
            while (true) : (current_sector_offset = storage.get_prev_sector(current_sector_offset)) {
                var item_it: ItemIterator = .init(storage, current_sector_offset);
                while (try item_it.next()) |item| {
                    std.debug.print("found item, sector={}, offset={}, tag={}, key={d}\n", .{
                        current_sector_offset,
                        item.offset,
                        item.tag,
                        item.key,
                    });
                }
                if (current_sector_offset == storage.last_sector_offset) {
                    break;
                }
            }
        }

        fn fetch_item_internal(storage: *Storage, key: Key, last_sector_offset: u32) !?Item {
            var current_sector_offset = storage.active_sector_offset;
            while (true) : (current_sector_offset = storage.get_prev_sector(current_sector_offset)) {
                var maybe_item: ?Item = null;
                var item_it: ItemIterator = .init(storage, current_sector_offset);
                while (try item_it.next()) |item| {
                    if (item.tag == .freed) continue;
                    if (item_it.corrupted_data_flag) continue;
                    // TODO: key equality based on type
                    if (key == item.key) {
                        maybe_item = item;
                    }
                }
                if (maybe_item) |item| {
                    return item;
                }
                if (current_sector_offset == last_sector_offset) {
                    return null;
                }
            }
        }

        fn get_next_sector(storage: *Storage, sector_offset: u32) u32 {
            assert(std.mem.isAligned(sector_offset, ERASE_SIZE));
            return if (sector_offset == storage.range_end - ERASE_SIZE)
                storage.range_start
            else
                sector_offset + ERASE_SIZE;
        }

        fn get_prev_sector(storage: *Storage, sector_offset: u32) u32 {
            assert(std.mem.isAligned(sector_offset, ERASE_SIZE));
            return if (sector_offset == storage.range_start)
                storage.range_end - ERASE_SIZE
            else
                sector_offset - ERASE_SIZE;
        }

        const Tag = enum {
            in_use,
            freed,

            pub const locked: Tag = .freed; // alias for sector tags
        };

        fn read_tag(storage: *Storage, offset: u32) !Tag {
            var tag: u8 = undefined;
            try storage.flash.read(offset, (&tag)[0..1]);
            return if (tag == std.math.maxInt(u8)) .in_use else .freed;
        }

        fn clear_tag(storage: *Storage, offset: u32) !void {
            const tag: [WRITE_SIZE]u8 = @splat(0);
            try storage.write_aligned(offset, &tag);
        }

        fn write_aligned(storage: *Storage, offset: u32, data: []const u8) !void {
            if (comptime WRITE_SIZE == 1) {
                try storage.flash.write(offset, data);
            } else {
                const aligned_count = alignBackward(usize, data.len, WRITE_SIZE);
                if (aligned_count > 0) {
                    try storage.flash.write(offset, data[0..aligned_count]);
                }

                if (aligned_count != data.len) {
                    var remaining_buf: [WRITE_SIZE]u8 = @splat(0xFF);
                    std.mem.copyForwards(u8, remaining_buf[0 .. data.len - aligned_count], data[aligned_count..]);
                    try storage.flash.write(offset + @as(u32, @truncate(aligned_count)), &remaining_buf);
                }
            }
        }

        fn ensure_sector_erased(storage: *Storage, offset: u32) !void {
            if (!try storage.is_sector_erased(offset)) {
                try storage.flash.erase(offset, ERASE_SIZE);
            }
        }

        fn is_sector_erased(storage: *Storage, offset: u32) !bool {
            var current_offset = offset;
            const end = current_offset + ERASE_SIZE;
            while (try storage.flash.read_buf(current_offset, end)) |data| : (current_offset += @truncate(data.len)) {
                if (!std.mem.allEqual(u8, data, 0xFF)) {
                    return false;
                }
            } else return true;
        }

        const Item = struct {
            offset: u32,
            tag: Tag,
            header: ItemHeader,
            key: Key,
        };

        const ItemIterator = struct {
            storage: *Storage,
            next_offset: u32,
            sector_end: u32,
            corrupted_header_flag: bool = false,
            corrupted_data_flag: bool = false,

            pub fn init(storage: *Storage, sector_offset: u32) ItemIterator {
                return .{
                    .storage = storage,
                    .next_offset = sector_offset + WRITE_SIZE, // skip sector tag
                    .sector_end = sector_offset + ERASE_SIZE,
                };
            }

            pub fn next(it: *ItemIterator) !?Item {
                it.corrupted_data_flag = false;

                const current_offset = it.next_offset;

                // if there is not enough space for an item to fit
                if (current_offset + VALUE_OFFSET > it.sector_end) {
                    return null;
                }

                var header: ItemHeader = undefined;
                try it.storage.flash.read(current_offset + HEADER_OFFSET, std.mem.asBytes(&header));

                if (header.is_uninitialized()) {
                    // we reached the end of the sector, we can continue from here
                    return null;
                }

                if (!header.is_valid()) {
                    it.corrupted_header_flag = true;
                    return null;
                }

                const next_offset = current_offset + alignForward(u32, header.len, WRITE_SIZE);
                if (next_offset > it.sector_end) {
                    it.corrupted_header_flag = true;
                    return null;
                }

                const tag = try it.storage.read_tag(current_offset);
                if (tag == .in_use) {
                    var crc: std.hash.crc.@"CRC-32/CKSUM" = .init();
                    var check_offset = current_offset + KEY_OFFSET;
                    const key_end = check_offset + @sizeOf(Key);
                    while (try it.storage.flash.read_buf(check_offset, key_end)) |data| : (check_offset += @truncate(data.len)) {
                        crc.update(data);
                    }
                    check_offset = current_offset + VALUE_OFFSET;
                    const value_end = current_offset + header.len;
                    while (try it.storage.flash.read_buf(check_offset, value_end)) |data| : (check_offset += @truncate(data.len)) {
                        crc.update(data);
                    }
                    if (header.data_crc != crc.final()) {
                        it.corrupted_data_flag = true;
                    }
                }

                var key: Key = undefined;
                try it.storage.flash.read(current_offset + KEY_OFFSET, std.mem.asBytes(&key));

                it.next_offset = next_offset;

                return .{
                    .offset = current_offset,
                    .tag = tag,
                    .header = header,
                    .key = key,
                };
            }
        };
    };
}

const MAX_KEY_LEN = std.math.maxInt(u4);

const ItemHeader = packed struct(u64) {
    data_crc: u32,
    len: u16,
    len_flipped: u16,

    pub fn init(data_crc: u32, len: u16) ItemHeader {
        return .{
            .data_crc = data_crc,
            .len = len,
            .len_flipped = ~len,
        };
    }

    pub fn is_uninitialized(self: ItemHeader) bool {
        return @as(u64, @bitCast(self)) == std.math.maxInt(u64);
    }

    pub fn is_valid(self: ItemHeader) bool {
        return @as(u16, @bitCast(self.len)) == ~self.len_flipped;
    }
};

pub const MockFlashOptions = struct {
    write_size: u32,
    erase_size: u32,
};

pub fn MockFlash(options: MockFlashOptions) type {
    return struct {
        const Self = @This();

        pub const WRITE_SIZE = options.write_size;
        pub const ERASE_SIZE = options.erase_size;

        data: []u8,
        page_programmed_bitset: std.bit_set.Dynamic,

        schedule_power_loss_in_n_bytes: u32 = 0xFFFFFFFF,

        pub fn init(gpa: std.mem.Allocator, size: usize) !Self {
            if (size % options.write_size != 0 or size % options.erase_size != 0) {
                return error.InvalidSize;
            }

            const buf: []u8 = try gpa.alloc(u8, size);
            errdefer gpa.free(buf);
            @memset(buf, 0xFF);

            var page_programmed_bitset: std.bit_set.Dynamic = try .initEmpty(gpa, size / options.write_size);
            errdefer page_programmed_bitset.deinit(gpa);

            return .{
                .data = buf,
                .page_programmed_bitset = page_programmed_bitset,
            };
        }

        pub fn deinit(self: *Self, gpa: std.mem.Allocator) void {
            gpa.free(self.data);
            self.page_programmed_bitset.deinit(gpa);
        }

        pub fn erase(self: *Self, offset: u32, size: u32) !void {
            @memset(self.data[offset..][0..size], 0xFF);
            self.page_programmed_bitset.unsetAll();
        }

        pub fn read_buf(self: *Self, offset: u32, end: u32) !?[]const u8 {
            if (offset < end) {
                return self.data[offset..end];
            } else {
                return null;
            }
        }

        pub fn read(self: *Self, offset: u32, data: []u8) !void {
            std.mem.copyForwards(u8, data, self.data[offset..][0..data.len]);
        }

        pub fn write(self: *Self, offset: u32, data: []const u8) !void {
            // return errors before writing anything to the buffer
            for (offset / WRITE_SIZE..(offset + data.len) / WRITE_SIZE) |page_id| {
                if (self.page_programmed_bitset.isSet(page_id)) {
                    return error.PageAlreadyProgrammed;
                }
            }
            var current_offset = offset;
            while (current_offset < data.len) : (current_offset += WRITE_SIZE) {
                self.page_programmed_bitset.set(current_offset / WRITE_SIZE);
            }
            std.mem.copyForwards(u8, self.data[offset..][0..data.len], data);
        }
    };
}

const testing = std.testing;

comptime {
    for (&.{ 1, 2, 4, 16, 32 }) |write_size| {
        _ = GenerateTests(.{ .write_size = write_size, .erase_size = 1024 });
    }
}

pub fn GenerateTests(comptime flash_options: MockFlashOptions) type {
    return struct {
        const TestFlash = MockFlash(flash_options);
        const TestStorage = StorageGeneric(TestFlash, u32);
        const FLASH_SIZE = 4 * 1024; // 4 sectors

        test "store then fetch roundtrip" {
            var flash: TestFlash = try .init(testing.allocator, FLASH_SIZE);
            defer flash.deinit(testing.allocator);
            var s: TestStorage = try .init(&flash, 0, FLASH_SIZE);

            try s.store(1, @as(u32, 111));
            try s.store(2, @as(u32, 222));
            try testing.expectEqual(@as(?u32, 111), try s.fetch(1, u32));
            try testing.expectEqual(@as(?u32, 222), try s.fetch(2, u32));
            try testing.expectEqual(@as(?u32, null), try s.fetch(3, u32));
        }

        test "overwrite frees the old item" {
            var flash: TestFlash = try .init(testing.allocator, FLASH_SIZE);
            defer flash.deinit(testing.allocator);
            var s: TestStorage = try .init(&flash, 0, FLASH_SIZE);

            try s.store(1, @as(u32, 111));
            try s.store(1, @as(u32, 999));
            try testing.expectEqual(@as(?u32, 999), try s.fetch(1, u32));
        }

        test "sector wraparound and reinit" {
            var flash: TestFlash = try .init(testing.allocator, FLASH_SIZE);
            defer flash.deinit(testing.allocator);

            {
                var s: TestStorage = try .init(&flash, 0, FLASH_SIZE);
                // write enough small items to force several sector rotations
                var i: u32 = 0;
                while (i < 100) : (i += 1) {
                    try s.store(i % 20, i);
                }
            }

            {
                // reinit from the same buffer, as if we just power-cycled cleanly
                var s = try TestStorage.init(&flash, 0, FLASH_SIZE);
                var k: u32 = 0;
                while (k < 20) : (k += 1) {
                    try testing.expect((try s.fetch(k, u32)) != null);
                }
            }
        }

        test "item too big is rejected" {
            var flash: TestFlash = try .init(testing.allocator, FLASH_SIZE);
            defer flash.deinit(testing.allocator);
            var s: TestStorage = try .init(&flash, 0, FLASH_SIZE);

            const big: [2000]u8 = undefined;
            try testing.expectError(error.ItemTooBig, s.store(1, big));
        }

        test "single sector range cannot advance" {
            var flash: TestFlash = try .init(testing.allocator, FLASH_SIZE);
            defer flash.deinit(testing.allocator);
            var s: TestStorage = try .init(&flash, 0, 1024);

            // fill the single sector until it can't fit another item
            var i: u32 = 0;
            while (true) : (i += 1) {
                s.store(i, i) catch |err| {
                    try testing.expectEqual(error.OutOfMemory, err);
                    break;
                };
                if (i > 1000) return error.NeverRanOutOfMemory;
            }
        }

        test "normal out of memory error" {
            var flash: TestFlash = try .init(testing.allocator, FLASH_SIZE);
            defer flash.deinit(testing.allocator);

            var s: TestStorage = try .init(&flash, 0, FLASH_SIZE);
            // write enough small items to force several sector rotations
            var i: u32 = 0;
            while (true) : (i += 1) {
                s.store(i, i) catch |err| {
                    try testing.expectEqual(error.OutOfMemory, err);
                    break;
                };
                if (i > 1000) return error.NeverRanOutOfMemory;
            }
        }
    };
}

// test "storage.basic" {
//     try testing.fuzz(void, fuzz_storage, .{});
// }
//
// fn fuzz_storage(_: void, smith: testing.Smith) void {
//     const WriteSize = enum(u32) {
//         @"1" = 1,
//         @"2" = 2,
//         @"4" = 4,
//         @"16" = 16,
//         @"32" = 32,
//     };
//
//     const TestFlash = MockFlash(.{
//         .write_size = @intFromEnum(smith.value(WriteSize)),
//         .erase_size = 4096,
//     });
//
//     const flash_size = 24 * 1024;
//
//     const buf: []u8 = try testing.allocator.alloc(u8, flash_size);
//     defer testing.allocator.free(buf);
//     const flash: TestFlash = try .init(buf);
//     var storage: StorageGeneric(TestFlash, u32) = try .init(flash, 0, flash_size);
//
//     const Action = union(enum) {
//         fetch: struct {
//             key: u32,
//         },
//         store: struct {
//             key: u32,
//             value: u32,
//         },
//     };
// }

// pub fn write(self: *Self, offset: u32, value: anytype, endian: std.lang.Endian) !void {
//     switch (@typeInfo(@TypeOf(value))) {
//         .@"struct" => |info| switch (info.layout) {
//             .auto => @compileError("struct must be extern or packed"),
//             .@"extern" => {
//                 if (native_endian == endian) {
//                     try self.write_raw(
//                         offset,
//                         @ptrCast((&value)[0..1]),
//                     );
//                 } else {
//                     var copy = value;
//                     std.mem.byteSwapAllFields(@TypeOf(value), &copy);
//                     try self.write_raw(
//                         offset,
//                         @ptrCast((&copy)[0..1]),
//                     );
//                 }
//             },
//             .@"packed" => {
//                 const BackingInt = info.backing_integer.?;
//                 var header_buf: [@sizeOf(BackingInt)]u8 = undefined;
//                 std.mem.writeInt(BackingInt, &header_buf, @bitCast(value), endian);
//                 try self.write_raw(
//                     offset,
//                     &header_buf,
//                 );
//             },
//         },
//         // .@"union" => |info| switch (info.layout) {
//         //     .auto => {
//         //         const Tag = info.tag_type.?;
//         //         switch (value) {
//         //             inline else => |inner, tag| {
//         //                 var tag_buf: [@sizeOf(Tag)]u8 = undefined;
//         //                 std.mem.writeInt(Tag, &tag_buf, @bitCast(tag), endian);
//         //                 try self.write_raw(
//         //                     offset,
//         //                     &tag_buf,
//         //                 );
//         //                 try self.write(
//         //                     offset + @sizeOf(Tag),
//         //                     inner,
//         //                     endian,
//         //                 );
//         //             },
//         //         }
//         //     },
//         //     else => @compileError("union must be auto"),
//         // },
//         else => @compileError("unexpected type " ++ @typeName(@TypeOf(value))),
//     }
// }

// pub fn read(comptime T: type, offset: u32, endian: std.lang.Endian) T {
//     switch (@typeInfo(T)) {
//         .@"struct" => |info| switch (info.layout) {
//             .auto => @compileError("struct must be extern or packed"),
//             .@"extern" => {
//                 if (native_endian == endian) {
//                     return @as([]T, @ptrCast(flash.data[offset..][0..@sizeOf(T)]))[0];
//                 } else {
//                     var value: T = undefined;
//                     std.mem.readInt(T, @as([]T, @ptrCast(flash.data[offset..][0..@sizeOf(T)]))[0], &value, endian);
//                     std.mem.byteSwapAllFields(T, &value);
//                     return value;
//                 }
//             },
//             .@"packed" => {
//                 const BackingInt = info.backing_integer.?;
//                 var value: T = undefined;
//                 std.mem.readInt(
//                     BackingInt,
//                     @as([]BackingInt, @ptrCast(flash.data[offset..][0..@sizeOf(BackingInt)]))[0],
//                     &value,
//                     endian,
//                 );
//                 return value;
//             },
//         },
//         else => @compileError("expected struct"),
//     }
// }
