const builtin = @import("builtin");
const std = @import("std");
const assert = std.debug.assert;
const alignForward = std.mem.alignForward;
const alignBackward = std.mem.alignBackward;
const isAligned = std.mem.isAlignedGeneric;

const log = std.log.scoped(.storage);

fn is_type_allowed(T: type) bool {
    return switch (@typeInfo(T)) {
        .@"struct" => |info| info.layout == .@"extern" or info.layout == .@"packed",
        .@"enum" => |info| is_type_allowed(info.tag_type),
        .int => |info| info.bits % 8 == 0,
        .array => |info| is_type_allowed(info.child),
        else => false,
    };
}

pub const StorageGenericOptions = struct {
    max_write_attempts: usize = 2,
    max_erase_attempts: usize = 2,
    max_read_attempts: usize = 2,
};

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
pub fn StorageGeneric(Flash: type, Key: type, options: StorageGenericOptions) type {
    if (!is_type_allowed(Key)) @compileError("Invalid key type " ++ @typeName(Key));

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
            if (range_start >= range_end) {
                return error.InvalidRange;
            }
            if (!isAligned(u32, range_start, ERASE_SIZE)) {
                return error.InvalidRange;
            }
            if (!isAligned(u32, range_end, ERASE_SIZE)) {
                return error.InvalidRange;
            }

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
            var active_sector_offset: u32 = find_sector: while (sector_offset < range_end) : (sector_offset += ERASE_SIZE) {
                const tag = storage.read_tag(sector_offset) catch |err| switch (err) {
                    error.Corrupted => Tag.locked,
                    else => return err,
                };
                if (tag == .in_use) {
                    break :find_sector sector_offset;
                }
            } else return error.Corrupted;

            // we got interrupted while advancing to the next sector at wraparound! ik.. one in a million
            if (active_sector_offset == range_start) {
                const prev_sector_offset = storage.get_prev_sector(active_sector_offset);
                const prev_sector_tag = storage.read_tag(prev_sector_offset) catch |err| switch (err) {
                    error.Corrupted => Tag.locked,
                    else => return err,
                };
                if (prev_sector_tag == .in_use and !try storage.is_sector_erased(prev_sector_offset)) {
                    active_sector_offset = prev_sector_offset;
                }
            }

            storage.active_sector_offset = active_sector_offset;

            // verify the reserved sector. it must be erased!
            const reserved_sector_offset = storage.get_next_sector(active_sector_offset);
            storage.erase_sector_retrying(reserved_sector_offset) catch |err| {
                log.warn("failed to erase reserved sector at 0x{X}: {}", .{ reserved_sector_offset, err });
            };

            // if the sector after the reserved one is erased, we haven't wrapped around
            const last_sector_offset = storage.get_next_sector(reserved_sector_offset);
            storage.last_sector_offset = if (try storage.is_sector_erased(last_sector_offset))
                range_start
            else
                last_sector_offset;

            var item_it: ItemIterator = .init(&storage, active_sector_offset);

            while (try item_it.next()) |item| {
                if (item.tag == .freed) continue;
                if (item.key == .corrupted) {
                    storage.clear_tag(item.offset) catch |err| {
                        log.warn("failed to free corrupted item at 0x{X}: {}", .{ item.offset, err });
                    };
                }
            }
            storage.current_offset = item_it.next_offset;

            if (item_it.corrupted_header_flag) {
                try storage.advance_to_next_sector();
            }

            return storage;
        }

        pub fn fetch(storage: *Storage, key: Key, comptime T: type) !?T {
            if (comptime !is_type_allowed(T)) @compileError("Invalid value type " ++ @typeName(T));

            if (try storage.fetch_item_internal(key, storage.last_sector_offset)) |item| {
                var value: T = undefined;
                try storage.read_retrying(item.offset + VALUE_OFFSET, std.mem.asBytes(&value));
                return value;
            } else return null;
        }

        pub fn store(storage: *Storage, key: Key, value: anytype) !void {
            if (comptime !is_type_allowed(@TypeOf(value))) @compileError("Invalid value type " ++ @typeName(@TypeOf(value)));

            const item_len: u16 = VALUE_OFFSET + @sizeOf(@TypeOf(value));
            const item_stride = alignForward(u16, item_len, WRITE_SIZE);
            if (item_stride > ERASE_SIZE - WRITE_SIZE) { // don't forget the sector tag
                return error.ItemTooBig;
            }

            var recursive_limit: u32 = (storage.range_end - storage.range_start) / ERASE_SIZE;
            to_next_sector: while (recursive_limit > 0) : ({
                try storage.advance_to_next_sector();
                recursive_limit -= 1;
            }) {
                to_next_slot: while (storage.current_offset + item_stride <= storage.active_sector_offset + ERASE_SIZE) : ({
                    storage.current_offset += item_stride;
                }) {
                    var crc: std.hash.crc.@"CRC-32/CKSUM" = .init();
                    crc.update(std.mem.asBytes(&key));
                    crc.update(std.mem.asBytes(&value));

                    var header: ItemHeader = .init(crc.final(), item_len);

                    storage.write_retrying_aligned(storage.current_offset + HEADER_OFFSET, std.mem.asBytes(&header)) catch |err| switch (err) {
                        error.PageAlreadyProgrammed => continue :to_next_sector,
                        else => return err,
                    };
                    storage.write_retrying_aligned(storage.current_offset + KEY_OFFSET, std.mem.asBytes(&key)) catch |err| switch (err) {
                        error.PageAlreadyProgrammed => continue :to_next_slot,
                        else => return err,
                    };
                    storage.write_retrying_aligned(storage.current_offset + VALUE_OFFSET, std.mem.asBytes(&value)) catch |err| switch (err) {
                        error.PageAlreadyProgrammed => continue :to_next_slot,
                        else => return err,
                    };

                    errdefer comptime unreachable; // we have written the item successfully

                    // because we call it before updating current_offset, we
                    // skip the last added item
                    storage.try_to_mark_all_items_free(key) catch {};

                    storage.current_offset += item_stride;

                    return;
                } else continue :to_next_sector;
            } else return error.OutOfMemory;
        }

        fn try_to_mark_all_items_free(storage: *Storage, key: Key) !void {
            var current_sector_offset = storage.active_sector_offset;
            while (true) : (current_sector_offset = storage.get_prev_sector(current_sector_offset)) {
                var item_it: ItemIterator = .init(storage, current_sector_offset);
                while (try item_it.next()) |item| {
                    if (item.tag == .freed) continue;
                    switch (item.key) {
                        .ok => |item_key| {
                            // TODO: key equality based on type
                            if (key == item_key) {
                                storage.clear_tag(item.offset) catch |err| {
                                    log.warn("failed to mark old item as free at 0x{X}: {}", .{ item.offset, err });
                                };
                            }
                        },
                        .corrupted => continue,
                    }
                }
                if (current_sector_offset == storage.last_sector_offset) {
                    break;
                }
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

            try storage.erase_sector_retrying(reserved_sector_offset);

            const last_sector_offset =
                if (storage.last_sector_offset == to_be_erased_sector_offset)
                    storage.get_next_sector(storage.last_sector_offset)
                else
                    storage.last_sector_offset;

            var item_it: ItemIterator = .init(storage, to_be_erased_sector_offset);
            while (try item_it.next()) |item| {
                if (item.tag == .freed) continue;
                switch (item.key) {
                    .ok => |item_key| {
                        if (try storage.fetch_item_internal(item_key, last_sector_offset) == null) {
                            var src_offset = item.offset + HEADER_OFFSET;
                            const src_end = item_it.next_offset;
                            const item_new_start = new_sector_offset;
                            var dst_offset = item_new_start + HEADER_OFFSET;

                            while (try storage.read_buf_retrying(src_offset, src_end)) |data| : ({
                                src_offset += @truncate(data.len);
                                dst_offset += @truncate(data.len);
                            }) {
                                // can't fail with error.PageAlreadyProgrammed
                                // because we are writing to a fresh sector
                                try storage.write_retrying_aligned(dst_offset, data);
                            }

                            new_sector_offset = item_new_start + (src_end - item.offset);
                        }
                    },
                    .corrupted => continue,
                }
            }

            // lock the current sector
            try storage.clear_tag(active_sector_offset);

            // we are done with the critical parts, update state
            storage.current_offset = new_sector_offset;
            storage.active_sector_offset = reserved_sector_offset;
            storage.last_sector_offset = last_sector_offset;

            errdefer comptime unreachable;

            storage.erase_sector_retrying(to_be_erased_sector_offset) catch |err| {
                log.warn("failed to erase the new reserved sector at 0x{X}: {}", .{ to_be_erased_sector_offset, err });
            };
        }

        fn fetch_item_internal(storage: *Storage, key: Key, last_sector_offset: u32) !?Item {
            var current_sector_offset = storage.active_sector_offset;
            while (true) : (current_sector_offset = storage.get_prev_sector(current_sector_offset)) {
                var item_it: ItemIterator = .init(storage, current_sector_offset);
                var maybe_item: ?Item = null;
                while (try item_it.next()) |item| {
                    if (item.tag == .freed) continue;
                    switch (item.key) {
                        .ok => |item_key| {
                            // TODO: key equality based on type
                            if (key == item_key) {
                                maybe_item = item;
                            }
                        },
                        .corrupted => continue,
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

        pub fn print_all_items(storage: *Storage) !void {
            var current_sector_offset = storage.active_sector_offset;
            while (true) : (current_sector_offset = storage.get_prev_sector(current_sector_offset)) {
                var item_it: ItemIterator = .init(storage, current_sector_offset);
                while (try item_it.next()) |item| {
                    std.debug.print("found item, sector={}, offset={}, tag={}, key={}\n", .{
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

        fn get_next_sector(storage: *Storage, sector_offset: u32) u32 {
            assert(isAligned(u32, sector_offset, ERASE_SIZE));
            return if (sector_offset == storage.range_end - ERASE_SIZE)
                storage.range_start
            else
                sector_offset + ERASE_SIZE;
        }

        fn get_prev_sector(storage: *Storage, sector_offset: u32) u32 {
            assert(isAligned(u32, sector_offset, ERASE_SIZE));
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
            try storage.read_retrying(offset, (&tag)[0..1]);
            return if (tag == std.math.maxInt(u8)) .in_use else .freed;
        }

        fn clear_tag(storage: *Storage, offset: u32) !void {
            const tag: [WRITE_SIZE]u8 = @splat(0);
            try storage.write_retrying_aligned(offset, &tag);
        }

        fn read_retrying(storage: *Storage, offset: u32, data: []u8) !void {
            var attempts_remaining: usize = options.max_read_attempts;
            while (attempts_remaining > 0) : (attempts_remaining -= 1) {
                return storage.flash.read(offset, data) catch |err| switch (err) {
                    error.Corrupted => return error.Corrupted,
                    else => continue,
                };
            } else return error.ReadFailed;
        }

        fn read_buf_retrying(storage: *Storage, offset: u32, end: u32) !?[]const u8 {
            var attempts_remaining: usize = options.max_read_attempts;
            while (attempts_remaining > 0) : (attempts_remaining -= 1) {
                return storage.flash.read_buf(offset, end) catch |err| switch (err) {
                    error.Corrupted => return error.Corrupted,
                    else => continue,
                };
            } else return error.ReadFailed;
        }

        fn write_retrying_aligned(storage: *Storage, offset: u32, data: []const u8) !void {
            if (comptime WRITE_SIZE == 1) {
                try storage.write_retrying(offset, data);
            } else {
                const aligned_count = alignBackward(usize, data.len, WRITE_SIZE);
                if (aligned_count > 0) {
                    try storage.write_retrying(offset, data[0..aligned_count]);
                }

                if (aligned_count != data.len) {
                    var remaining_buf: [WRITE_SIZE]u8 = @splat(0xFF);
                    std.mem.copyForwards(u8, remaining_buf[0 .. data.len - aligned_count], data[aligned_count..]);
                    try storage.write_retrying(offset + @as(u32, @truncate(aligned_count)), &remaining_buf);
                }
            }
        }

        fn write_retrying(storage: *Storage, offset: u32, data: []const u8) !void {
            var attempts_remaining: usize = options.max_write_attempts;
            attempts: while (attempts_remaining > 0) : (attempts_remaining -= 1) {
                storage.flash.write(offset, data) catch |err| switch (err) {
                    error.PageAlreadyProgrammed => return error.PageAlreadyProgrammed,
                    else => continue,
                };

                // verify what was written
                var current_offset = offset;
                var data_offset: usize = 0;
                const end = current_offset + @as(u32, @truncate(data.len));
                while (try storage.read_buf_retrying(current_offset, end)) |flash_data| : ({
                    current_offset += @truncate(flash_data.len);
                    data_offset += flash_data.len;
                }) {
                    if (!std.mem.eql(u8, flash_data, data[data_offset..][0..flash_data.len])) {
                        continue :attempts;
                    }
                } else break :attempts;
            } else return error.WriteFailed;
        }

        fn erase_sector_retrying(storage: *Storage, offset: u32) !void {
            if (try storage.is_sector_erased(offset)) {
                return;
            }
            var attempts_remaining: usize = options.max_erase_attempts;
            attempts: while (attempts_remaining > 0) : (attempts_remaining -= 1) {
                storage.flash.erase(offset, ERASE_SIZE) catch continue :attempts;
                if (try storage.is_sector_erased(offset)) {
                    break :attempts;
                } else {
                    continue :attempts;
                }
            } else return error.EraseFailed;
        }

        fn is_sector_erased(storage: *Storage, offset: u32) !bool {
            var current_offset = offset;
            const end = current_offset + ERASE_SIZE;
            while (storage.read_buf_retrying(current_offset, end) catch |err| switch (err) {
                error.Corrupted => return false,
                else => return err,
            }) |data| : (current_offset += @truncate(data.len)) {
                if (!std.mem.allEqual(u8, data, 0xFF)) {
                    return false;
                }
            } else return true;
        }

        const Item = struct {
            offset: u32,
            tag: Tag,
            header: ItemHeader,
            key: union(enum) {
                ok: Key,
                corrupted,

                pub fn format(self: *const @This(), writer: *std.Io.Writer) std.Io.Writer.Error!void {
                    switch (self.*) {
                        .ok => |key| try writer.print("{f}", .{key}),
                        .corrupted => try writer.writeAll("corrupted"),
                    }
                }
            },
        };

        const ItemIterator = struct {
            storage: *Storage,
            next_offset: u32,
            end_offset: u32,
            corrupted_header_flag: bool = false,

            pub fn init(storage: *Storage, sector_offset: u32) ItemIterator {
                return .{
                    .storage = storage,
                    .next_offset = sector_offset + WRITE_SIZE, // don't forget the sector tag
                    .end_offset = if (sector_offset == storage.active_sector_offset)
                        storage.current_offset
                    else
                        sector_offset + ERASE_SIZE,
                };
            }

            pub fn next(it: *ItemIterator) !?Item {
                const current_offset = it.next_offset;

                // if there is not enough space for an item to fit
                if (current_offset + VALUE_OFFSET > it.end_offset) {
                    return null;
                }

                var header: ItemHeader = undefined;
                it.storage.read_retrying(current_offset + HEADER_OFFSET, std.mem.asBytes(&header)) catch |err| switch (err) {
                    error.Corrupted => {
                        it.corrupted_header_flag = true;
                        return null;
                    },
                    else => return err,
                };

                if (header.is_uninitialized()) {
                    // we reached the end of the sector, we can continue from here
                    return null;
                }

                if (!header.is_valid()) {
                    it.corrupted_header_flag = true;
                    return null;
                }

                const next_offset = current_offset + alignForward(u32, header.len, WRITE_SIZE);
                if (next_offset > it.end_offset) {
                    it.corrupted_header_flag = true;
                    return null;
                }

                defer it.next_offset = next_offset;

                // if the tag is unreadable .in_use is the safe default
                const tag = it.storage.read_tag(current_offset) catch |err| switch (err) {
                    error.Corrupted => .in_use,
                    else => return err,
                };

                var key: Key = undefined;
                it.storage.read_retrying(current_offset + KEY_OFFSET, std.mem.asBytes(&key)) catch |err| switch (err) {
                    error.Corrupted => return .{
                        .offset = current_offset,
                        .tag = tag,
                        .header = header,
                        .key = .corrupted,
                    },
                    else => return err,
                };

                if (tag == .in_use) {
                    var crc: std.hash.crc.@"CRC-32/CKSUM" = .init();

                    crc.update(std.mem.asBytes(&key));

                    var check_offset = current_offset + VALUE_OFFSET;
                    const value_end = current_offset + header.len;
                    while (it.storage.read_buf_retrying(check_offset, value_end) catch |err| switch (err) {
                        error.Corrupted => return .{
                            .offset = current_offset,
                            .tag = tag,
                            .header = header,
                            .key = .corrupted,
                        },
                        else => return err,
                    }) |data| : (check_offset += @truncate(data.len)) {
                        crc.update(data);
                    }

                    if (header.data_crc != crc.final()) {
                        return .{
                            .offset = current_offset,
                            .tag = tag,
                            .header = header,
                            .key = .corrupted,
                        };
                    }
                }

                return .{
                    .offset = current_offset,
                    .tag = tag,
                    .header = header,
                    .key = .{ .ok = key },
                };
            }
        };
    };
}

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

        buf: []u8,
        page_programmed_bitset: std.bit_set.Dynamic,

        corruption_tripwire: ?u32 = null,
        power_loss_tripwire: ?u32 = null,
        power_loss_tripped: bool = false,

        pub fn init(gpa: std.mem.Allocator, size: usize) !Self {
            if (size % options.write_size != 0 or size % options.erase_size != 0) {
                return error.InvalidSize;
            }

            const buf: []u8 = try gpa.alloc(u8, size);
            errdefer gpa.free(buf);
            @memset(buf, 0xFF);

            const page_count = size / options.write_size;

            var page_programmed_bitset: std.bit_set.Dynamic = try .initEmpty(gpa, page_count);
            errdefer page_programmed_bitset.deinit(gpa);

            return .{
                .buf = buf,
                .page_programmed_bitset = page_programmed_bitset,
            };
        }

        pub fn deinit(self: *Self, gpa: std.mem.Allocator) void {
            gpa.free(self.buf);
            self.page_programmed_bitset.deinit(gpa);
        }

        pub fn erase(self: *Self, offset: u32, size: u32) error{InvalidRange}!void {
            if (offset + size > self.buf.len) {
                return error.InvalidRange;
            }
            if (!isAligned(u32, offset, ERASE_SIZE)) {
                return error.InvalidRange;
            }
            if (!isAligned(u32, size, ERASE_SIZE)) {
                return error.InvalidRange;
            }

            if (self.power_loss_tripped) {
                return;
            }

            var erase_offset: u32 = 0;
            while (erase_offset < size) : (erase_offset += WRITE_SIZE) {
                const page_id = (offset + erase_offset) / WRITE_SIZE;

                if (self.corruption_tripwire == page_id) {
                    self.corruption_tripwire = null;
                }

                if (self.power_loss_tripwire == page_id) {
                    self.power_loss_tripwire = null;
                    self.power_loss_tripped = true;
                    break;
                }

                self.page_programmed_bitset.unset(page_id);

                @memset(self.buf[offset..][erase_offset..][0..WRITE_SIZE], 0xFF);
            }
        }

        pub fn read_buf(self: *Self, offset: u32, end: u32) error{ InvalidRange, ReadFailed, Corrupted }!?[]const u8 {
            if (offset > self.buf.len or end > self.buf.len) return error.InvalidRange;

            if (self.corruption_tripwire) |corruption_tripwire| {
                if (offset <= corruption_tripwire and corruption_tripwire < end) {
                    return error.Corrupted;
                }
            }

            if (offset < end) {
                return self.buf[offset..end];
            } else {
                return null;
            }
        }

        pub fn read(self: *Self, offset: u32, data: []u8) error{ InvalidRange, ReadFailed, Corrupted }!void {
            if (offset + data.len > self.buf.len) return error.InvalidRange;

            if (self.corruption_tripwire) |corruption_tripwire| {
                if (offset <= corruption_tripwire and corruption_tripwire < offset + @as(u32, @truncate(data.len))) {
                    return error.Corrupted;
                }
            }

            std.mem.copyForwards(u8, data, self.buf[offset..][0..data.len]);
        }

        pub fn write(self: *Self, offset: u32, data: []const u8) error{ InvalidRange, WriteFailed, PageAlreadyProgrammed }!void {
            if (offset + data.len > self.buf.len) {
                return error.InvalidRange;
            }
            if (!isAligned(u32, offset, WRITE_SIZE)) {
                return error.InvalidRange;
            }
            if (!isAligned(usize, data.len, WRITE_SIZE)) {
                return error.InvalidRange;
            }

            if (self.power_loss_tripped) {
                return;
            }

            var copy_offset: u32 = 0;
            while (copy_offset < @as(u32, @truncate(data.len))) : (copy_offset += WRITE_SIZE) {
                const page_id = (offset + copy_offset) / WRITE_SIZE;

                if (self.power_loss_tripwire == page_id) {
                    self.power_loss_tripwire = null;
                    self.power_loss_tripped = true;
                    break;
                }

                if (self.page_programmed_bitset.isSet(page_id)) {
                    return error.PageAlreadyProgrammed;
                }
                self.page_programmed_bitset.set(page_id);

                std.mem.copyForwards(u8, self.buf[offset..][copy_offset..][0..WRITE_SIZE], data[copy_offset..][0..WRITE_SIZE]);
            }
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
        const TestStorage = StorageGeneric(TestFlash, u32, .{});
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
            // NOTE: this test takes a while because every read from the
            // MockFlash does an expensive corruption check. If we add caching
            // it should be faster.

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

// TODO: fuzzing when it is fixed
// test "storage.basic" {
//     try testing.fuzz({}, fuzz_storage, .{});
// }
//
// fn fuzz_storage(_: void, smith: *testing.Smith) !void {
//     const TestFlash = MockFlash(.{
//         .write_size = 4,
//         .erase_size = 1024,
//     });
//     const Storage = StorageGeneric(TestFlash, u32, .{});
//     const Value = u128;
//
//     const flash_size = 4 * 1024;
//
//     var flash: TestFlash = try .init(testing.allocator, flash_size);
//     defer flash.deinit(testing.allocator);
//
//     var storage: Storage = try .init(&flash, 0, flash_size);
//
//     var model: std.AutoHashMapUnmanaged(u32, Value) = .empty;
//     defer model.deinit(testing.allocator);
//
//     const Action = union(enum) {
//         fetch: struct {
//             key: u32,
//         },
//         store: struct {
//             key: u32,
//             value: Value,
//         },
//         // corrupt: u32,
//     };
//
//     for (0..10000) |_| {
//         const action = smith.value(Action);
//         switch (action) {
//             .fetch => |a| {
//                 const expected = model.get(a.key);
//                 const actual = try storage.fetch(a.key, Value);
//                 try testing.expectEqual(expected, actual);
//             },
//             .store => |a| {
//                 try model.put(testing.allocator, a.key, a.value);
//                 storage.store(a.key, a.value) catch |err| switch (err) {
//                     error.OutOfMemory => break,
//                     else => return err,
//                 };
//             },
//             // .corrupt => |offset| {
//             //     const aligned_offset = alignBackward(u32, std.math.clamp(offset, 0, flash_size - 1), TestFlash.WRITE_SIZE);
//             //     var it: Storage.ItemIterator = .{
//             //         .storage = storage,
//             //         .next_offset = aligned_offset,
//             //         .end_offset = alignBackward(u32, aligned_offset, TestFlash.ERASE_SIZE) + TestFlash.ERASE_SIZE,
//             //     };
//             //     while (try it.next()) |item| {}
//             // },
//         }
//     }
// }
//
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
