const std = @import("std");

pub fn RingBuffer(T: type) type {
    return struct {
        const Self = @This();

        buffer: []T,
        read_pos: std.atomic.Value(usize) = .init(0),
        write_pos: std.atomic.Value(usize) = .init(0),

        pub fn init(buffer: []T) Self {
            return .{
                .buffer = buffer,
            };
        }

        pub fn is_empty(self: *Self) bool {
            return self.read_pos.load(.acquire) == self.write_pos.load(.acquire);
        }

        pub fn push(self: *Self, value: T) error{OutOfMemory}!void {
            const write_pos = self.write_pos.load(.monotonic);
            const next_write_pos = write_pos +% 1;

            const read_pos = self.read_pos.load(.acquire);

            if (next_write_pos -% read_pos > self.buffer.len) {
                return error.OutOfMemory;
            }

            self.buffer[write_pos % self.buffer.len] = value;
            self.write_pos.store(next_write_pos, .release);
        }

        pub fn push_many(self: *Self, values: []const T) error{OutOfMemory}!void {
            const write_pos = self.write_pos.load(.monotonic);
            const next_write_pos = write_pos +% values.len;

            const read_pos = self.read_pos.load(.acquire);

            if (next_write_pos -% read_pos > self.buffer.len) {
                return error.OutOfMemory;
            }

            const write_index = write_pos % self.buffer.len;
            const next_write_index = next_write_pos % self.buffer.len;

            if (write_index < next_write_index) {
                std.mem.copyForwards(T, self.buffer[write_index..next_write_index], values);
            } else {
                const first_part_len = self.buffer.len - write_index;
                std.mem.copyForwards(T, self.buffer[write_index..], values[0..first_part_len]);
                std.mem.copyForwards(T, self.buffer[0..next_write_index], values[first_part_len..]);
            }

            self.write_pos.store(next_write_pos, .release);
        }

        pub fn pop(self: *Self) ?T {
            const write_pos = self.write_pos.load(.acquire);
            const read_pos = self.read_pos.load(.monotonic);

            if (write_pos == read_pos) {
                return null;
            }

            const value = self.buffer[read_pos % self.buffer.len];
            self.read_pos.store(read_pos +% 1, .release);
            return value;
        }
    };
}

const testing = std.testing;

test "push/pop preserves FIFO order" {
    var buffer: [4]u32 = undefined;
    var rb: RingBuffer(u32) = .init(&buffer);
    try rb.push(1);
    try rb.push(2);
    try testing.expectEqual(@as(?u32, 1), rb.pop());
    try testing.expectEqual(@as(?u32, 2), rb.pop());
    try testing.expectEqual(@as(?u32, null), rb.pop());
    try rb.push_many(&.{ 3, 4, 5 });
    try testing.expectEqual(@as(?u32, 3), rb.pop());
    try testing.expectEqual(@as(?u32, 4), rb.pop());
    try testing.expectEqual(@as(?u32, 5), rb.pop());
    try testing.expectEqual(@as(?u32, null), rb.pop());
}

test "buffer full/empty boundaries" {
    var buffer: [2]u32 = undefined;
    var rb: RingBuffer(u32) = .init(&buffer);
    try rb.push_many(&.{ 1, 2 });
    try testing.expectError(error.OutOfMemory, rb.push(3));
    _ = rb.pop();
    try rb.push(3); // room again after a pop
}
