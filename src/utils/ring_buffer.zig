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

        pub fn push(self: *Self, value: T) error{BufferFull}!void {
            const write_pos = self.write_pos.load(.monotonic);
            const next_write_pos = write_pos +% 1;

            const read_pos = self.read_pos.load(.acquire);

            if (next_write_pos -% read_pos > self.buffer.len) {
                return error.BufferFull;
            }

            self.buffer[write_pos % self.buffer.len] = value;
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
}

test "buffer full/empty boundaries" {
    var buffer: [2]u32 = undefined;
    var rb: RingBuffer(u32) = .init(&buffer);
    try rb.push(1);
    try rb.push(2);
    try testing.expectError(error.BufferFull, rb.push(3));
    _ = rb.pop();
    try rb.push(3); // room again after a pop
}
