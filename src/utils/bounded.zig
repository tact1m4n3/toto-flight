const std = @import("std");

pub fn Array(T: type, capacity: usize) type {
    return struct {
        const Self = @This();

        buffer: [capacity]T = undefined,
        len: usize,

        pub const empty: Self = .{
            .len = 0,
        };

        pub fn items(self: *Self) []T {
            return self.buffer[0..self.len];
        }

        pub fn append(self: *Self, value: T) error{OutOfMemory}!void {
            if (self.len >= capacity) {
                return error.OutOfMemory;
            }

            self.buffer[self.len] = value;
            self.len += 1;
        }

        pub fn pop(self: *Self) ?T {
            if (self.len == 0) {
                return null;
            }

            self.len -= 1;
            return self.buffer[self.len];
        }
    };
}

pub fn Queue(T: type, capacity: usize) type {
    return struct {
        const Self = @This();

        buffer: [capacity]T = undefined,
        read_pos: usize,
        write_pos: usize,
        len: usize,

        pub const empty: Self = .{
            .read_pos = 0,
            .write_pos = 0,
            .len = 0,
        };

        pub fn append(self: *Self, value: T) error{OutOfMemory}!void {
            if (self.len >= capacity) {
                return error.OutOfMemory;
            }

            self.buffer[self.write_pos] = value;
            self.write_pos = (self.write_pos + 1) % capacity;
            self.len += 1;
        }

        pub fn pop(self: *Self) ?T {
            if (self.len == 0) {
                return null;
            }

            const value = self.buffer[self.read_pos];
            self.read_pos = (self.read_pos + 1) % capacity;
            self.len -= 1;
            return value;
        }
    };
}
