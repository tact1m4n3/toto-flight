const hw = @import("hw.zig");
const Message = @import("Scheduler.zig").Message;

pub const tickers = struct {
    pub var @"100Hz": Message(void) = .{};
};

pub fn get_time_since_boot() Absolute {
    return hw.chip.get_time_since_boot();
}

pub const Absolute = enum(u64) {
    _,

    pub fn from_us(us: u64) Absolute {
        return @as(Absolute, @fromBackingInt(@intCast(us)));
    }

    pub fn to_us(abs: Absolute) u64 {
        return @backingInt(abs);
    }

    pub fn is_reached_by(deadline: Absolute, point: Absolute) bool {
        return deadline.to_us() <= point.to_us();
    }

    pub fn diff(future: Absolute, past: Absolute) Duration {
        return Duration.from_us(future.to_us() - past.to_us());
    }

    pub fn saturating_diff(future: Absolute, past: Absolute) Duration {
        return Duration.from_us(future.to_us() -| past.to_us());
    }

    pub fn add_duration(abs: Absolute, dur: Duration) Absolute {
        return Absolute.from_us(abs.to_us() + dur.to_us());
    }

    pub fn earliest(abs: Absolute, other: Absolute) Absolute {
        if (abs.to_us() < other.to_us()) {
            return abs;
        } else {
            return other;
        }
    }
};

pub const Duration = enum(u64) {
    _,

    pub fn from_us(us: u64) Duration {
        return @as(Duration, @fromBackingInt(@intCast(us)));
    }

    pub fn from_ms(ms: u64) Duration {
        return from_us(1000 * ms);
    }

    pub fn from_hz(hz: u64) Duration {
        return .from_us(1_000_000 / hz);
    }

    pub fn to_us(duration: Duration) u64 {
        return @backingInt(duration);
    }

    pub fn to_ms(duration: Duration) u64 {
        return @backingInt(duration) / 1000;
    }

    pub fn less_than(self: Duration, other: Duration) bool {
        return self.to_us() < other.to_us();
    }

    pub fn minus(self: Duration, other: Duration) Duration {
        return from_us(self.to_us() - other.to_us());
    }

    pub fn plus(self: Duration, other: Duration) Duration {
        return from_us(self.to_us() + other.to_us());
    }

    pub fn max(self: Duration, other: Duration) Duration {
        if (self.to_us() > other.to_us()) {
            return self;
        } else {
            return other;
        }
    }
};
