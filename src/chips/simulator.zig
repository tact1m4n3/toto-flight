const time = @import("../time.zig");

pub fn get_absolute_time() time.Absolute {
    return @import("root").time_absolute;
}

pub fn enter_critical_section() CriticalSection {}
pub const CriticalSection = struct {
    pub fn leave(_: CriticalSection) void {}
};
