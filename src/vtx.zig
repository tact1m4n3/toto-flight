const hw = @import("hw.zig");
const control = @import("control.zig");
const Scheduler = @import("Scheduler.zig");
const Receiver = Scheduler.Receiver;

pub const VTX = struct {
    rcv_tick: Receiver(void) = undefined,

    enable_state: union(enum) {
        off_delay: u32,
        off,
        on,
    } = .{ .off_delay = 5 },

    pub fn init(vtx: *VTX, scheduler: *Scheduler) void {
        vtx.* = .{};

        hw.UART.get(.vtx).subscribe(*VTX, vtx, receive_callback, scheduler);
        hw.ticker(.@"1Hz").subscribe(&vtx.rcv_tick, *VTX, vtx, tick_callback, scheduler);
    }

    pub fn receive_callback(vtx: *VTX, byte: u8) void {
        _ = vtx;
        _ = byte;
    }

    pub fn tick_callback(vtx: *VTX, _: void) void {
        const status = control.get_status();

        switch (vtx.enable_state) {
            .off_delay => |*remaining_ticks| {
                if (remaining_ticks.* == 0) {
                    if (!status.armed) {
                        // turn off
                        vtx.enable_state = .off;
                    } else {
                        // turn on
                        vtx.enable_state = .on;
                    }
                } else {
                    remaining_ticks.* -= 1;
                }
            },
            .off => {
                if (status.armed) {
                    // turn on
                    vtx.enable_state = .on;
                }
            },
            .on => {
                if (!status.armed) {
                    vtx.enable_state = .{ .off_delay = 5 };
                }
            },
        }
    }
};
