const std = @import("std");
const assert = std.debug.assert;
const builtin = @import("builtin");
const root = @import("root");

const hw = @import("hw.zig");
const time = @import("time.zig");
const Absolute = time.Absolute;
const Duration = time.Duration;

const Scheduler = @This();

// TODO: Make an EventGroup kind of type, so you could listen for armed, failsafe
// etc. events directly

pend_fn: *const fn () void,
ready_tasks: TransferStack,

pub fn init(pend_fn: *const fn () void) Scheduler {
    return .{
        .pend_fn = pend_fn,
        .ready_tasks = .{},
    };
}

pub fn run(scheduler: *Scheduler) void {
    var it = scheduler.ready_tasks.pop_all();
    while (it) |node| {
        it = node.next;

        const task: *Task = @fieldParentPtr("node", node);
        task.callback(task.context, task);
    }
}

pub const Task = struct {
    state: std.atomic.Value(State),
    scheduler: *Scheduler,
    context: ?*anyopaque,
    callback: *const fn (?*anyopaque, *Task) void,
    node: TransferStack.Node = .{},

    pub const State = enum(u8) {
        idle,
        ready,
    };

    pub fn init(
        Context: type,
        context: Context,
        comptime callback: fn (Context, *Task) void,
        scheduler: *Scheduler,
    ) Task {
        return .{
            .state = .init(.idle),
            .scheduler = scheduler,
            .context = context,
            .callback = init_callback(Context, callback),
        };
    }

    /// Thread safe.
    pub fn ready(task: *Task) void {
        if (task.state.swap(.ready, .acquire) == .idle) {
            task.scheduler.ready_tasks.push(&task.node);
            task.scheduler.pend_fn();
        }
    }

    fn init_callback(
        Context: type,
        comptime callback: fn (Context, *Task) void,
    ) *const fn (?*anyopaque, *Task) void {
        const Erased = struct {
            pub fn wrapper(type_erased_context: ?*anyopaque, task: *Task) void {
                const context: Context = @ptrCast(@alignCast(type_erased_context));
                assert(task.state.swap(.idle, .release) == .ready);
                callback(context, task);
            }
        };
        return &Erased.wrapper;
    }
};

pub const Waker = struct {
    wakey: std.atomic.Value(?*Task) = .init(null),

    pub fn register(
        waker: *Waker,
        task: *Task,
    ) void {
        assert(waker.wakey.swap(task, .release) == null);
    }

    pub fn wake(waker: *Waker) void {
        const task = waker.wakey.load(.monotonic) orelse return;
        task.ready();
    }
};

// TODO: see how much code duplication this produces because it is generic
pub fn Message(T: type) type {
    return struct {
        const Self = @This();

        pub const Value = T;

        /// Must only be accessed through a critical section.
        value: ?T = null,
        /// Must only be accessed through a critical section.
        receivers: std.SinglyLinkedList = .{},

        pub fn get(message: *Self) ?T {
            const cs = hw.enter_critical_section();
            defer cs.leave();
            return message.value;
        }

        pub fn publish(message: *Self, value: T) void {
            const cs = hw.enter_critical_section();
            defer cs.leave();

            message.value = value;

            var it = message.receivers.first;
            while (it) |node| : (it = node.next) {
                const subscriber: *Receiver(T) = @alignCast(@fieldParentPtr("node", node));
                subscriber.task.ready();
            }
        }

        pub fn subscribe(
            message: *Self,
            receiver: *Receiver(T),
            Context: type,
            context: Context,
            comptime callback: fn (context: Context, value: T) void,
            scheduler: *Scheduler,
        ) void {
            receiver.* = .{
                .task = .init(Context, context, struct {
                    fn wrapper(ctx: Context, task: *Task) void {
                        const rcv: *Receiver(T) = @alignCast(@fieldParentPtr("task", task));
                        if (rcv.message.get()) |value| {
                            callback(ctx, value);
                        }
                    }
                }.wrapper, scheduler),
                .message = message,
            };

            const cs = hw.enter_critical_section();
            defer cs.leave();

            message.receivers.prepend(&receiver.node);

            // if a value has already been published, immediately schedule the
            // task (it will run after init)
            if (message.value != null) {
                receiver.task.ready();
            }
        }
    };
}

pub fn Receiver(T: type) type {
    return struct {
        task: Task,
        message: *Message(T),
        node: std.SinglyLinkedList.Node = .{},
    };
}

pub fn ParamTable(T: type) type {
    return struct {
        const Self = @This();

        pub const Type = T;

        /// Must only be accessed through a critical section.
        value: ?T = null,
        /// Must only be accessed through a critical section.
        version: u32 = 0,

        pub fn get(table: *Self) ?T {
            const cs = hw.enter_critical_section();
            defer cs.leave();
            return table.value;
        }

        pub fn get_with_version(table: *Self) struct { ?T, u32 } {
            const cs = hw.enter_critical_section();
            defer cs.leave();
            return .{ table.value, table.version };
        }

        pub fn update(table: *Self, value: T) void {
            const cs = hw.enter_critical_section();
            defer cs.leave();
            table.value = value;
            table.version += 1;
        }
    };
}

const TransferStack = struct {
    first: std.atomic.Value(?*Node) = .init(null),

    pub const Node = struct {
        next: ?*Node = null,
    };

    pub fn push(self: *TransferStack, node: *Node) void {
        while (true) {
            node.next = self.first.load(.monotonic);
            if (self.first.cmpxchgWeak(node.next, node, .acq_rel, .monotonic) == null) {
                break;
            }
        }
    }

    pub fn pop_all(self: *TransferStack) ?*Node {
        return self.first.swap(null, .acquire);
    }
};

const testing = std.testing;

test "TransferStack" {
    var stack: TransferStack = .{};
    var n1: TransferStack.Node = .{};
    var n2: TransferStack.Node = .{};

    stack.push(&n1);
    stack.push(&n2);

    const head = stack.pop_all();
    try testing.expectEqual(@as(?*TransferStack.Node, &n2), head);
    try testing.expectEqual(@as(?*TransferStack.Node, &n1), n2.next);
    try testing.expectEqual(@as(?*TransferStack.Node, null), stack.pop_all());
}

test "Scheduler" {
    const Pend = struct {
        var called: bool = false;
        fn pend() void {
            called = true;
        }
    };
    var scheduler = Scheduler.init(Pend.pend);

    var ran = false;
    var task = Task.init(*bool, &ran, struct {
        fn cb(ctx: *bool, _: *Task) void {
            ctx.* = true;
        }
    }.cb, &scheduler);

    task.ready();
    try testing.expect(Pend.called);
    try testing.expect(!ran); // queued, not yet run

    scheduler.run();
    try testing.expect(ran);
    try testing.expectEqual(Task.State.idle, task.state.load(.monotonic));
}
