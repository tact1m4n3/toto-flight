const std = @import("std");
const assert = std.debug.assert;
const builtin = @import("builtin");
const root = @import("root");

const hw = @import("hw.zig");
const time = @import("time.zig");
const Absolute = time.Absolute;
const Duration = time.Duration;

const Scheduler = @This();

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

        assert(task.state.swap(.waiting, .monotonic) == .ready);
        task.callback(task.context, task);
    }
}

pub const Task = struct {
    state: std.atomic.Value(State),
    scheduler: *Scheduler,
    context: ?*anyopaque,
    callback: *const fn (?*anyopaque, *Task) void,
    node: TransferStack.Node = .{},

    pub const uninitialized: Task = .{
        .state = .init(.uninitialized),
        .scheduler = undefined,
        .context = undefined,
        .callback = undefined,
    };

    pub const State = enum(u8) {
        uninitialized,
        initializing,
        waiting,
        ready,
    };

    pub fn init(
        Context: type,
        context: Context,
        comptime callback: fn (Context, *Task) void,
        scheduler: *Scheduler,
    ) Task {
        return .{
            .state = .init(.waiting),
            .scheduler = scheduler,
            .context = context,
            .callback = erase_context(Context, callback),
        };
    }

    /// Thread safe. Idempotent.
    pub fn init_late(
        task: *Task,
        Context: type,
        context: Context,
        comptime callback: fn (Context, *Task) void,
        scheduler: *Scheduler,
    ) void {
        assert(task.state.swap(.initializing, .acquire) == .uninitialized);

        task.scheduler = scheduler;
        task.context = context;
        task.callback = erase_context(Context, callback);

        assert(task.state.swap(.waiting, .release) == .initializing);
    }

    /// Thread safe.
    pub fn ready(task: *Task) void {
        if (task.state.swap(.ready, .acquire) == .waiting) {
            task.scheduler.ready_tasks.push(&task.node);
            task.scheduler.pend_fn();
        }
    }

    fn erase_context(
        Context: type,
        comptime callback: fn (Context, *Task) void,
    ) *const fn (?*anyopaque, *Task) void {
        const Erased = struct {
            pub fn wrapper(type_erased_context: ?*anyopaque, task: *Task) void {
                const context: Context = @ptrCast(@alignCast(type_erased_context));
                callback(context, task);
            }
        };
        return &Erased.wrapper;
    }
};

// TODO: see how much code duplication this produces because it is generic
pub fn Message(T: type) type {
    return struct {
        const Self = @This();

        pub const Value = T;

        /// Must only be accessed through a critical section.
        version: u32 = 0,
        /// Must only be accessed through a critical section.
        value: ?T = null,
        /// Must only be accessed through a critical section.
        receivers: std.SinglyLinkedList = .{},

        pub fn get(self: *Self) ?T {
            const cs = hw.enter_critical_section();
            defer cs.leave();
            return self.value;
        }

        pub fn get_with_version(self: *Self) struct { ?T, u32 } {
            const cs = hw.enter_critical_section();
            defer cs.leave();
            return .{ self.value, self.version };
        }

        pub fn publish(self: *Self, value: T) void {
            const cs = hw.enter_critical_section();
            defer cs.leave();

            self.value = value;
            self.version +%= 1;

            var it = self.receivers.first;
            while (it) |node| : (it = node.next) {
                const subscriber: *Receiver(T) = @alignCast(@fieldParentPtr("node", node));
                subscriber.task.ready();
                it = node.next;
            }
        }

        pub fn subscribe(
            self: *Self,
            receiver: *Receiver(T),
            Context: type,
            context: Context,
            comptime callback: fn (context: Context, value: T) void,
            scheduler: *Scheduler,
        ) void {
            receiver.* = .{
                .task = .init(Context, context, struct {
                    fn wrapper(ctx: Context, task: *Task) void {
                        const sub: *Receiver(T) = @alignCast(@fieldParentPtr("task", task));
                        const msg = sub.message;

                        const maybe_value = blk: {
                            const cs = hw.enter_critical_section();
                            defer cs.leave();
                            break :blk msg.value;
                        };

                        if (maybe_value) |value| {
                            callback(ctx, value);
                        }
                    }
                }.wrapper, scheduler),
                .message = self,
            };

            const cs = hw.enter_critical_section();
            defer cs.leave();

            self.receivers.prepend(&receiver.node);

            // if a value has already been published, immediately schedule the task
            if (self.value != null) {
                receiver.task.ready();
            }
        }
    };
}

pub fn Receiver(T: type) type {
    return struct {
        const Self = @This();

        task: Task,
        message: *Message(T),
        node: std.SinglyLinkedList.Node = .{},
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
            if (self.first.cmpxchgWeak(node.next, node, .release, .monotonic) == null) {
                break;
            }
        }
    }

    pub fn pop_all(self: *TransferStack) ?*Node {
        return self.first.swap(null, .monotonic);
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
    try testing.expectEqual(Task.State.waiting, task.state.load(.monotonic));
}
