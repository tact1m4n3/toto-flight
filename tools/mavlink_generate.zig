const std = @import("std");
const Io = std.Io;
const Allocator = std.mem.Allocator;
const eql = std.mem.eql;
const xml = @import("xml");

pub const std_options: std.Options = .{
    // .log_level = .warn,
};

// TODO: maybe format identifier names in some way

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const gpa = init.gpa;
    const arena = init.arena.allocator();

    const args = try init.minimal.args.toSlice(arena);

    if (args.len < 4) {
        std.debug.print("usage: {s} <input_file> <output_file> <document>\n", .{args[0]});
        return error.InvalidArguments;
    }

    const defs_path = args[1];
    const output_file = args[2];
    const document = args[3];

    var data = blk: {
        var parser: Parser = try .init(io, gpa, arena, defs_path);
        defer parser.deinit();

        try parser.visit_document(document);

        break :blk try parser.get_data();
    };

    remove_mav_bool(&data);
    try resolve_enum_types(&data);
    reorder_message_fields_by_size(&data);
    compute_crc_extra(&data);

    try gen_file(io, &data, output_file);
}

pub fn remove_mav_bool(data: *Data) void {
    if (!data.enums.remove("MAV_BOOL")) {
        return;
    }

    var msg_it = data.messages.valueIterator();
    while (msg_it.next()) |msg| {
        for (msg.fields) |*field| {
            const enum_name = field.@"enum" orelse continue;
            if (eql(u8, enum_name, "MAV_BOOL")) {
                field.@"enum" = null;
                field.type = .{ .primitive = .bool };
            }
        }
    }
}

/// Infers the tag type of every enum from the fields referencing it.
pub fn resolve_enum_types(data: *Data) !void {
    var msg_it = data.messages.valueIterator();
    while (msg_it.next()) |msg| {
        for (msg.fields) |field| {
            const enum_name = field.@"enum" orelse continue;
            std.log.debug("resolving enum {s} from field {s}", .{ enum_name, field.name });
            const enum_ptr = data.enums.getPtr(enum_name) orelse return error.InvalidEnumReference;
            const primitive = switch (field.type) {
                .primitive => |p| p,
                .array => |a| a.primitive,
            };
            if (enum_ptr.resolved_tag_type) |existing| {
                if (existing != primitive) {
                    std.log.err("enum {s} used as both {t} and {t}", .{
                        enum_name,
                        existing,
                        primitive,
                    });
                    enum_ptr.resolved_tag_type = null;
                }
            } else {
                enum_ptr.resolved_tag_type = primitive;
            }
        }
    }

    var enum_it = data.enums.valueIterator();
    while (enum_it.next()) |@"enum"| {
        if (@"enum".resolved_tag_type == null) {
            var min_value: u32 = 0;
            for (@"enum".entries) |entry| {
                min_value = @max(min_value, entry.value);
            }
            const resolved: Data.Type.Primitive = switch (min_value) {
                0...0xff => .uint8_t,
                0x100...0xffff => .uint16_t,
                0x10000...0xffffffff => .uint32_t,
            };
            std.log.warn("enum {s} has no fields referencing it, inferring tag type as {t}", .{ @"enum".name, resolved });
            @"enum".resolved_tag_type = resolved;
        }
    }
}

pub fn reorder_message_fields_by_size(data: *Data) void {
    var msg_it = data.messages.iterator();
    while (msg_it.next()) |msg_entry| {
        const msg = msg_entry.value_ptr;
        std.sort.insertion(Data.Message.Field, msg.fields[0..msg.extension_fields_start], {}, struct {
            fn less(_: void, a: Data.Message.Field, b: Data.Message.Field) bool {
                return a.type.size() > b.type.size();
            }
        }.less);
    }
}

pub fn compute_crc_extra(data: *Data) void {
    var msg_it = data.messages.iterator();
    while (msg_it.next()) |msg_entry| {
        const msg = msg_entry.value_ptr;
        var crc: std.hash.crc.@"CRC-16/MCRF4XX" = .init();
        crc.update(msg.name);
        crc.update(" ");
        for (msg.fields, 0..) |field, i| {
            if (i == msg.extension_fields_start) {
                break;
            }
            const type_str = switch (field.type) {
                .primitive => |p| @tagName(p),
                .array => |a| @tagName(a.primitive),
            };
            crc.update(type_str);
            crc.update(" ");
            crc.update(field.name);
            crc.update(" ");
            switch (field.type) {
                .array => |a| {
                    crc.update(&.{@as(u8, @intCast(a.count))});
                },
                else => {},
            }
        }
        const final = crc.final();
        const extra: u8 = @as(u8, @intCast(final & 0xff)) ^ @as(u8, @intCast(final >> 8));
        msg.crc_extra = extra;
    }
}

pub fn gen_file(io: Io, data: *Data, output_file: []const u8) !void {
    const file = try Io.Dir.cwd().createFile(io, output_file, .{});
    defer file.close(io);

    var writer_buf: [1024]u8 = undefined;
    var file_writer = file.writer(io, &writer_buf);
    const writer = &file_writer.interface;
    try writer.writeAll(
        \\///
        \\/// This file is auto-generated using tools/mavlink_generate.zig.
        \\/// DO NOT EDIT MANUALLY.
        \\///
        \\const MAVLINK = @This();
        \\
    );

    {
        try writer.writeAll(
            \\
            \\pub const MessageId = enum(u24) {
            \\
        );

        var it = data.messages.valueIterator();
        while (it.next()) |msg| {
            if (msg.description.len > 0) {
                try gen_docs(writer, "    ", msg.description);
            }
            try writer.print("    {f} = 0x{X:0>6},\n", .{ std.zig.fmtId(msg.name), msg.id });
        }

        try writer.writeAll(
            \\
            \\    pub fn get_crc_extra(self: MessageId) u8 {
            \\        return switch (self) {
            \\
        );
        it = data.messages.valueIterator();
        while (it.next()) |msg| {
            if (msg.crc_extra) |crc| {
                try writer.print("            .{f} => 0x{X:0>2},\n", .{ std.zig.fmtId(msg.name), crc });
            } else {
                std.log.warn("message {s} has no crc_extra", .{msg.name});
            }
        }
        try writer.writeAll(
            \\        };
            \\    }
            \\};
            \\
        );
    }

    {
        try writer.writeAll(
            \\
            \\pub const enums = struct {
            \\
        );
        var it = data.enums.valueIterator();
        while (it.next()) |@"enum"| {
            const resolved = @"enum".resolved_tag_type orelse {
                std.log.warn("enum {s} has no resolved tag type... ignoring", .{@"enum".name});
                continue;
            };
            const bit_size = resolved.size() * @bitSizeOf(u8);
            const zig_type = resolved.to_zig_type();

            if (@"enum".description.len > 0) {
                try gen_docs(writer, "    ", @"enum".description);
            }
            if (@"enum".deprecation_note) |note| {
                try gen_deprecation_note(writer, "    ", note);
            }

            if (!@"enum".bitmask) {
                try writer.print("    pub const {f} = enum({s}) {{\n", .{
                    std.zig.fmtId(@"enum".name),
                    zig_type,
                });

                for (@"enum".entries) |entry| {
                    if (entry.description.len > 0) {
                        try gen_docs(writer, "        ", entry.description);
                    }
                    if (entry.deprecation_note) |note| {
                        try gen_deprecation_note(writer, "        ", note);
                    }
                    if (entry.wip_flag) {
                        try writer.writeAll("        /// WIP\n");
                    }
                    try writer.print("        {f} = {d},\n", .{
                        std.zig.fmtId(entry.name),
                        entry.value,
                    });
                }

                try writer.writeAll(
                    \\    };
                    \\
                );
            } else {
                // Ensure the entries are sorted by value.
                std.mem.sort(Data.Enum.Entry, @"enum".entries, {}, struct {
                    fn less(_: void, a: Data.Enum.Entry, b: Data.Enum.Entry) bool {
                        return a.value < b.value;
                    }
                }.less);

                try writer.print("    pub const {f} = packed struct({s}) {{\n", .{
                    std.zig.fmtId(@"enum".name),
                    zig_type,
                });

                var i: usize = 0;
                for (@"enum".entries) |entry| {
                    defer i += 1;

                    while (entry.value >> @truncate(i) != 1 and i < bit_size) : (i += 1) {
                        try writer.print("        reserved{d}: u1 = 0,\n", .{i});
                    }

                    if (entry.description.len > 0) {
                        try gen_docs(writer, "        ", entry.description);
                    }
                    if (entry.deprecation_note) |note| {
                        try gen_deprecation_note(writer, "        ", note);
                    }
                    if (entry.wip_flag) {
                        try writer.writeAll("        /// WIP\n");
                    }
                    try writer.print("        {f}: bool = false,\n", .{std.zig.fmtId(entry.name)});
                }

                while (i < bit_size) : (i += 1) {
                    try writer.print("        reserved{d}: u1 = 0,\n", .{i});
                }

                try writer.writeAll(
                    \\    };
                    \\
                );
            }
        }

        try writer.writeAll(
            \\};
            \\
        );
    }

    {
        try writer.writeAll(
            \\
            \\pub const messages = struct {
            \\
        );

        var it = data.messages.valueIterator();
        while (it.next()) |msg| {
            if (msg.description.len > 0) {
                try gen_docs(writer, "    ", msg.description);
            }
            if (msg.deprecation_note) |note| {
                try gen_deprecation_note(writer, "    ", note);
            }
            if (msg.wip_flag) {
                try writer.writeAll("    /// WIP\n");
            }
            try writer.print("    pub const {f} = struct {{\n", .{std.zig.fmtId(msg.name)});

            var use_zero_default: bool = false;
            for (msg.fields, 0..) |field, i| {
                if (i == msg.extension_fields_start) {
                    try writer.writeAll(
                        \\
                        \\        // Extensions
                        \\
                    );
                    use_zero_default = true;
                }
                if (field.description.len > 0) {
                    try gen_docs(writer, "        ", field.description);
                }
                try writer.print("        {f}: ", .{std.zig.fmtId(field.name)});

                const maybe_enum_name = if (field.@"enum") |enum_name| blk: {
                    const enum_ptr = data.enums.getPtr(enum_name) orelse return error.InvalidEnumReference;
                    if (enum_ptr.resolved_tag_type) |_| {
                        break :blk enum_name;
                    } else {
                        std.log.warn("enum {s} has no resolved tag type... using field type instead", .{enum_name});
                        break :blk null;
                    }
                } else null;

                if (maybe_enum_name) |enum_name| {
                    try writer.print("enums.{f}", .{std.zig.fmtId(enum_name)});
                    // TODO: maybe assert that there actually is a zero field
                    if (use_zero_default) try writer.writeAll(" = @fromBackingInt(0)");
                } else {
                    switch (field.type) {
                        .primitive => |p| {
                            try writer.writeAll(p.to_zig_type());
                            if (use_zero_default) try writer.writeAll(" = 0");
                        },
                        .array => |a| {
                            try writer.print("[{}]{s}", .{ a.count, a.primitive.to_zig_type() });
                            if (use_zero_default) try writer.writeAll(" = @splat(0)");
                        },
                    }
                }

                try writer.writeAll(",\n");
            }

            try writer.writeAll(
                \\    };
                \\
            );
        }

        try writer.writeAll(
            \\};
            \\
        );
    }

    try writer.flush();

    std.log.info("output file ready: {s}", .{output_file});
}

fn gen_docs(writer: *Io.Writer, ident: []const u8, text: []const u8) !void {
    var it = std.mem.splitScalar(u8, text, '\n');
    while (it.next()) |line| {
        const trimmed = std.mem.trim(u8, line, &.{ ' ', '\t' });
        if (trimmed.len == 0) continue;
        try writer.print("{s}/// {s}\n", .{ ident, trimmed });
    }
}

fn gen_deprecation_note(writer: *Io.Writer, ident: []const u8, deprecation_note: Data.DeprecationNote) !void {
    try writer.print("{s}/// DEPRECATED({s})\n", .{
        ident,
        deprecation_note.since,
    });
    if (deprecation_note.replaced_by.len > 0) {
        const trimmed = std.mem.trim(u8, deprecation_note.replaced_by, &.{ ' ', '\t' });
        try writer.print("{s}/// Replaced by: {s}\n", .{ ident, trimmed });
    }
    try gen_docs(writer, ident, deprecation_note.text);
}

pub const Data = struct {
    enums: std.StringHashMapUnmanaged(Data.Enum),
    messages: std.StringHashMapUnmanaged(Data.Message),

    pub const Enum = struct {
        name: []const u8,
        bitmask: bool,
        description: []const u8,
        deprecation_note: ?DeprecationNote,
        entries: []Entry,
        resolved_tag_type: ?Type.Primitive,

        pub const Entry = struct {
            name: []const u8,
            value: u32,
            description: []const u8,
            deprecation_note: ?DeprecationNote,
            wip_flag: bool,
            params: []Param,

            pub const Param = struct {
                index: usize,
                label: ?[]const u8,
                units: ?[]const u8, // TODO: maybe typed
                description: []const u8,
            };
        };
    };

    pub const Message = struct {
        name: []const u8,
        id: u24,
        description: []const u8,
        fields: []Field,
        extension_fields_start: usize,
        deprecation_note: ?DeprecationNote,
        wip_flag: bool,
        crc_extra: ?u8 = null,

        pub const Field = struct {
            name: []const u8,
            type: Type,
            @"enum": ?[]const u8,
            description: []const u8,
        };
    };

    pub const DeprecationNote = struct {
        since: []const u8,
        replaced_by: []const u8,
        text: []const u8,
    };

    pub const Type = union(enum) {
        primitive: Primitive,
        array: struct {
            primitive: Primitive,
            count: u8,
        },

        pub const Primitive = enum {
            bool,
            char,
            uint8_t,
            int8_t,
            uint16_t,
            int16_t,
            uint32_t,
            int32_t,
            uint64_t,
            int64_t,
            float,
            double,

            pub fn from_str(str: []const u8) ?Primitive {
                for (std.enums.values(Primitive)) |primitive| {
                    // NOTE: we do this because there is a magic type
                    // uint8_t_magic_version that needs to resolve to uint8_t
                    if (std.mem.startsWith(u8, str, @tagName(primitive))) {
                        return primitive;
                    }
                } else return null;
            }

            pub fn size(self: Primitive) usize {
                return switch (self) {
                    .bool => 1,
                    .char => 1,
                    .uint8_t => 1,
                    .int8_t => 1,
                    .uint16_t => 2,
                    .int16_t => 2,
                    .uint32_t => 4,
                    .int32_t => 4,
                    .uint64_t => 8,
                    .int64_t => 8,
                    .float => 4,
                    .double => 8,
                };
            }

            pub fn to_zig_type(self: Primitive) []const u8 {
                return switch (self) {
                    .bool => "bool",
                    .char => "i8",
                    .uint8_t => "u8",
                    .int8_t => "i8",
                    .uint16_t => "u16",
                    .int16_t => "i16",
                    .uint32_t => "u32",
                    .int32_t => "i32",
                    .uint64_t => "u64",
                    .int64_t => "i64",
                    .float => "f32",
                    .double => "f64",
                };
            }
        };

        pub fn size(self: Type) usize {
            return switch (self) {
                .primitive => |primitive| primitive.size(),
                .array => |array| array.primitive.size() * array.count,
            };
        }
    };
};

pub const Parser = struct {
    io: Io,
    gpa: Allocator,
    arena: Allocator,

    defs_dir: Io.Dir,
    visited_documents: std.StringHashMapUnmanaged(void),

    enums: std.StringHashMapUnmanaged(Data.Enum),
    messages: std.StringHashMapUnmanaged(Data.Message),

    pub fn init(io: Io, gpa: Allocator, arena: Allocator, defs_path: []const u8) !Parser {
        const defs_dir = try Io.Dir.cwd().openDir(io, defs_path, .{});
        errdefer defs_dir.close(io);

        return .{
            .io = io,
            .gpa = gpa,
            .arena = arena,

            .defs_dir = defs_dir,
            .visited_documents = .empty,

            .enums = .empty,
            .messages = .empty,
        };
    }

    pub fn deinit(p: *Parser) void {
        p.visited_documents.deinit(p.gpa);
        p.enums.deinit(p.gpa);
        p.messages.deinit(p.gpa);
        p.defs_dir.close(p.io);
    }

    /// Exports the acquired data. The lifetime of the data is the same as the input arena.
    pub fn get_data(p: *Parser) !Data {
        return .{
            .enums = try p.enums.clone(p.arena),
            .messages = try p.messages.clone(p.arena),
        };
    }

    pub fn visit_document(p: *Parser, name: []const u8) !void {
        if (p.visited_documents.contains(name)) return error.DocumentAlreadyVisited;
        try p.visited_documents.put(p.gpa, name, {});

        var input_file = try p.defs_dir.openFile(p.io, name, .{
            .mode = .read_only,
        });
        defer input_file.close(p.io);

        var input_buf: [1024]u8 = undefined;
        var input_reader = input_file.reader(p.io, &input_buf);

        var streaming_reader: xml.Reader.Streaming = .init(p.gpa, &input_reader.interface, .{});
        defer streaming_reader.deinit();
        const reader = &streaming_reader.interface;

        while (true) {
            const node = try reader.read();
            switch (node) {
                .eof => break,
                .element_start => {
                    const element_name = reader.elementName();
                    std.log.debug("visiting element {s}", .{element_name});
                    if (eql(u8, element_name, "mavlink")) {
                        //
                    } else if (eql(u8, element_name, "version")) {
                        // TODO
                    } else if (eql(u8, element_name, "dialect")) {
                        // TODO
                    } else if (eql(u8, element_name, "include")) {
                        const include_name = switch (try reader.read()) {
                            .text => try p.arena.dupe(u8, try reader.text()),
                            else => return error.InvalidDescription,
                        };
                        std.log.debug("include found {s}", .{include_name});
                        try visit_document(p, include_name);
                        std.log.debug("include ended {s}", .{include_name});
                        try reader.skipElement();
                    } else if (eql(u8, element_name, "enums")) {
                        //
                    } else if (eql(u8, element_name, "messages")) {
                        //
                    } else if (eql(u8, element_name, "enum")) {
                        try p.visit_enum(reader);
                    } else if (eql(u8, element_name, "message")) {
                        try p.visit_message(reader);
                    } else {
                        return error.UnexpectedElement;
                    }
                },
                else => {},
            }
        }
    }

    fn visit_enum(p: *Parser, reader: *xml.Reader) !void {
        const name_idx = reader.attributeIndex("name") orelse return error.EnumNoName;
        const name = try reader.attributeValueAlloc(p.arena, name_idx);

        const bitmask = if (reader.attributeIndex("bitmask")) |bitmask_idx|
            eql(u8, try reader.attributeValueAlloc(p.arena, bitmask_idx), "true")
        else
            false;

        std.log.debug("visiting enum {s} bitmask = {}", .{ name, bitmask });

        var description: []const u8 = "";
        var entries: std.ArrayListUnmanaged(Data.Enum.Entry) = .empty;
        defer entries.deinit(p.gpa);
        var deprecation_note: ?Data.DeprecationNote = null;

        while (true) {
            const node = try reader.read();
            switch (node) {
                .eof => return error.UnexpectedEof,
                .element_start => {
                    const element_name = reader.elementName();
                    if (eql(u8, element_name, "entry")) {
                        const entry = try p.parse_enum_entry(reader);
                        try entries.append(p.gpa, entry);
                    } else if (eql(u8, element_name, "description")) {
                        description = try p.parse_description(reader);
                    } else if (is_deprecated(element_name)) {
                        deprecation_note = try p.parse_deprecation_note(reader);
                    } else {
                        std.log.warn("unhandled element in enum: {s}", .{element_name});
                        try reader.skipElement();
                    }
                },
                .element_end => {
                    const element_name = reader.elementName();
                    if (eql(u8, element_name, "enum")) {
                        break;
                    } else {
                        return error.UnexpectedElementEnd;
                    }
                },
                else => {},
            }
        }

        try p.enums.put(p.gpa, name, .{
            .name = name,
            .bitmask = bitmask,
            .description = description,
            .deprecation_note = deprecation_note,
            .entries = try p.arena.dupe(Data.Enum.Entry, entries.items),
            .resolved_tag_type = null,
        });
    }

    fn parse_enum_entry(p: *Parser, reader: *xml.Reader) !Data.Enum.Entry {
        const name_idx = reader.attributeIndex("name") orelse return error.EnumEntryNoName;
        const name = try reader.attributeValueAlloc(p.arena, name_idx);

        const value_idx = reader.attributeIndex("value") orelse return error.EnumEntryNoValue;
        const value_str = try reader.attributeValue(value_idx);
        const value = std.fmt.parseInt(u32, value_str, 10) catch return error.EnumEntryInvalidValue;

        std.log.debug("visiting enum field {s} = {}", .{ name, value });

        var description: []const u8 = "";
        var deprecation_note: ?Data.DeprecationNote = null;
        var wip_flag: bool = false;
        var params: std.ArrayListUnmanaged(Data.Enum.Entry.Param) = .empty;
        defer params.deinit(p.gpa);

        while (true) {
            const node = try reader.read();
            switch (node) {
                .eof => return error.UnexpectedEof,
                .element_start => {
                    const element_name = reader.elementName();
                    if (eql(u8, element_name, "description")) {
                        description = try p.parse_description(reader);
                    } else if (is_deprecated(element_name)) {
                        deprecation_note = try p.parse_deprecation_note(reader);
                    } else if (eql(u8, element_name, "wip")) {
                        wip_flag = true;
                        try reader.skipElement();
                    } else if (eql(u8, element_name, "param")) {
                        const param = try p.parse_enum_entry_param(reader);
                        try params.append(p.gpa, param);
                    } else {
                        std.log.warn("unhandled element in enum entry: {s}", .{element_name});
                        try reader.skipElement();
                    }
                },
                .element_end => {
                    const element_name = reader.elementName();
                    if (eql(u8, element_name, "entry")) {
                        break;
                    } else {
                        return error.UnexpectedElementEnd;
                    }
                },
                else => {},
            }
        }

        return .{
            .name = name,
            .value = value,
            .description = description,
            .deprecation_note = deprecation_note,
            .wip_flag = wip_flag,
            .params = try p.arena.dupe(Data.Enum.Entry.Param, params.items),
        };
    }

    fn parse_enum_entry_param(p: *Parser, reader: *xml.Reader) !Data.Enum.Entry.Param {
        const index_idx = reader.attributeIndex("index") orelse return error.EnumEntryParamNoIndex;
        const index_str = try reader.attributeValueAlloc(p.arena, index_idx);
        const index = std.fmt.parseInt(usize, index_str, 10) catch return error.EnumEntryParamInvalidIndex;

        const label = if (reader.attributeIndex("label")) |label_idx|
            try reader.attributeValueAlloc(p.arena, label_idx)
        else
            null;

        const units = if (reader.attributeIndex("units")) |units_idx|
            try reader.attributeValueAlloc(p.arena, units_idx)
        else
            null;

        const description = switch (try reader.read()) {
            .text => try p.arena.dupe(u8, try reader.text()),
            else => "",
        };
        try reader.skipElement();

        std.log.debug("visiting enum entry param {} {?s}", .{
            index,
            label,
        });

        return .{
            .index = index,
            .label = label,
            .units = units,
            .description = description,
        };
    }

    fn visit_message(p: *Parser, reader: *xml.Reader) !void {
        const name_idx = reader.attributeIndex("name") orelse return error.MessageNoName;
        const name = try reader.attributeValueAlloc(p.arena, name_idx);

        const id_idx = reader.attributeIndex("id") orelse return error.MessageNoId;
        const id_str = try reader.attributeValue(id_idx);
        const id = std.fmt.parseInt(u24, id_str, 10) catch return error.MessageInvalidId;

        std.log.debug("visiting message {s} id = {}", .{ name, id });

        var description: []const u8 = "";
        var fields: std.ArrayListUnmanaged(Data.Message.Field) = .empty;
        defer fields.deinit(p.gpa);
        var extension_fields_start: ?usize = null;
        var deprecation_note: ?Data.DeprecationNote = null;
        var wip_flag = false;

        while (true) {
            const node = try reader.read();
            switch (node) {
                .eof => return error.UnexpectedEof,
                .element_start => {
                    const element_name = reader.elementName();
                    if (eql(u8, element_name, "field")) {
                        const field = try p.parse_message_field(reader);
                        try fields.append(p.gpa, field);
                    } else if (eql(u8, element_name, "description")) {
                        description = try p.parse_description(reader);
                    } else if (is_deprecated(element_name)) {
                        deprecation_note = try p.parse_deprecation_note(reader);
                    } else if (eql(u8, element_name, "extensions")) {
                        extension_fields_start = fields.items.len;
                        try reader.skipElement();
                    } else if (eql(u8, element_name, "wip")) {
                        wip_flag = true;
                        try reader.skipElement();
                    } else {
                        std.log.warn("unhandled element in message: {s}", .{element_name});
                        try reader.skipElement();
                    }
                },
                .element_end => {
                    const element_name = reader.elementName();
                    if (eql(u8, element_name, "message")) {
                        break;
                    } else {
                        return error.UnexpectedElementEnd;
                    }
                },
                else => {},
            }
        }

        try p.messages.put(p.gpa, name, .{
            .name = name,
            .id = id,
            .fields = try p.arena.dupe(Data.Message.Field, fields.items),
            .extension_fields_start = extension_fields_start orelse fields.items.len,
            .description = description,
            .deprecation_note = deprecation_note,
            .wip_flag = wip_flag,
        });
    }

    fn parse_message_field(p: *Parser, reader: *xml.Reader) !Data.Message.Field {
        const name_idx = reader.attributeIndex("name") orelse return error.MessageFieldNoName;
        const name = try reader.attributeValueAlloc(p.arena, name_idx);

        const type_idx = reader.attributeIndex("type") orelse return error.MessageFieldNoType;
        const type_str = try reader.attributeValueAlloc(p.arena, type_idx);
        const @"type" = try parse_type(type_str);

        const @"enum" = if (reader.attributeIndex("enum")) |enum_idx|
            try reader.attributeValueAlloc(p.arena, enum_idx)
        else
            null;

        std.log.debug("visiting message field {s}, type = {s}, enum = {?s}", .{
            name,
            type_str,
            @"enum",
        });

        const text = switch (try reader.read()) {
            .text => try p.arena.dupe(u8, try reader.text()),
            else => return error.ExpectedTextNode,
        };
        try reader.skipElement();

        return .{
            .name = name,
            .type = @"type",
            .@"enum" = @"enum",
            .description = text,
        };
    }

    fn parse_description(p: *Parser, reader: *xml.Reader) ![]const u8 {
        const text = switch (try reader.read()) {
            .text => try p.arena.dupe(u8, try reader.text()),
            else => "",
        };
        // std.log.debug("visiting description {s}", .{text});
        try reader.skipElement();
        return text;
    }

    fn parse_deprecation_note(p: *Parser, reader: *xml.Reader) !Data.DeprecationNote {
        const since_idx = reader.attributeIndex("since") orelse return error.InvalidDeprecationNote;
        const since = try reader.attributeValueAlloc(p.arena, since_idx);

        const replaced_by_idx = reader.attributeIndex("replaced_by") orelse return error.InvalidDeprecationNote;
        const replaced_by = try reader.attributeValueAlloc(p.arena, replaced_by_idx);

        const text = switch (try reader.read()) {
            .text => try p.arena.dupe(u8, try reader.text()),
            else => "",
        };
        try reader.skipElement();

        // std.log.debug("visiting deprecation note since = {s}, replaced_by = {s}, text = {s}", .{
        //     since,
        //     replaced_by,
        //     text,
        // });

        return .{
            .since = since,
            .replaced_by = replaced_by,
            .text = text,
        };
    }

    fn parse_type(text: []const u8) !Data.Type {
        var it = std.mem.splitScalar(u8, text, '[');
        const primitive_str = it.next() orelse return error.InvalidType;
        const primitive = Data.Type.Primitive.from_str(primitive_str) orelse {
            std.log.err("primitive type not recognized: {s}", .{primitive_str});
            return error.InvalidPrimitiveType;
        };
        if (it.next()) |array_part| {
            const count_str = std.mem.trimEnd(u8, array_part, &.{']'});
            const count = std.fmt.parseInt(u8, count_str, 10) catch return error.InvalidType;
            return .{
                .array = .{
                    .primitive = primitive,
                    .count = count,
                },
            };
        } else {
            return .{
                .primitive = primitive,
            };
        }
    }

    fn is_deprecated(element_name: []const u8) bool {
        return eql(u8, element_name, "deprecated") or eql(u8, element_name, "superseded");
    }
};
