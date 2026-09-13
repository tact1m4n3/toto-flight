const std = @import("std");

const math = @import("../../math.zig");

pub fn Lsm6dsvGeneric(
    comptime Spi: type,
    comptime who_am_i_reg: u8,
    comptime accel_axes_reversed: bool,
) type {
    return struct {
        const Self = @This();

        spi: Spi,
        accel_fs: FsXl,
        gyro_fs: FsG,

        pub const Data = struct {
            gyro: math.Vec3,
            accel: math.Vec3,
        };

        /// Accelerometer full scale (FS_XL, CTRL8 bits[1:0]).
        pub const FsXl = enum(u2) {
            g2 = 0b00,
            g4 = 0b01,
            g8 = 0b10,
            g16 = 0b11,

            /// g per LSB.
            pub fn sensitivity_g(self: FsXl) f32 {
                return switch (self) {
                    .g2 => 0.000061,
                    .g4 => 0.000122,
                    .g8 => 0.000244,
                    .g16 => 0.000488,
                };
            }
        };

        /// Gyroscope full scale (FS_G, CTRL6 bits[3:0]). Note 4000 dps
        /// lives at 0xC, not 0x5.
        pub const FsG = enum(u4) {
            dps125 = 0x0,
            dps250 = 0x1,
            dps500 = 0x2,
            dps1000 = 0x3,
            dps2000 = 0x4,
            dps4000 = 0xC,

            /// dps per LSB.
            pub fn sensitivity_dps(self: FsG) f32 {
                return switch (self) {
                    .dps125 => 0.004375,
                    .dps250 => 0.00875,
                    .dps500 => 0.0175,
                    .dps1000 => 0.035,
                    .dps2000 => 0.070,
                    .dps4000 => 0.140,
                };
            }
        };

        /// Accelerometer operating mode (OP_MODE_XL, CTRL1 bits[6:4]);
        /// 0b010 is reserved.
        pub const XlMode = enum(u3) {
            high_performance = 0b000,
            high_accuracy_odr = 0b001,
            odr_triggered = 0b011,
            low_power_2_avg = 0b100,
            low_power_4_avg = 0b101,
            low_power_8_avg = 0b110,
            normal = 0b111,
        };

        /// Gyroscope operating mode (OP_MODE_G, CTRL2 bits[6:4]);
        /// 0b010, 0b011, 0b110, 0b111 are reserved.
        pub const GMode = enum(u3) {
            high_performance = 0b000,
            high_accuracy_odr = 0b001,
            sleep = 0b100,
            low_power = 0b101,
        };

        /// Output data rate, named after its nominal frequency; accel
        /// and gyro share the code tables. Which table applies depends
        /// on the sensor operating mode: the standard table, or one of
        /// two high-accuracy tables (+-1% ODR variation with gyro on,
        /// +-3% without) picked by `haodr_sel`. If one sensor is in
        /// `.high_accuracy_odr`, the datasheet requires both to be.
        pub const Odr = enum(u8) {
            off,
            hz1_875,
            hz7_5,
            hz12_5,
            hz15,
            hz15_625,
            hz25,
            hz30,
            hz31_25,
            hz50,
            hz60,
            hz62_5,
            hz100,
            hz120,
            hz125,
            hz200,
            hz240,
            hz250,
            hz400,
            hz480,
            hz500,
            hz800,
            hz960,
            hz1000,
            hz1600,
            hz1920,
            hz2000,
            hz3200,
            hz3840,
            hz4000,
            hz6400,
            hz7680,
            hz8000,

            const Table = enum { std, ha1, ha2 };

            /// Register nibble (CTRL1/CTRL2 bits[3:0]) for `table`;
            /// errors if the rate is not part of that table.
            fn nibble(self: Odr, table: Table) !u4 {
                return switch (table) {
                    .std => switch (self) {
                        .off => 0x0,
                        .hz1_875 => 0x1,
                        .hz7_5 => 0x2,
                        .hz15 => 0x3,
                        .hz30 => 0x4,
                        .hz60 => 0x5,
                        .hz120 => 0x6,
                        .hz240 => 0x7,
                        .hz480 => 0x8,
                        .hz960 => 0x9,
                        .hz1920 => 0xA,
                        .hz3840 => 0xB,
                        .hz7680 => 0xC,
                        else => error.InvalidOdr,
                    },
                    .ha1 => switch (self) {
                        .off => 0x0,
                        .hz15_625 => 0x3,
                        .hz31_25 => 0x4,
                        .hz62_5 => 0x5,
                        .hz125 => 0x6,
                        .hz250 => 0x7,
                        .hz500 => 0x8,
                        .hz1000 => 0x9,
                        .hz2000 => 0xA,
                        .hz4000 => 0xB,
                        .hz8000 => 0xC,
                        else => error.InvalidOdr,
                    },
                    .ha2 => switch (self) {
                        .off => 0x0,
                        .hz12_5 => 0x3,
                        .hz25 => 0x4,
                        .hz50 => 0x5,
                        .hz100 => 0x6,
                        .hz200 => 0x7,
                        .hz400 => 0x8,
                        .hz800 => 0x9,
                        .hz1600 => 0xA,
                        .hz3200 => 0xB,
                        .hz6400 => 0xC,
                        else => error.InvalidOdr,
                    },
                };
            }
        };

        /// Accelerometer LPF2 bandwidth (CTRL8 bits[7:5]). The named
        /// values are ODR-independent; other encodings pick an
        /// ODR-dependent filter stage (datasheet Table 12).
        pub const XlLpf2Bw = union(enum) {
            ultra_light,
            strong,
            odr_dependent: u3,

            fn encode(self: XlLpf2Bw) u3 {
                return switch (self) {
                    .ultra_light => 0b000,
                    .strong => 0b110,
                    .odr_dependent => |bw| bw,
                };
            }
        };

        /// Gyroscope LPF1 bandwidth (CTRL6 bits[6:4]). The named value
        /// is ODR-independent; other encodings pick an ODR-dependent
        /// filter stage (datasheet Table 9/10).
        pub const GLpf1Bw = union(enum) {
            ultra_light,
            odr_dependent: u3,

            fn encode(self: GLpf1Bw) u3 {
                return switch (self) {
                    .ultra_light => 0b000,
                    .odr_dependent => |bw| bw,
                };
            }
        };

        /// Defaults: high-accuracy ODR mode 1 at 1000 Hz, +-16 g accel
        /// with Strong LPF2, +-2000 dps gyro with UltraLight LPF1,
        /// pulsed gyro DRDY on INT1.
        pub const Config = struct {
            accel_odr: Odr = .hz1000,
            accel_mode: XlMode = .high_accuracy_odr,
            accel_fs: FsXl = .g16,
            accel_lpf2_bw: XlLpf2Bw = .strong,
            accel_lpf2_enable: bool = true,

            gyro_odr: Odr = .hz1000,
            gyro_mode: GMode = .high_accuracy_odr,
            gyro_fs: FsG = .dps2000,
            gyro_lpf1_bw: GLpf1Bw = .ultra_light,
            gyro_lpf1_enable: bool = true,

            /// High-accuracy ODR table (HAODR_CFG bits[1:0]): 1 or 2
            /// when a sensor is in `.high_accuracy_odr`, else ignored
            /// (0 selects the standard table).
            haodr_sel: u2 = 1,

            drdy_pulsed: bool = true,
            route_gyro_drdy_to_int1: bool = true,
            route_accel_drdy_to_int1: bool = false,
            block_data_update: bool = true,
        };

        const SPI_READ: u8 = 0x80;

        /// Verifies the device, software-resets it, then applies
        /// `config`. `clock` must expose `sleep_ms(self, ms: u32) void`
        /// and is only used to wait out the reset.
        pub fn init(spi: Spi, clock: anytype, config: Config) !Self {
            var self: Self = .{
                .spi = spi,
                .accel_fs = config.accel_fs,
                .gyro_fs = config.gyro_fs,
            };
            try self.verify();
            try self.reset(clock);
            try self.configure(config);
            return self;
        }

        pub fn verify(self: *Self) !void {
            const whoami = try self.raw_read(.who_am_i);
            if (whoami != who_am_i_reg) {
                std.log.err("wrong device id: expected {}, got {}", .{ who_am_i_reg, whoami });
                return error.UnexpectedDeviceId;
            }
        }

        /// Software reset, blocking until the device reports
        /// completion.
        fn reset(self: *Self, clock: anytype) !void {
            try self.write_reg(.ctrl3, Ctrl3{ .sw_reset = true });

            for (0..10) |_| {
                clock.sleep_ms(10);
                const v = try self.read_reg(.ctrl3, Ctrl3);
                if (!v.sw_reset) break;
            } else return error.Timeout;
        }

        fn odr_table(is_haodr: bool, haodr_sel: u2) !Odr.Table {
            if (!is_haodr or haodr_sel == 0) return .std;
            return switch (haodr_sel) {
                1 => .ha1,
                2 => .ha2,
                else => error.InvalidHaodrSel,
            };
        }

        fn configure(self: *Self, config: Config) !void {
            // Auto-increment + BDU (prevents torn high/low bytes across
            // a sample boundary).
            try self.write_reg(.ctrl3, Ctrl3{
                .if_inc = true,
                .bdu = config.block_data_update,
            });

            if (config.haodr_sel != 0) {
                try self.write_reg(.haodr_cfg, HaodrCfg{
                    .haodr_sel = config.haodr_sel,
                });
            }

            try self.write_reg(.ctrl8, Ctrl8{
                .fs_xl = config.accel_fs,
                .hp_lpf2_xl_bw = config.accel_lpf2_bw.encode(),
            });

            try self.write_reg(.ctrl6, Ctrl6{
                .fs_g = config.gyro_fs,
                .lpf1_g_bw = config.gyro_lpf1_bw.encode(),
            });

            try self.write_reg(.ctrl7, Ctrl7{
                .lpf1_g_en = config.gyro_lpf1_enable,
            });

            try self.write_reg(.ctrl9, Ctrl9{
                .lpf2_xl_en = config.accel_lpf2_enable,
            });

            const xl_table = try odr_table(config.accel_mode == .high_accuracy_odr, config.haodr_sel);
            const g_table = try odr_table(config.gyro_mode == .high_accuracy_odr, config.haodr_sel);

            try self.write_reg(.ctrl1, Ctrl1{
                .odr_xl = try config.accel_odr.nibble(xl_table),
                .op_mode_xl = config.accel_mode,
            });

            try self.write_reg(.ctrl2, Ctrl2{
                .odr_g = try config.gyro_odr.nibble(g_table),
                .op_mode_g = config.gyro_mode,
            });

            try self.write_reg(.ctrl4, Ctrl4{
                .drdy_pulsed = config.drdy_pulsed,
            });

            try self.write_reg(.int1_ctrl, Int1Ctrl{
                .int1_drdy_xl = config.route_accel_drdy_to_int1,
                .int1_drdy_g = config.route_gyro_drdy_to_int1,
            });

            self.accel_fs = config.accel_fs;
            self.gyro_fs = config.gyro_fs;
        }

        /// One gyro+accel sample, scaled by the configured full scale.
        pub fn read(self: *Self) !Data {
            // 12 data bytes: gyro XYZ then accel XYZ; accel axis order
            // depends on the device variant.
            var buf: [13]u8 = @splat(0);
            buf[0] = @backingInt(Register.outx_l_g) | SPI_READ;
            try self.spi.transceive(&buf);

            const gx = std.mem.readInt(i16, buf[1..3], .little);
            const gy = std.mem.readInt(i16, buf[3..5], .little);
            const gz = std.mem.readInt(i16, buf[5..7], .little);

            const accel_raw: [3]i16 = if (accel_axes_reversed)
                .{
                    std.mem.readInt(i16, buf[7..9], .little),
                    std.mem.readInt(i16, buf[9..11], .little),
                    std.mem.readInt(i16, buf[11..13], .little),
                }
            else
                .{
                    std.mem.readInt(i16, buf[11..13], .little),
                    std.mem.readInt(i16, buf[9..11], .little),
                    std.mem.readInt(i16, buf[7..9], .little),
                };

            const gyro_scale = self.gyro_fs.sensitivity_dps();
            const accel_scale = self.accel_fs.sensitivity_g();

            return .{
                .gyro = .{
                    .x = @as(f32, @floatFromInt(gx)) * gyro_scale,
                    .y = @as(f32, @floatFromInt(gy)) * gyro_scale,
                    .z = @as(f32, @floatFromInt(gz)) * gyro_scale,
                },
                .accel = .{
                    .x = @as(f32, @floatFromInt(accel_raw[0])) * accel_scale,
                    .y = @as(f32, @floatFromInt(accel_raw[1])) * accel_scale,
                    .z = @as(f32, @floatFromInt(accel_raw[2])) * accel_scale,
                },
            };
        }

        fn raw_write(self: *Self, reg: Register, value: u8) !void {
            var buf: [2]u8 = .{ @backingInt(reg) & ~SPI_READ, value };
            try self.spi.transceive(&buf);
        }

        fn raw_read(self: *Self, reg: Register) !u8 {
            var value: [2]u8 = .{ @backingInt(reg) | SPI_READ, 0 };
            try self.spi.transceive(&value);
            return value[1];
        }

        fn read_reg(self: *Self, reg: Register, T: type) !T {
            return @bitCast(try self.raw_read(reg));
        }

        fn write_reg(self: *Self, reg: Register, value: anytype) !void {
            try self.raw_write(reg, @bitCast(value));
        }

        /// Read-modify-write named fields of `reg`.
        fn modify_reg(self: *Self, reg: Register, T: type, fields: anytype) !void {
            const current_val = try self.read_reg(reg, T);

            var val: T = current_val;
            inline for (@typeInfo(@TypeOf(fields)).@"struct".field_names) |field_name| {
                @field(val, field_name) = @field(fields, field_name);
            }

            try self.write_reg(reg, val);
        }

        // Bitfield layouts verified against ST's lsm6dsv16x_reg.h
        // (little-endian branch).

        pub const Register = enum(u8) {
            who_am_i = 0x0F,
            int1_ctrl = 0x0D,
            ctrl1 = 0x10,
            ctrl2 = 0x11,
            ctrl3 = 0x12,
            ctrl4 = 0x13,
            ctrl6 = 0x15,
            ctrl7 = 0x16,
            ctrl8 = 0x17,
            ctrl9 = 0x18,
            haodr_cfg = 0x62,
            outx_l_g = 0x22,
            _,
        };

        pub const Ctrl1 = packed struct(u8) {
            odr_xl: u4 = 0,
            op_mode_xl: XlMode = .high_performance,
            not_used0: u1 = 0,
        };

        pub const Ctrl2 = packed struct(u8) {
            odr_g: u4 = 0,
            op_mode_g: GMode = .high_performance,
            not_used0: u1 = 0,
        };

        pub const Ctrl3 = packed struct(u8) {
            sw_reset: bool = false,
            not_used0: u1 = 0,
            if_inc: bool = false,
            not_used1: u3 = 0,
            bdu: bool = false,
            boot: bool = false,
        };

        pub const Ctrl4 = packed struct(u8) {
            int2_in_lh: bool = false,
            drdy_pulsed: bool = false,
            int2_drdy_temp: bool = false,
            drdy_mask: bool = false,
            int2_on_int1: bool = false,
            not_used0: u3 = 0,
        };

        pub const Ctrl6 = packed struct(u8) {
            fs_g: FsG = .dps125,
            lpf1_g_bw: u3 = 0,
            not_used0: u1 = 0,
        };

        pub const Ctrl7 = packed struct(u8) {
            lpf1_g_en: bool = false,
            not_used0: u3 = 0,
            ah_qvar_c_zin: u2 = 0,
            int2_drdy_ah_qvar: bool = false,
            ah_qvar_en: bool = false,
        };

        pub const Ctrl8 = packed struct(u8) {
            fs_xl: FsXl = .g2,
            not_used0: u1 = 0,
            xl_dualc_en: bool = false,
            not_used1: u1 = 0,
            hp_lpf2_xl_bw: u3 = 0,
        };

        pub const Ctrl9 = packed struct(u8) {
            usr_off_on_out: bool = false,
            usr_off_w: bool = false,
            not_used0: u1 = 0,
            lpf2_xl_en: bool = false,
            hp_slope_xl_en: bool = false,
            xl_fastsettl_mode: bool = false,
            hp_ref_mode_xl: bool = false,
            not_used1: u1 = 0,
        };

        pub const HaodrCfg = packed struct(u8) {
            haodr_sel: u2 = 0,
            not_used0: u6 = 0,
        };

        pub const Int1Ctrl = packed struct(u8) {
            int1_drdy_xl: bool = false,
            int1_drdy_g: bool = false,
            not_used0: u1 = 0,
            int1_fifo_th: bool = false,
            int1_fifo_ovr: bool = false,
            int1_fifo_full: bool = false,
            int1_cnt_bdr: bool = false,
            not_used1: u1 = 0,
        };
    };
}

pub fn Lsm6dsv(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x71, false);
}
pub fn Lsm6dsv16x(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x70, false);
}
pub fn Lsm6dsv16bAccelAxesRev(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x71, true);
}
pub fn Lsm6dsv16b(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x71, false);
}

test "odr nibble maps per table" {
    const D = Lsm6dsvGeneric(u8, 0x70, false);

    const off: D.Odr = .off;
    const hz960: D.Odr = .hz960;
    const hz1000: D.Odr = .hz1000;
    const hz800: D.Odr = .hz800;
    try std.testing.expectEqual(@as(u4, 0x0), off.nibble(.ha1));
    try std.testing.expectEqual(@as(u4, 0x9), hz960.nibble(.std));
    try std.testing.expectEqual(@as(u4, 0x9), hz1000.nibble(.ha1));
    try std.testing.expectEqual(@as(u4, 0x9), hz800.nibble(.ha2));
    try std.testing.expectError(error.InvalidOdr, hz1000.nibble(.std));
}

test "lpf bandwidth encode" {
    const D = Lsm6dsvGeneric(u8, 0x70, false);

    const ultra: D.XlLpf2Bw = .ultra_light;
    const strong: D.XlLpf2Bw = .strong;
    const raw: D.XlLpf2Bw = .{ .odr_dependent = 0b011 };
    try std.testing.expectEqual(@as(u3, 0b000), ultra.encode());
    try std.testing.expectEqual(@as(u3, 0b110), strong.encode());
    try std.testing.expectEqual(@as(u3, 0b011), raw.encode());

    const gyro_ultra: D.GLpf1Bw = .ultra_light;
    try std.testing.expectEqual(@as(u3, 0b000), gyro_ultra.encode());
}
