//! LSM6DSV IMU driver
//! Ported from Betaflight's accgyro_spi_lsm6dsv16x.c, then reworked to
//! expose the real register bitfields (verified against
//! STMicroelectronics's official register map,
//! https://github.com/STMicroelectronics/lsm6dsv16x-pid/blob/main/lsm6dsv16x_reg.h)
//! instead of hardcoding one fixed ODR/filter/scale configuration.
//!
//! Register access follows the same shape as the MPU-6050 driver: a
//! `Register` enum, byte-level `raw_read`/`raw_write`, and generic
//! `read_reg`/`write_reg`/`modify_reg` helpers that bitcast to/from typed
//! packed-struct register layouts.
//!
//! Generic over any duck-typed SPI device exposing:
//!
//!   pub const Transaction = struct {
//!       receive: []u8 = &.{},
//!       send: []const u8 = &.{},
//!       pad: u8 = 0x00,
//!   };
//!   pub fn transceivev(self: Self, transactions: []const Transaction) !void
//!
//! `transceivev` is assumed to execute all transactions in the slice as a
//! single atomic SPI operation (one chip-select assertion), the way a
//! vectored `writev`/`readv` would. That's what lets a register read be
//! expressed as "send the address byte" followed by "receive the data",
//! rather than needing a combined send+receive buffer with a throwaway
//! address-echo byte at the front.
//!
//! Example usage (reproduces the old hardcoded behavior: High-Accuracy
//! mode, 1000 Hz, +-16 g / +-2000 dps):
//! ```zig
//! const Imu = lsm6dsv.Lsm6dsv16x(SPI_Device);
//! var imu = try Imu.init(spi_device, clock_device, .{});
//! const sample = try imu.read();
//! ```
//!
//! Picking your own ODR / full scale / filters:
//! ```zig
//! var imu = try Imu.init(spi_device, clock_device, .{
//!     .accel_fs = .g8,
//!     .gyro_fs = .dps1000,
//!     .accel_odr = 0x08, // see datasheet ODR table for your accel_mode/haodr_sel
//!     .gyro_odr = 0x08,
//! });
//! ```

const std = @import("std");

const math = @import("../../math.zig");

/// Builds an LSM6DSV driver generic over:
///   - `Spi`: a duck-typed SPI device type (see module docs for the
///     required shape of `Transaction` / `transceivev`).
///   - `who_am_i_reg`: expected value of the WHO_AM_I register for this
///     device variant.
///   - `accel_axes_reversed`: whether the accelerometer XYZ axis order is
///     reversed relative to the gyroscope in the burst-read register
///     layout (true for the LSM6DSV16B "accel axes reversed" variant).
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

            /// g per LSB, from the datasheet's typical sensitivity table.
            pub fn sensitivity_g(self: FsXl) f32 {
                return switch (self) {
                    .g2 => 0.000061,
                    .g4 => 0.000122,
                    .g8 => 0.000244,
                    .g16 => 0.000488,
                };
            }
        };

        /// Gyroscope full scale (FS_G, CTRL6 bits[3:0]). Not a contiguous
        /// range: 4000 dps lives at nibble 0xC, not 0x5.
        pub const FsG = enum(u4) {
            dps125 = 0x0,
            dps250 = 0x1,
            dps500 = 0x2,
            dps1000 = 0x3,
            dps2000 = 0x4,
            dps4000 = 0xC,

            /// dps per LSB, from the datasheet's typical sensitivity table.
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

        /// Accelerometer operating mode (OP_MODE_XL, CTRL1 bits[6:4]).
        /// Value 0b010 is reserved/undefined by the datasheet and
        /// deliberately has no name here.
        pub const XlMode = enum(u3) {
            high_performance = 0b000,
            high_accuracy_odr = 0b001,
            odr_triggered = 0b011,
            low_power_2_avg = 0b100,
            low_power_4_avg = 0b101,
            low_power_8_avg = 0b110,
            normal = 0b111,
        };

        /// Gyroscope operating mode (OP_MODE_G, CTRL2 bits[6:4]). Values
        /// 0b010, 0b011, 0b110, 0b111 are reserved/undefined by the
        /// datasheet and deliberately have no name here.
        pub const GMode = enum(u3) {
            high_performance = 0b000,
            high_accuracy_odr = 0b001,
            sleep = 0b100,
            low_power = 0b101,
        };

        /// Named bandwidth points for CTRL8.hp_lpf2_xl_bw. The other 6
        /// values of this 3-bit field select ODR-dependent filter stages;
        /// see datasheet Table 12 ("Accelerometer bandwidth selection")
        /// for their cutoff frequency at your chosen `accel_odr`.
        pub const accel_lpf2_bw_ultra_light: u3 = 0b000;
        pub const accel_lpf2_bw_strong: u3 = 0b110;

        /// Named bandwidth point for CTRL6.lpf1_g_bw. The other 6 values
        /// of this 3-bit field select ODR-dependent filter stages; see
        /// datasheet Table 9/10 ("Gyroscope filtering chain") for their
        /// cutoff frequency at your chosen `gyro_odr`.
        pub const gyro_lpf1_bw_ultra_light: u3 = 0b000;

        /// Runtime configuration for `init`. Defaults reproduce the
        /// original fixed behavior of this driver: High-Accuracy ODR mode
        /// 1 at 1000 Hz, +-16 g accel with Strong LPF2, +-2000 dps gyro
        /// with UltraLight LPF1, pulsed DRDY routed to INT1 for the gyro.
        pub const Config = struct {
            /// ODR_XL register nibble (CTRL1 bits[3:0]). Meaning depends
            /// on `accel_mode` and, when `accel_mode == .high_accuracy_odr`,
            /// on `haodr_sel` — see the datasheet's ODR selection tables.
            /// 0x9 is 1000 Hz in HAODR table 1 (`haodr_sel = 1`).
            accel_odr: u4 = 0x9,
            accel_mode: XlMode = .high_accuracy_odr,
            accel_fs: FsXl = .g16,
            accel_lpf2_bw: u3 = accel_lpf2_bw_strong,
            accel_lpf2_enable: bool = true,

            /// ODR_G register nibble (CTRL2 bits[3:0]); same caveats as
            /// `accel_odr`, but against `gyro_mode`.
            gyro_odr: u4 = 0x9,
            gyro_mode: GMode = .high_accuracy_odr,
            gyro_fs: FsG = .dps2000,
            gyro_lpf1_bw: u3 = gyro_lpf1_bw_ultra_light,
            gyro_lpf1_enable: bool = true,

            /// HAODR_SEL (HAODR_CFG bits[1:0]): selects which of the 3
            /// High-Accuracy ODR frequency tables `accel_odr`/`gyro_odr`
            /// are interpreted against. 0 leaves HAODR mode unselected
            /// (irrelevant unless `accel_mode`/`gyro_mode` is
            /// `.high_accuracy_odr`).
            haodr_sel: u2 = 1,

            drdy_pulsed: bool = true,
            route_gyro_drdy_to_int1: bool = true,
            route_accel_drdy_to_int1: bool = false,
            block_data_update: bool = true,

            /// Upper bound on how long to wait for the software reset to
            /// complete before `init` gives up with `error.Timeout`.
            reset_timeout_ms: u32 = 1000,
        };

        /// Initializes the sensor: verifies WHO_AM_I, performs a software
        /// reset, then applies `config`.
        ///
        /// `clock` is only used during initialization (to wait out the
        /// software reset) and must expose `sleep_ms(self, ms: u32) void`.
        pub fn init(spi: Spi, clock: anytype, config: Config) !Self {
            var self: Self = .{
                .spi = spi,
                .accel_fs = config.accel_fs,
                .gyro_fs = config.gyro_fs,
            };
            try self.verify();
            try self.reset(clock, config.reset_timeout_ms);
            try self.configure(config);
            return self;
        }

        /// Reads and checks the WHO_AM_I register against the expected
        /// value for this device variant.
        pub fn verify(self: *Self) !void {
            const whoami = try self.raw_read(.who_am_i);
            if (whoami != who_am_i_reg) {
                std.log.err("wrong device id: expected {}, got {}", .{ who_am_i_reg, whoami });
                return error.UnexpectedDeviceId;
            }
        }

        /// Triggers a software reset and blocks (via `clock`) until the
        /// device reports the reset has completed, or `timeout_ms` 1 ms
        /// ticks have elapsed.
        fn reset(self: *Self, clock: anytype, timeout_ms: u32) !void {
            try self.write_reg(.ctrl3, Ctrl3{ .sw_reset = true });

            var timeout = timeout_ms;
            while (true) {
                clock.sleep_ms(1);
                const v = try self.read_reg(.ctrl3, Ctrl3);
                if (!v.sw_reset) break;

                timeout -= 1;
                if (timeout == 0) return error.Timeout;
            }
        }

        fn configure(self: *Self, config: Config) !void {
            // Enable register auto-increment (always on) + Block Data
            // Update. BDU prevents reading mismatched high/low bytes
            // across a sample boundary.
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
                .hp_lpf2_xl_bw = config.accel_lpf2_bw,
            });

            try self.write_reg(.ctrl6, Ctrl6{
                .fs_g = config.gyro_fs,
                .lpf1_g_bw = config.gyro_lpf1_bw,
            });

            try self.write_reg(.ctrl7, Ctrl7{
                .lpf1_g_en = config.gyro_lpf1_enable,
            });

            try self.write_reg(.ctrl9, Ctrl9{
                .lpf2_xl_en = config.accel_lpf2_enable,
            });

            try self.write_reg(.ctrl1, Ctrl1{
                .odr_xl = config.accel_odr,
                .op_mode_xl = config.accel_mode,
            });

            try self.write_reg(.ctrl2, Ctrl2{
                .odr_g = config.gyro_odr,
                .op_mode_g = config.gyro_mode,
            });

            // DRDY interrupt pulse behavior (clears automatically when
            // pulsed, no register read needed to unlatch).
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

        /// Reads one gyro+accel sample, scaled according to the full
        /// scale this instance was configured with.
        pub fn read(self: *Self) !Data {
            // 12 data bytes: gyro XYZ then accel XYZ (accel byte order
            // depends on the device variant).
            var buf: [13]u8 = @splat(0);
            buf[0] = @backingInt(Register.outx_l_g) | SPI_READ;
            try self.spi.transceive(&buf);

            // buf[0..2] = OUTX_L_G / OUTX_H_G -> gyro X
            // buf[2..4] = OUTY_L_G / OUTY_H_G -> gyro Y
            // buf[4..6] = OUTZ_L_G / OUTZ_H_G -> gyro Z
            const gx = std.mem.readInt(i16, buf[1..3], .little);
            const gy = std.mem.readInt(i16, buf[3..5], .little);
            const gz = std.mem.readInt(i16, buf[5..7], .little);

            const accel_raw: [3]i16 = if (accel_axes_reversed)
                .{
                    // buf[6..8]   = OUTX_L_A / OUTX_H_A -> accel X
                    // buf[8..10]  = OUTY_L_A / OUTY_H_A -> accel Y
                    // buf[10..12] = OUTZ_L_A / OUTZ_H_A -> accel Z
                    std.mem.readInt(i16, buf[7..9], .little),
                    std.mem.readInt(i16, buf[9..11], .little),
                    std.mem.readInt(i16, buf[11..13], .little),
                }
            else
                .{
                    // buf[6..8]   = OUTZ_L_A / OUTZ_H_A -> accel Z
                    // buf[8..10]  = OUTY_L_A / OUTY_H_A -> accel Y
                    // buf[10..12] = OUTX_L_A / OUTX_H_A -> accel X
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

        /// Single-byte register write, MSB (read/write bit) forced low.
        fn raw_write(self: *Self, reg: Register, value: u8) !void {
            var buf: [2]u8 = .{ @backingInt(reg) & ~SPI_READ, value };
            try self.spi.transceive(&buf);
        }

        /// Single-byte register read.
        fn raw_read(self: *Self, reg: Register) !u8 {
            var value: [2]u8 = .{ @backingInt(reg) | SPI_READ, 0 };
            try self.spi.transceive(&value);
            return value[1];
        }

        /// Reads a register and bitcasts it into the packed-struct layout
        /// `T` (one of the `Ctrl*`/`HaodrCfg`/`Int1Ctrl` types below).
        fn read_reg(self: *Self, reg: Register, T: type) !T {
            return @bitCast(try self.raw_read(reg));
        }

        /// Bitcasts a packed-struct register value down to a byte and
        /// writes it.
        fn write_reg(self: *Self, reg: Register, value: anytype) !void {
            try self.raw_write(reg, @bitCast(value));
        }

        /// Read-modify-write: reads the current value of `reg` as `T`,
        /// overwrites the named fields from `fields`, and writes the
        /// result back.
        fn modify_reg(self: *Self, reg: Register, T: type, fields: anytype) !void {
            const current_val = try self.read_reg(reg, T);

            var val: T = current_val;
            inline for (@typeInfo(@TypeOf(fields)).@"struct".field_names) |field_name| {
                @field(val, field_name) = @field(fields, field_name);
            }

            try self.write_reg(reg, val);
        }

        // -- Register bitfield layouts -----------------------------------
        // Verified bit-for-bit against STMicroelectronics's official
        // lsm6dsv16x_reg.h (DRV_LITTLE_ENDIAN branch; field declaration
        // order == bit 0 upward, matching Zig packed struct layout).

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

/// LSM6DSV / LSM6DSV16X (WHO_AM_I = 0x70, standard accel axis order).
pub fn Lsm6dsv(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x71, false);
}
pub fn Lsm6dsv16x(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x70, false);
}
/// LSM6DSV16B variant with reversed accelerometer axis order in the
/// burst-read register layout (WHO_AM_I = 0x71).
pub fn Lsm6dsv16bAccelAxesRev(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x71, true);
}
/// LSM6DSV16B, standard accel axis order (WHO_AM_I = 0x71).
pub fn Lsm6dsv16b(comptime Spi: type) type {
    return Lsm6dsvGeneric(Spi, 0x71, false);
}

const SPI_READ: u8 = 0x80;
