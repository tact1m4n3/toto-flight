const std = @import("std");

/// A0 and A1 tied to ground.
pub const DEFAULT_SLAVE_ADDRESS: u8 = 0x40;

pub const MANUFACTURER_ID: u16 = 0x5449;
pub const DIE_ID: u16 = 0x2260;

pub const SHUNT_VOLTAGE_LSB: f32 = 2.5e-6;
pub const BUS_VOLTAGE_LSB: f32 = 1.25e-3;
pub const POWER_LSB_FACTOR: f32 = 25;

/// Cal = CAL_COEFFICIENT / (current_lsb * shunt_resistance)
pub const CAL_COEFFICIENT: f32 = 0.00512;

pub fn INA_226(comptime I2C: type) type {
    return struct {
        const Self = @This();

        i2c: I2C,
        address: u8,
        current_lsb: f32,
        power_lsb: f32,

        pub const VerifyError = error{ DeviceNotResponding, UnexpectedDeviceId };

        pub const Config = struct {
            averaging: Averaging = .x1,
            bus_conversion_time: ConversionTime = .@"1.1ms",
            shunt_conversion_time: ConversionTime = .@"1.1ms",
            mode: Mode = .continuous_shunt_and_bus,

            /// Shunt resistor value in ohms.
            shunt_resistance: f32,

            /// Current scaling in amperes per LSB of the current register.
            /// The datasheet recommends max expected current / 32768,
            /// optionally scaled down so the calibration value fits the
            /// register better.
            current_lsb: f32,
        };

        /// Verifies the device identity, resets it, then programs the
        /// configuration and calibration registers.
        ///
        /// Fails with `DeviceNotResponding` or `UnexpectedDeviceId` if the
        /// device is not an INA226 at `address`, and with
        /// `InvalidCalibration` if no calibration value fits the register
        /// for the given `current_lsb` and `shunt_resistance` (pick a
        /// larger current LSB).
        pub fn init(i2c: I2C, address: u8, clock: anytype, config: Config) !Self {
            var self: Self = .{
                .i2c = i2c,
                .address = address,
                .current_lsb = config.current_lsb,
                .power_lsb = POWER_LSB_FACTOR * config.current_lsb,
            };

            try self.verify();

            try self.reset(clock);

            try self.modify_reg(.config, regs.CONFIG, .{
                .mode = config.mode,
                .shunt_conversion_time = config.shunt_conversion_time,
                .bus_conversion_time = config.bus_conversion_time,
                .averaging = config.averaging,
            });

            const cal = CAL_COEFFICIENT / (config.current_lsb * config.shunt_resistance);
            if (!std.math.isFinite(cal) or cal < 1 or cal > 65535) {
                return error.InvalidCalibration;
            }

            try self.write_word(.calibration, @intFromFloat(cal));

            return self;
        }

        pub fn verify(self: Self) VerifyError!void {
            const manufacturer_id = self.read_word(.manufacturer_id) catch {
                return VerifyError.DeviceNotResponding;
            };

            const die_id = self.read_word(.die_id) catch {
                return VerifyError.DeviceNotResponding;
            };

            if (manufacturer_id != MANUFACTURER_ID or die_id != DIE_ID) {
                return VerifyError.UnexpectedDeviceId;
            }
        }

        /// Resets the device. All registers return to their power-on
        /// defaults.
        pub fn reset(self: Self, clock: anytype) !void {
            try self.modify_reg(.config, regs.CONFIG, .{
                .rst = true,
            });

            // The reset bit is self-clearing, so there is nothing to poll.
            clock.sleep_ms(10);
        }

        pub fn set_mode(self: Self, mode: Mode) !void {
            try self.modify_reg(.config, regs.CONFIG, .{
                .mode = mode,
            });
        }

        pub fn set_averaging(self: Self, averaging: Averaging) !void {
            try self.modify_reg(.config, regs.CONFIG, .{
                .averaging = averaging,
            });
        }

        pub fn set_bus_conversion_time(self: Self, t: ConversionTime) !void {
            try self.modify_reg(.config, regs.CONFIG, .{
                .bus_conversion_time = t,
            });
        }

        pub fn set_shunt_conversion_time(self: Self, t: ConversionTime) !void {
            try self.modify_reg(.config, regs.CONFIG, .{
                .shunt_conversion_time = t,
            });
        }

        pub fn read_shunt_voltage_raw(self: Self) !i16 {
            return @bitCast(try self.read_word(.shunt_voltage));
        }

        pub fn read_shunt_voltage(self: Self) !f32 {
            return @as(f32, @floatFromInt(try self.read_shunt_voltage_raw())) * SHUNT_VOLTAGE_LSB;
        }

        pub fn read_bus_voltage_raw(self: Self) !u16 {
            return try self.read_word(.bus_voltage);
        }

        pub fn read_bus_voltage(self: Self) !f32 {
            return @as(f32, @floatFromInt(try self.read_bus_voltage_raw())) * BUS_VOLTAGE_LSB;
        }

        pub fn read_current_raw(self: Self) !i16 {
            return @bitCast(try self.read_word(.current));
        }

        pub fn read_current(self: Self) !f32 {
            return @as(f32, @floatFromInt(try self.read_current_raw())) * self.current_lsb;
        }

        pub fn read_power_raw(self: Self) !u16 {
            return try self.read_word(.power);
        }

        pub fn read_power(self: Self) !f32 {
            return @as(f32, @floatFromInt(try self.read_power_raw())) * self.power_lsb;
        }

        /// Burst-reads all four data registers in one transaction.
        pub fn read_raw(self: Self) !DataRaw {
            var buf: [8]u8 = undefined;
            try self.i2c.write_than_read(self.address, &.{@backingInt(Register.shunt_voltage)}, &buf);
            return .{
                .shunt_voltage = std.mem.readInt(i16, buf[0..2], .big),
                .bus_voltage = std.mem.readInt(u16, buf[2..4], .big),
                .power = std.mem.readInt(u16, buf[4..6], .big),
                .current = std.mem.readInt(i16, buf[6..8], .big),
            };
        }

        /// Burst-reads all four data registers in one transaction, scaled
        /// according to the calibration this instance was configured with.
        pub fn read(self: Self) !Data {
            const raw = try self.read_raw();
            return .{
                .shunt_voltage = @as(f32, @floatFromInt(raw.shunt_voltage)) * SHUNT_VOLTAGE_LSB,
                .bus_voltage = @as(f32, @floatFromInt(raw.bus_voltage)) * BUS_VOLTAGE_LSB,
                .power = @as(f32, @floatFromInt(raw.power)) * self.power_lsb,
                .current = @as(f32, @floatFromInt(raw.current)) * self.current_lsb,
            };
        }

        pub const DataRaw = struct {
            shunt_voltage: i16,
            bus_voltage: u16,
            power: u16,
            current: i16,
        };

        pub const Data = struct {
            /// V
            shunt_voltage: f32,
            /// V
            bus_voltage: f32,
            /// W
            power: f32,
            /// A
            current: f32,
        };

        /// Reads the Mask/Enable register. Note that reading this register
        /// clears the alert function flag when the alert is latched.
        pub fn read_mask_enable(self: Self) !regs.MASK_ENABLE {
            return self.read_reg(.mask_enable, regs.MASK_ENABLE);
        }

        /// Set after all conversions, averaging and multiplication are
        /// complete. Useful for coordinating one-shot or triggered modes.
        pub fn conversion_ready(self: Self) !bool {
            return (try self.read_mask_enable()).conversion_ready_flag;
        }

        /// Selects which alert function is compared against the alert
        /// limit and asserted on the alert pin.
        pub fn set_alert_function(self: Self, f: AlertFunction) !void {
            try self.modify_reg(.mask_enable, regs.MASK_ENABLE, .{
                .alert_function = f,
            });
        }

        pub fn set_alert_limit_raw(self: Self, limit: u16) !void {
            try self.write_word(.alert_limit, limit);
        }

        /// The alert pin is open-drain: active-low (asserted low) by
        /// default.
        pub fn set_alert_polarity(self: Self, active_high: bool) !void {
            try self.modify_reg(.mask_enable, regs.MASK_ENABLE, .{
                .alert_active_high = active_high,
            });
        }

        /// When latched, the alert pin and flag stay asserted until the
        /// mask register is read; otherwise they clear on the next
        /// conversion that is not over the limit.
        pub fn set_alert_latch_enabled(self: Self, enabled: bool) !void {
            try self.modify_reg(.mask_enable, regs.MASK_ENABLE, .{
                .latch_enabled = enabled,
            });
        }

        /// Asserts the alert pin when a conversion completes, alongside
        /// the selected alert function.
        pub fn set_alert_conversion_ready(self: Self, enabled: bool) !void {
            try self.modify_reg(.mask_enable, regs.MASK_ENABLE, .{
                .conversion_ready_alert = enabled,
            });
        }

        pub const Mode = enum(u3) {
            triggered_shunt_and_bus = 0b000,
            triggered_shunt = 0b001,
            triggered_bus = 0b010,
            // 0b011 duplicates .triggered_shunt_and_bus
            power_down = 0b100,
            continuous_shunt = 0b101,
            continuous_bus = 0b110,
            continuous_shunt_and_bus = 0b111,
        };

        pub const ConversionTime = enum(u3) {
            @"140us" = 0,
            @"204us" = 1,
            @"332us" = 2,
            @"588us" = 3,
            @"1.1ms" = 4,
            @"2.116ms" = 5,
            @"4.156ms" = 6,
            @"8.244ms" = 7,
        };

        pub const Averaging = enum(u3) {
            x1 = 0,
            x4 = 1,
            x16 = 2,
            x64 = 3,
            x128 = 4,
            x256 = 5,
            x512 = 6,
            x1024 = 7,
        };

        /// One-hot selection of the alert function (CB4-CB0).
        pub const AlertFunction = enum(u5) {
            none = 0b00000,
            shunt_over_voltage = 0b10000,
            shunt_under_voltage = 0b01000,
            bus_over_voltage = 0b00100,
            bus_under_voltage = 0b00010,
            power_over_limit = 0b00001,
        };

        fn write_word(self: Self, reg: Register, value: u16) !void {
            var buf: [3]u8 = undefined;
            buf[0] = @backingInt(reg);
            std.mem.writeInt(u16, buf[1..], value, .big);
            try self.i2c.write(self.address, &buf);
        }

        fn read_word(self: Self, reg: Register) !u16 {
            var buf: [2]u8 = undefined;
            try self.i2c.write_than_read(self.address, &.{@backingInt(reg)}, &buf);
            return std.mem.readInt(u16, &buf, .big);
        }

        inline fn read_reg(self: Self, reg: Register, T: type) !T {
            return @bitCast(try self.read_word(reg));
        }

        inline fn write_reg(self: Self, reg: Register, value: anytype) !void {
            try self.write_word(reg, @bitCast(value));
        }

        inline fn modify_reg(self: Self, reg: Register, T: type, fields: anytype) !void {
            const current_val = try self.read_reg(reg, T);

            var val: T = current_val;
            inline for (@typeInfo(@TypeOf(fields)).@"struct".field_names) |field_name| {
                @field(val, field_name) = @field(fields, field_name);
            }

            try self.write_reg(reg, val);
        }

        pub const Register = enum(u8) {
            config = 0x00,
            shunt_voltage = 0x01,
            bus_voltage = 0x02,
            power = 0x03,
            current = 0x04,
            calibration = 0x05,
            mask_enable = 0x06,
            alert_limit = 0x07,
            manufacturer_id = 0xFE,
            die_id = 0xFF,
            _,
        };

        pub const regs = struct {
            pub const CONFIG = packed struct(u16) {
                mode: Mode = .continuous_shunt_and_bus,
                shunt_conversion_time: ConversionTime = .@"1.1ms",
                bus_conversion_time: ConversionTime = .@"1.1ms",
                averaging: Averaging = .x1,
                reserved: u3 = 0,
                rst: bool = false,
            };

            pub const MASK_ENABLE = packed struct(u16) {
                latch_enabled: bool = false,
                alert_active_high: bool = false,
                overflow_flag: bool = false,
                conversion_ready_flag: bool = false,
                alert_function_flag: bool = false,
                reserved: u5 = 0,
                conversion_ready_alert: bool = false,
                alert_function: AlertFunction = .none,
            };
        };
    };
}
