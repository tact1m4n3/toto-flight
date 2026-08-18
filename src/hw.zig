const options = @import("options");

const time = @import("time.zig");

pub const is_sim = options.board == .simulator;

pub const chip = switch (options.board) {
    .madflight_rp2350 => @import("chips/rp2350.zig"),
    .simulator => struct {
        pub fn get_absolute_time() time.Absolute {
            return @import("root").time_absolute;
        }
    },
};
pub const board = switch (options.board) {
    .madflight_rp2350 => @import("boards/madflight_rp2350.zig"),
    .simulator => @compileError("simulator doesn't have a board"),
};

pub const enter_critical_section = chip.enter_critical_section;
pub const CriticalSection = chip.CriticalSection;

pub const get_time_since_boot = chip.get_time_since_boot;
pub const clock = chip.clock;
pub const flash = chip.flash;

pub const Pin = chip.Pin;
pub const InterruptPin = chip.InterruptPin;

pub const UART_Config = chip.UART_Config;
pub const UART_TX = chip.UART_TX;
pub const UART_RX = chip.UART_RX;

pub const SPI_Config = chip.SPI_Config;
pub const SPI = chip.SPI;

pub const I2C_Config = chip.I2C_Config;
pub const PWM_Config = chip.PWM_Config;

pub const Definition = struct {
    imu: struct {
        tick_period: time.Duration,
        type: enum {
            lsm6dsv,
        },
        spi: SPI_Config,
        pin_interrupt: Pin,
    },
    receiver: struct {
        protocol: enum {
            crsf,
        },
        uart: UART_Config,
    },
    motors: []const struct {
        protocol: enum {
            dshot_300,
        },
        pins: []const Pin,
    },
    servos: []const PWM_Config,
};

pub const def: Definition = board.hw_def;
