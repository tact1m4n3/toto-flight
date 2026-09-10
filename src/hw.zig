const options = @import("options");

const time = @import("time.zig");

pub const is_sim = options.board == .simulator;

pub const chip = switch (options.board) {
    .madflight_rp2350 => @import("chips/rp2350.zig"),
    .simulator => @import("chips/simulator.zig"),
};
pub const board = switch (options.board) {
    .madflight_rp2350 => @import("boards/madflight_rp2350.zig"),
    .simulator => @compileError("simulator doesn't have a board"),
};

pub const enter_critical_section = chip.enter_critical_section;
pub const CriticalSection = chip.CriticalSection;

pub const get_time_since_boot = chip.get_time_since_boot;
pub const clock = &chip.clock;
pub const Ticker = chip.Ticker;

// TODO: maybe get rid of these re-exports
pub const Pin = chip.Pin;
pub const InterruptPin = chip.InterruptPin;

pub const UART_Config = chip.UART_Config;
pub const UART_TX = chip.UART_TX;
pub const UART_RX = chip.UART_RX;

pub const SPI_Config = chip.SPI_Config;
pub const SPI = chip.SPI;

pub const FlashConfig = chip.FlashConfig;
pub const flash = &chip.flash;
pub const Flash = chip.Flash;

pub const I2C_Config = chip.I2C_Config;

pub const MotorConfig = chip.MotorConfig;
pub const motors = chip.motors;
pub const ServoConfig = chip.ServoConfig;
pub const servos = chip.servos;

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
    motors: struct {
        protocol: enum {
            dshot_300,
        },
        outputs: []const MotorConfig,
    },
    servos: []const ServoConfig,
    flash: chip.FlashConfig,
};

pub const def: Definition = board.hw_def;
