const hw = @import("../hw.zig");

pub const FLASH_SIZE = 16 * 1024 * 1024;
pub const FLASH_STORAGE_START = FLASH_SIZE - 64 * 1024;
pub const FLASH_STORAGE_END = FLASH_SIZE;

// TODO: separate config into build specific (eg. my custom plane) and board
// specific
pub const hw_def: hw.Definition = .{
    .imu = .{
        .tick_period = .from_hz(1000),
        .type = .lsm6dsv,
        .spi = .{
            .instance = .num(1),
            .baud_rate = 10_000_000,
            .pin_clk = .num(30),
            .pin_mosi = .num(31),
            .pin_miso = .num(28),
            .pin_cs = .num(29),
        },
        .pin_interrupt = .num(27),
    },
    .receiver = .{
        .protocol = .crsf,
        .uart = .{
            .instance = .{ .uart = .num(0) },
            .baud_rate = 115_200,
            .pin_tx = .num(0),
            .pin_rx = .num(1),
            .buf_size_tx = 128,
            .buf_size_rx = 128,
        },
    },
    .motors = &.{
        .{ .protocol = .dshot_300, .pins = &.{.num(6)} },
    },
    .servos = &.{
        .{ .pwm_slice = .num(5), .pin_a = .num(11) },
        .{ .pwm_slice = .num(7), .pin_a = .num(15) },
    },
    .flash = .{
        .size = FLASH_SIZE,
        .storage_start = FLASH_SIZE - 64 * 1024,
        .storage_end = FLASH_SIZE,
    },
};

// assign_resources! {
//     esc: EscOutputPeripherals {
//         pio: PIO0,
//         signal_pin: PIN_6,
//         telemetry_pin: PIN_16,
//     },
//     servos: ServoOutputsPeripherals {
//         left_elevon_slice: PWM_SLICE5,
//         right_elevon_slice: PWM_SLICE7,
//         left_elevon_pin: PIN_11,
//         right_elevon_pin: PIN_15,
//     },
//     imu: ImuPeripherals {
//         spi: SPI1,
//         clk_pin: PIN_30,
//         mosi_pin: PIN_31,
//         miso_pin: PIN_28,
//         cs_pin: PIN_29,
//         tx_dma: DMA_CH0,
//         rx_dma: DMA_CH1,
//         int_pin: PIN_27,
//     },
//     receiver: ReceiverPeripherals {
//         uart: UART0,
//         tx_pin: PIN_0,
//         rx_pin: PIN_1,
//     },
//     gps: GpsPeripherals {
//         uart: UART1,
//         tx_pin: PIN_4,
//         rx_pin: PIN_5,
//     },
//     vtx: VtxPeripherals {
//         pio: PIO1,
//         tx_pin: PIN_12,
//         rx_pin: PIN_13,
//     },
//     i2c_bus: I2cBusPeripherals {
//         i2c: I2C0,
//         sda_pin: PIN_32,
//         scl_pin: PIN_33,
//     },
//     sd_card: SdCardPeripherals {
//         spi: SPI0,
//         clk_pin: PIN_34,
//         mosi_pin: PIN_35,
//         miso_pin: PIN_36,
//         cs_pin: PIN_39,
//         tx_dma: DMA_CH3,
//         rx_dma: DMA_CH4,
//     },
//     flash: FlashPeripherals {
//         flash: FLASH,
//     },
//     usb: UsbPeripherals {
//         usb: USB,
//     },
//     led: LedPeripherals {
//         pin: PIN_46,
//         pio: PIO2,
//         dma: DMA_CH2,
//     },
// }
