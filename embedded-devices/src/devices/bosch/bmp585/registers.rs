use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;

/// Known chip ids
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Chip {
    BMP585 = 0x51,
    Invalid(u8),
}

/// The chip identification register.
#[device_register(super::BMP585)]
#[register(address = 0x01, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ChipId {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Chip::BMP585)]
    pub chip: Chip,
}

/// The chip revision register.
#[device_register(super::BMP585)]
#[register(address = 0x02, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct RevId {
    #[register(default = 0x32)]
    pub revision: u8,
}

/// Host Interface Mode for BMP585
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum HifMode {
    /// I2C mode only (SPI disabled)
    I2COnly = 0b00,
    /// SPI Mode 1 and Mode 2
    SPIMode1And2 = 0b01,
    /// SPI Mode 0 and Mode 3
    SPIMode0And3 = 0b10,
    /// SPI and I2C available (Autoconfig)
    AutoConfig = 0b11,
}

/// The ASIC status register (0x11).
/// Provides host interface mode and I3C error flags.
#[device_register(super::BMP585)]
#[register(address = 0x11, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct AsicStatus {
    /// Host interface mode (NVM-backed).
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = HifMode::I2COnly)]
    pub hif_mode: HifMode,

    /// I3C SDR parity error occurred (clear-on-read).
    #[register(default = false)]
    pub i3c_parity_error: bool,

    /// I3C S0/S1 error occurred (clear-on-read).
    #[register(default = false)]
    pub i3c_s0s1_error: bool,

    /// Reserved bits [4:7]
    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

#[device_register(super::BMP585)]
#[register(address = 0x28, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Status {
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    #[register(default = false)]
    pub nvm_ready: bool,

    #[register(default = false)]
    pub nvm_error: bool,

    #[register(default = false)]
    pub nvm_cmd_error: bool,

    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,
}

#[device_register(super::BMP585)]
#[register(address = 0x1D, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct Temperature {
    #[bondrewd(bit_length = 24)]
    #[register(default = 0)]
    pub temperature: u32, // Interpreted as signed 24-bit, shifted by 16
}

#[device_register(super::BMP585)]
#[register(address = 0x20, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct Pressure {
    #[bondrewd(bit_length = 24)]
    #[register(default = 0)]
    pub pressure: u32, // Interpreted as signed 24-bit, shifted by 6
}

#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Cmd {
    Nop = 0x00,
    Reset = 0xB6,
    ReadNvmA5 = 0xA5,
    ReadNvmA0 = 0xA0,
    Invalid(u8),
}

#[device_register(super::BMP585)]
#[register(address = 0x7E, mode = "w")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Command {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Cmd::Nop)]
    pub command: Cmd,
}

/// FIFO operational mode
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum FifoMode {
    Streaming = 0,
    StopOnFull = 1,
    Invalid(u8),
}

/// FIFO configuration register
#[device_register(super::BMP585)]
#[register(address = 0x16, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoConfig {
    #[bondrewd(bit_length = 5)]
    #[register(default = 0)]
    pub fifo_threshold: u8, // 0x00 to 0x1F

    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = FifoMode::Streaming)]
    pub fifo_mode: FifoMode,

    #[bondrewd(bit_length = 2, reserve)]
    pub reserved: u8,
}

/// FIFO frame count register
#[device_register(super::BMP585)]
#[register(address = 0x17, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoCount {
    #[register(default = 0)]
    pub fifo_count: u8,
}

/// FIFO frame type selection
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum FifoFrameType {
    /// Disabled - FIFO frame type is disabled.
    Disabled = 0b00,
    /// Temperature only - FIFO frame type is temperature only.
    TemperatureOnly = 0b01,
    /// Pressure only - FIFO frame type is pressure only.
    PressureOnly = 0b10,
    /// Pressure and temperature - FIFO frame type is pressure and temperature.
    PressureTemperature = 0b11,
    /// Invalid - FIFO frame type is invalid.
    Invalid(u8),
}

#[device_register(super::BMP585)]
#[register(address = 0x18, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoSelect {
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = FifoFrameType::Disabled)]
    pub fifo_frame_sel: FifoFrameType,

    #[bondrewd(bit_length = 3)]
    #[register(default = 0)]
    pub fifo_dec_sel: u8,

    #[bondrewd(bit_length = 3, reserve)]
    pub reserved: u8,
}

#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IntMode {
    Pulsed = 0,
    Latched = 1,
}

#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IntPolarity {
    ActiveLow = 0,
    ActiveHigh = 1,
}

#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IntOutput {
    PushPull = 0,
    OpenDrain = 1,
}

/// Interrupt config
#[device_register(super::BMP585)]
#[register(address = 0x14, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptConfig {
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = IntMode::Pulsed)]
    pub int_mode: IntMode,

    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = IntPolarity::ActiveLow)]
    pub int_pol: IntPolarity,

    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = IntOutput::PushPull)]
    pub int_od: IntOutput,

    #[bondrewd(bit_length = 1)]
    #[register(default = false)]
    pub int_en: bool,

    #[bondrewd(bit_length = 4)]
    pub pad_int_drv: u8,
}

/// Interrupt source enable
#[device_register(super::BMP585)]
#[register(address = 0x15, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptSource {
    #[register(default = false)]
    pub drdy_data_reg_en: bool,
    #[register(default = false)]
    pub fifo_full_en: bool,
    #[register(default = false)]
    pub fifo_ths_en: bool,
    #[register(default = false)]
    pub oor_p_en: bool,

    #[bondrewd(bit_length = 4, reserve)]
    pub reserved: u8,
}

/// Oversampling rate
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum Oversampling {
    /// Disables oversampling.
    X1 = 0,
    /// Configures 2x oversampling.
    X2 = 1,
    /// Configures 4x oversampling.
    X4 = 2,
    /// Configures 8x oversampling.
    X8 = 3,
    /// Configures 16x oversampling.
    /// If this is used as the pressure oversampling rate, it is recommended to also use
    /// at least 2x temperature oversampling to get accurate compensation.
    X16 = 4,
    /// Configures 32x oversampling.
    /// If this is used as the pressure oversampling rate, it is recommended to also use
    /// at least 2x temperature oversampling to get accurate compensation.
    X32 = 5,

    X64 = 6,
    X128 = 7,
}

#[device_register(super::BMP585)]
#[register(address = 0x36, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OversamplingConfig {
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X1)]
    pub osr_t: Oversampling,

    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X4)]
    pub osr_p: Oversampling,

    #[register(default = true)]
    pub press_en: bool,

    #[bondrewd(bit_length = 1, reserve)]
    pub reserved: u8,
}

/// IIR filter coefficient for pressure and temperature.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IIRFilter {
    /// Disables the IIR filter (default).
    Disabled = 0b000,
    /// Sets the IIR filter coefficient to 1.
    Coefficient1 = 0b001,
    /// Sets the IIR filter coefficient to 3.
    Coefficient3 = 0b010,
    /// Sets the IIR filter coefficient to 7.
    Coefficient7 = 0b011,
    /// Sets the IIR filter coefficient to 15.
    Coefficient15 = 0b100,
    /// Sets the IIR filter coefficient to 31.
    Coefficient31 = 0b101,
    /// Sets the IIR filter coefficient to 63.
    Coefficient63 = 0b110,
    /// Sets the IIR filter coefficient to 127.
    Coefficient127 = 0b111,
}

/// DSP IIR filter configuration register
#[device_register(super::BMP585)]
#[register(address = 0x31, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IIRFilterConfig {
    /// Pressure filter coefficient
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = IIRFilter::Disabled)]
    pub pressure_iir: IIRFilter,

    /// Temperature filter coefficient
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = IIRFilter::Disabled)]
    pub temperature_iir: IIRFilter,

    /// Reserved
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// Reset IIR filter on next forced measurement
    #[register(default = false)]
    pub flush_on_forced: bool,
}
