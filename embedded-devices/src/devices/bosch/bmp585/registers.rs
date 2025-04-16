use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;

/// Known chip IDs.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Chip {
    BMP585 = 0x51,
    /// Unknown chip ID.
    Invalid(u8),
}

/// Chip identification register.
#[device_register(super::BMP585)]
#[register(address = 0x01, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ChipId {
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Chip::BMP585)]
    pub chip: Chip,
}

/// Chip revision register.
#[device_register(super::BMP585)]
#[register(address = 0x02, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct RevId {
    #[register(default = 0x32)]
    pub revision: u8,
}

/// Host Interface Mode.
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

/// ASIC status register.
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

/// Interrupt mode selection.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IntMode {
    Pulsed = 0,
    Latched = 1,
}

/// Interrupt pin polarity selection.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IntPolarity {
    ActiveLow = 0,
    ActiveHigh = 1,
}

/// Interrupt pin output configuration.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
pub enum IntOutput {
    PushPull = 0,
    OpenDrain = 1,
}

/// Interrupt configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x14, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptConfig {
    /// Interrupt mode.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = IntMode::Latched)]
    pub mode: IntMode,

    /// Interrupt pin polarity.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = IntPolarity::ActiveLow)]
    pub polarity: IntPolarity,

    /// Interrupt pin configuration.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = IntOutput::OpenDrain)]
    pub pin: IntOutput,

    /// Interrupt enabling.
    #[bondrewd(bit_length = 1)]
    #[register(default = false)]
    pub enabled: bool,

    /// Pad drive strength for INT (MSB should be set in INT open drain config only.)
    /// Note: these register fields should be read-back only after waiting at least 1 μs after they have been written.
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x3)]
    pub pad_drive_strength: u8,
}

/// Interrupt source selection register.
#[device_register(super::BMP585)]
#[register(address = 0x15, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptSource {
    /// Data ready.
    #[register(default = false)]
    pub drdy_data_reg_en: bool,

    /// FIFO full.
    #[register(default = false)]
    pub fifo_full_en: bool,

    /// FIFO threshold enabled.
    #[register(default = false)]
    pub fifo_ths_en: bool,

    /// Pressure data out-of-range.
    #[register(default = false)]
    pub oor_p_en: bool,

    #[bondrewd(bit_length = 4, reserve)]
    pub reserved: u8,
}

/// FIFO operational mode.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum FifoMode {
    /// Streaming mode - FIFO is continuously filled with data.
    Streaming = 0,
    /// Stop on empty - FIFO stops filling when empty.
    StopOnFull = 1,
}

/// FIFO configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x16, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoConfig {
    ///  FIFO threshold. Maximum FIFO threshold is 0x1F: 31 frames.
    /// The FIFO threshold is set to 0x00 by default - FIFO is disabled.
    /// Maximum FIFO threshold is 0x1F: 31 frames.
    #[bondrewd(bit_length = 5)]
    #[register(default = 0)]
    pub fifo_threshold: u8, // 0x00 to 0x1F

    /// FIFO mode.
    #[bondrewd(enum_primitive = "u8", bit_length = 1)]
    #[register(default = FifoMode::Streaming)]
    pub fifo_mode: FifoMode,

    #[bondrewd(bit_length = 2, reserve)]
    pub reserved: u8,
}

/// FIFO frame count register.
#[device_register(super::BMP585)]
#[register(address = 0x17, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoCount {
    /// Reserved bits [7:6].
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    reserved: u8,

    /// Number of frames currently stored in the FIFO buffer (0 to 32).
    #[bondrewd(bit_length = 6)]
    #[register(default = 0)]
    pub fifo_count: u8,
}

/// FIFO frame type selection.
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

/// FIFO selection configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x18, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoSelect {
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = FifoFrameType::Disabled)]
    pub frame_selection: FifoFrameType,

    #[bondrewd(bit_length = 3)]
    #[register(default = 0)]
    pub decimation_selection: u8,

    #[bondrewd(bit_length = 3, reserve)]
    pub reserved: u8,
}

/// Temperature data registers.
#[device_register(super::BMP585)]
#[register(address = 0x1D, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct Temperature {
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x7FFFFF)]
    pub temperature: u32, // Interpreted as signed 24-bit, shifted by 16
}

/// Pressure data registers.
#[device_register(super::BMP585)]
#[register(address = 0x20, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct Pressure {
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x7FFFFF)]
    pub pressure: u32, // Interpreted as signed 24-bit, shifted by 6
}

/// Interrupt status register.
#[device_register(super::BMP585)]
#[register(address = 0x27, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct InterruptStatus {
    /// Data ready interrupt – pressure and/or temperature available
    #[register(default = false)]
    pub data_ready: bool,

    /// FIFO full interrupt – FIFO is completely filled
    #[register(default = false)]
    pub fifo_full: bool,

    /// FIFO threshold interrupt – fill level above configured watermark
    #[register(default = false)]
    pub fifo_threshold: bool,

    /// Out-of-range pressure interrupt – pressure outside configured window
    #[register(default = false)]
    pub out_of_range: bool,

    /// Power-on-reset or soft reset has occurred
    #[register(default = false)]
    pub por_or_reset: bool,

    /// Reserved bits [7:5]
    #[bondrewd(bit_length = 3, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

/// Status register.
#[device_register(super::BMP585)]
#[register(address = 0x28, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Status {
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved0: u8,

    /// If asserted, device is ready for NVM operations.
    #[register(default = true)]
    pub nvm_ready: bool,

    /// If asserted, indicates an NVM error, due to at least one of the following reasons:
    /// - PMU power transition fail on NVM power request.
    /// - NVM timeout errors in P/E.
    /// - NVM Charge Pump voltage fail in PROGRAM/ERASE.
    /// - During last boot/load command, ECC has detected 2+ errors This bit is cleared/updated upon a new NVM command, if Boot command is executed.
    #[register(default = false)]
    pub nvm_error: bool,

    /// TODO: The datasheet states: "TODO UPDATE ME".
    #[register(default = false)]
    pub nvm_cmd_error: bool,

    #[bondrewd(bit_length = 4, reserve)]
    #[allow(dead_code)]
    pub reserved1: u8,
}

/// FIFO output data register.
#[device_register(super::BMP585)]
#[register(address = 0x29, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoData {
    /// FIFO read data byte
    #[register(default = 0x7F)]
    pub fifo_data: u8,
}

/// DSP configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x30, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct DspConfig {
    /// Reserved bits [1:0], always write 0.
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved_0_1: u8,

    /// If set, IIR filter flush is triggered in FORCED mode.
    #[register(default = false)]
    pub iir_flush_forced_en: bool,

    /// Shadow register temperature value selection: before or after IIR filter.
    #[register(default = false)]
    pub shadow_sel_iir_t: bool,

    /// FIFO temperature value selection: before or after IIR filter.
    #[register(default = false)]
    pub fifo_sel_iir_t: bool,

    /// Shadow register pressure value selection: before or after IIR filter.
    #[register(default = false)]
    pub shadow_sel_iir_p: bool,

    /// FIFO pressure value selection: before or after IIR filter.
    #[register(default = false)]
    pub fifo_sel_iir_p: bool,

    /// OOR pressure comparison selection: before or after IIR filter.
    #[register(default = false)]
    pub oor_sel_iir_p: bool,
}

/// IIR filter coefficient selection.
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

/// DSP IIR filter configuration register.
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
    #[bondrewd(bit_length = 2, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

/// Out-of-range (OOR) pressure threshold registers.
#[device_register(super::BMP585)]
#[register(address = 0x32, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct OorThresholdPressure {
    /// Out-of-range pressure threshold in Pascals
    #[bondrewd(bit_length = 16)]
    #[register(default = 0x0000)]
    pub oor_threshold_pascals: u16,
}

/// Out-of-range (OOR) pressure range register.
#[device_register(super::BMP585)]
#[register(address = 0x34, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OorRange {
    /// Out-of-range range in Pa (8-bit signed)
    #[register(default = 0x00)]
    pub oor_range_pascals: u8,
}

/// Out-of-range (OOR) configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x35, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OorConfig {
    /// Bit 16 of the OOR pressure threshold (`oor_threshold_pascals` in `OorThresholdPressure`).
    #[register(default = false)]
    pub oor_thr_p_16: bool,

    /// Reserved bits [5:1]. Should be kept at their default value.
    #[bondrewd(bit_length = 5, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// Out-of-range count limit. Defines how many consecutive out-of-range measurements
    /// trigger the OOR interrupt.
    /// Note: This field cannot be written during an ongoing Pressure/Temperature conversion.
    /// - `0b00`: Counter limit of 1
    /// - `0b01`: Counter limit of 3
    /// - `0b10`: Counter limit of 7
    /// - `0b11`: Counter limit of 15
    #[bondrewd(bit_length = 2)]
    #[register(default = 0x00)] // Counter limit of 1
    pub cnt_lim: u8,
}

/// Oversampling rate selection.
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
    /// Configures 32x oversampling.
    /// If this is used as the pressure oversampling rate, it is recommended to also use
    /// at least 2x temperature oversampling to get accurate compensation.
    X64 = 6,
        /// Configures 32x oversampling.
    /// If this is used as the pressure oversampling rate, it is recommended to also use
    /// at least 2x temperature oversampling to get accurate compensation.
    X128 = 7,
}

/// Over-sampling rate (OSR) configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x36, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OversamplingConfig {
    /// Temperature oversampling rate (OSR_T) selection.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X1)]
    pub osr_t: Oversampling,

    /// Pressure oversampling rate (OSR_P) selection.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = Oversampling::X1)]
    pub osr_p: Oversampling,

    /// Pressure measurement enable.
    /// If set (true), enables sensor pressure measurements.
    /// If clear (false), only temperature measurements are performed.
    #[register(default = false)]
    pub press_en: bool,

    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,
}

/// Power mode selection.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum PowerMode {
    /// Standby mode: no measurement ongoing.
    Standby = 0b00,

    /// Normal mode: measurement in configured ODR grid.
    Normal = 0b01,

    /// Forced mode: forced one-time measurement.
    Forced = 0b10,

    /// Non-Stop mode: repetitive measurements without further duty-cycling.
    NonStop = 0b11,
}

/// Output data rate (ODR) selection.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum DataRateHz {
    /// 240.000 Hz (Error = 0.00)
    Hz240 = 0x00,
    /// 218.537 Hz (Error = 0.67)
    Hz218_537 = 0x01,
    /// 199.111 Hz (Error = 0.44)
    Hz199_111 = 0x02,
    /// 179.200 Hz (Error = 0.44)
    Hz179_200 = 0x03,
    /// 160.000 Hz (Error = 0.00)
    Hz160 = 0x04,
    /// 149.333 Hz (Error = 0.44)
    Hz149_333 = 0x05,
    /// 140.000 Hz (Error = 0.00)
    Hz140 = 0x06,
    /// 129.855 Hz (Error = 0.11)
    Hz129_855 = 0x07,
    /// 120.000 Hz (Error = 0.00)
    Hz120 = 0x08,
    /// 110.164 Hz (Error = 0.15)
    Hz110_164 = 0x09,
    /// 100.299 Hz (Error = 0.30)
    Hz100_299 = 0x0A,
    /// 89.600 Hz (Error = 0.44)
    Hz89_600 = 0x0B,
    /// 80.000 Hz (Error = 0.00)
    Hz80 = 0x0C,
    /// 70.000 Hz (Error = 0.00)
    Hz70 = 0x0D,
    /// 60.000 Hz (Error = 0.00)
    Hz60 = 0x0E,
    /// 50.056 Hz (Error = 0.11)
    Hz50_056 = 0x0F,
    /// 45.025 Hz (Error = 0.06)
    Hz45_025 = 0x10,
    /// 40.000 Hz (Error = 0.00)
    Hz40 = 0x11,
    /// 35.000 Hz (Error = 0.00)
    Hz35 = 0x12,
    /// 30.000 Hz (Error = 0.00)
    Hz30 = 0x13,
    /// 25.005 Hz (Error = 0.02)
    Hz25_005 = 0x14,
    /// 20.000 Hz (Error = 0.00)
    Hz20 = 0x15,
    /// 15.000 Hz (Error = 0.00)
    Hz15 = 0x16,
    /// 10.000 Hz (Error = 0.00)
    Hz10 = 0x17,
    /// 5.000 Hz (Error = 0.00)
    Hz5 = 0x18,
    /// 4.000 Hz (Error = 0.00)
    Hz4 = 0x19,
    /// 3.000 Hz (Error = 0.00)
    Hz3 = 0x1A,
    /// 2.000 Hz (Error = 0.00)
    Hz2 = 0x1B,
    /// 1.000 Hz (Error = 0.00)
    Hz1 = 0x1C,
    /// 0.500 Hz (Error = 0.00)
    Hz0_5 = 0x1D,
    /// 0.250 Hz (Error = 0.00)
    Hz0_25 = 0x1E,
    /// 0.125 Hz (Error = 0.00)
    Hz0_125 = 0x1F,
}

/// Output data rate (ODR) configuration register.
#[device_register(super::BMP585)]
#[register(address = 0x37, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OdrConfig {
    /// Power mode configuration
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = PowerMode::Standby)]
    pub pwr_mode: PowerMode,

    /// Output data rate (ODR) selection.
    /// Note: the configured ODR might be invalid in combination with OSR 
    /// configuration This is observable with the flag odr_is_valid.
    /// If configured ODR/OSR settings are invalid, default OSR settings will be used.
    /// The effective OSR settings for P/T can be read from osr_t_eff and osr_p_eff.
    #[bondrewd(enum_primitive = "u8", bit_length = 5)]
    #[register(default = DataRateHz::Hz1)]
    pub odr: DataRateHz,

    /// Disables deep standby
    #[register(default = false)]
    pub deep_dis: bool,
}

/// Effective over-sampling rate (OSR) register.
#[device_register(super::BMP585)]
#[register(address = 0x38, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OsrEffective {
    /// Effective OSR for temperature.
    #[bondrewd(bit_length = 3)]
    #[register(default = 0x00)]
    pub osr_t_eff: u8,

    /// Effective OSR for pressure.
    #[bondrewd(bit_length = 3)]
    #[register(default = 0x00)]
    pub osr_p_eff: u8,

    /// Reserved bit [6].
    #[bondrewd(bit_length = 1, reserve)]
    #[allow(dead_code)]
    pub reserved: u8,

    /// ODR setting is valid (1) or invalid (0).
    #[register(default = false)]
    pub odr_is_valid: bool,
}

/// Command selection.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum Cmd {
    /// Reserved. No command.
    NoCmd = 0x00,
    /// Triggers a reset.
    /// All user configuration settings are overwritten with their default state.
    /// If this register is set using I2C, an ACK will NOT be transmitted to the host
    Reset = 0xB6,
    /// Last CMD in the sequence 0x5D, 0xA5 which enables the read of the NVM.
    /// If another CMD is sent within the sequence, the sequence for triggering the NVM read is reset.
    ReadNvmA5 = 0xA5,
    /// Last CMD in the sequence 0x5D, 0xA0 which enables the write of the NVM.
    /// If another CMD is sent within the sequence, the sequence for triggering the NVM programming is reset.
    ReadNvmA0 = 0xA0,
    /// First CMD in the sequence 0x5D, 0xA0/0xA5 which enables the write/read of the NVM.
    /// If another CMD is sent within the sequence, the sequence for enabling the NVM prog mode is reset.
    ReadWriteNvm = 0x5D,
    /// Invalid command.
    Invalid(u8),
}

/// Command register.
#[device_register(super::BMP585)]
#[register(address = 0x7E, mode = "w")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct Command {
    /// Command to be executed.
    #[bondrewd(enum_primitive = "u8", bit_length = 8)]
    #[register(default = Cmd::NoCmd)]
    pub command: Cmd,
}
