use bondrewd::{BitfieldEnum, Bitfields};
use embedded_devices_derive::device_register;
use embedded_registers::register;

// This would typically be in a higher-level module, e.g., `mod.rs` or `lib.rs`
// For this example, we define it here.
// pub struct ENS220DeviceInterface; // Placeholder for actual I2C/SPI interface type
// pub type ENS220Common = ENS220DeviceInterface; // Or some common struct

// Placeholder for the main device struct or a common type used by `device_register`
// In a real driver, this would be replaced with the actual device struct.
pub struct ENS220Common;

/// Device ID value for ENS220.
/// The PART_ID register (0x00-0x01) results in 0x0321. [cite: 9, 7]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u16)]
#[repr(u16)]
pub enum DeviceIdentifier {
    /// ENS220 Device ID
    ENS220 = 0x0321,
    /// Unknown or invalid device ID
    Unknown(u16),
}

/// The chip identification number (PART_ID).
/// This 16-bit number can be read as soon as the device is powered up. [cite: 8]
/// It is stored in little-endian format across addresses 0x00 (LSB) and 0x01 (MSB). [cite: 7, 9]
/// Default value is 0x0321. [cite: 9]
#[device_register(super::ENS220Common)]
#[register(address = 0x00, mode = "r")] // Access: Rr [cite: 9]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct PartId {
    /// The 16-bit device identifier.
    #[bondrewd(enum_primitive = "u16", bit_length = 16)]
    #[register(default = DeviceIdentifier::Unknown(0))] // Chip power-on default is ENS220
    pub device_id: DeviceIdentifier,
}

/// Unique ID (UID) of the device.
/// This 32-bit unique device identifier is read-only. [cite: 10]
/// It is stored in little-endian format across addresses 0x02-0x05. [cite: 11, 7]
#[device_register(super::ENS220Common)]
#[register(address = 0x02, mode = "r")] // Access: R [cite: 11]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 4)]
pub struct Uid {
    /// The 32-bit unique identifier.
    #[bondrewd(bit_length = 32)]
    pub unique_id: u32,
}

/// FIFO Mode settings for the MODE_CFG register. [cite: 14]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum FifoMode {
    /// Direct path, FIFO disabled.
    DirectPath = 0b00,
    /// FIFO enabled for pressure data.
    Fifo = 0b01,
    /// Moving average filter enabled for pressure data.
    MovingAverage = 0b10,
    /// Reserved setting.
    Reserved = 0b11,
}

/// Device Configuration (MODE_CFG) register.
/// Address: 0x06. Default: 0x03. Access: RWrw. [cite: 2, 14]
#[device_register(super::ENS220Common)]
#[register(address = 0x06, mode = "rw")] // Access: RWrw (Read/Write, some restrictions in low power) [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)] // Single byte
pub struct ModeCfg {
    /// High Power Bit.
    /// 0b1: All registers accessible via SPI/I2C; high power consumption.
    /// 0b0: Limited registers accessible; low power consumption. (Default) [cite: 14]
    #[register(default = false)] // Bit 7, Default 0b0
    pub hp: bool,

    /// Pressure data path configuration (FIFO mode). (Default: DirectPath) [cite: 14]
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = FifoMode::DirectPath)] // Bits 6:5, Default 0b00
    pub fifo_mode: FifoMode,

    /// Operating mode configuration (Start measurements).
    /// 0b0: Stop measurements (idle mode). (Default) [cite: 14]
    /// 0b1: Start measurements (measurement mode).
    #[register(default = false)] // Bit 4, Default 0b0
    pub start: bool,

    /// Device reset.
    /// 0b1: The device is reset to power-on configuration. Auto-cleared. (Default: 0b0) [cite: 14]
    #[register(default = false)] // Bit 3, Default 0b0
    pub reset: bool,

    /// Reserved bit. Must be set to default value (0b0). [cite: 14]
    #[allow(dead_code)]
    #[register(default = false)] // Bit 2, Default 0b0
    pub reserved_x: bool,

    /// Enable temperature measurements. (Default: 0b1, enabled) [cite: 14]
    /// Works in conjunction with MEAS_P. See Table 13. [cite: 17]
    #[register(default = true)] // Bit 1, Default 0b1
    pub meas_t: bool,

    /// Enable pressure measurements. (Default: 0b1, enabled) [cite: 14]
    /// Works in conjunction with MEAS_T. See Table 13. [cite: 17]
    #[register(default = true)] // Bit 0, Default 0b1
    pub meas_p: bool,
}

/// Pressure ADC conversion time settings for MEAS_CFG.P_CONV. [cite: 21]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum PressureConvTime {
    /// First conversion 4ms, subsequent 1ms.
    Ms4_1 = 0b00,
    /// First conversion 8ms, subsequent 2ms. (Default) [cite: 19]
    Ms8_2 = 0b01,
    /// First conversion 16ms, subsequent 4ms.
    Ms16_4 = 0b10,
    /// Reserved setting.
    Reserved = 0b11,
}

/// Ratio between Pressure and Temperature measurements for MEAS_CFG.PT_RATE. [cite: 24]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum PressureTempRate {
    /// P/T rate: 1 (Default) [cite: 19]
    Rate1 = 0b000,
    /// P/T rate: 4
    Rate4 = 0b001,
    /// P/T rate: 8
    Rate8 = 0b010,
    /// P/T rate: 16
    Rate16 = 0b011,
    /// P/T rate: 32
    Rate32 = 0b100,
    /// P/T rate: 64
    Rate64 = 0b101,
    /// P/T rate: 128
    Rate128 = 0b110,
    /// P/T rate: 256
    Rate256 = 0b111,
}

/// Measurement Configuration (MEAS_CFG) register.
/// Address: 0x07. Default: 0x08. Access: RW. [cite: 2, 19]
#[device_register(super::ENS220Common)]
#[register(address = 0x07, mode = "rw")] // Access: RW [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct MeasCfg {
    /// Reserved bits. Must be set to default value (0b000). [cite: 19]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 3)]
    #[register(default = 0b000)] // Bits 7:5, Default 0b000
    pub reserved_x: u8,

    /// Pressure ADC conversion time. (Default: Ms8_2) [cite: 19]
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = PressureConvTime::Ms8_2)] // Bits 4:3, Default 0b01
    pub p_conv: PressureConvTime,

    /// Ratio between P and T measurements. (Default: Rate1) [cite: 19]
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = PressureTempRate::Rate1)] // Bits 2:0, Default 0b000
    pub pt_rate: PressureTempRate,
}

/// Standby duration settings for STBY_CFG.STBY_T. [cite: 29]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum StandbyDuration {
    /// Continuous operation. (Default) [cite: 27]
    Continuous = 0b0000,
    /// One-shot operation (device returns to idle after one measurement).
    OneShot = 0b0001,
    /// Standby 10 ms.
    Ms10 = 0b0010,
    /// Standby 20 ms.
    Ms20 = 0b0011,
    /// Standby 30 ms.
    Ms30 = 0b0100,
    /// Standby 50 ms.
    Ms50 = 0b0101,
    /// Standby 100 ms.
    Ms100 = 0b0110,
    /// Standby 250 ms.
    Ms250 = 0b0111,
    /// Standby 500 ms.
    Ms500 = 0b1000,
    /// Standby 750 ms.
    Ms750 = 0b1001,
    /// Standby 1000 ms.
    Ms1000 = 0b1010,
    /// Standby 2000 ms.
    Ms2000 = 0b1011,
    /// Standby 5000 ms.
    Ms5000 = 0b1100,
    /// Standby 10000 ms.
    Ms10000 = 0b1101,
    /// Standby 60000 ms.
    Ms60000 = 0b1110,
    /// Standby 600000 ms.
    Ms600000 = 0b1111,
}

/// Standby Time Configuration (STBY_CFG) register.
/// Address: 0x08. Default: 0x00. Access: RWw. [cite: 2, 27]
#[device_register(super::ENS220Common)]
#[register(address = 0x08, mode = "rw")] // Access: RWw (Write access in low power mode) [cite: 2, 3]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct StbyCfg {
    /// Reserved bits. Needs to be set to default value (0x0). [cite: 27]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x0)] // Bits 7:4, Default 0x0
    pub reserved_x: u8,

    /// Standby duration in-between measurements. (Default: Continuous) [cite: 27]
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    #[register(default = StandbyDuration::Continuous)] // Bits 3:0, Default 0x0
    pub stby_t: StandbyDuration,
}

/// Oversampling settings for OVS_CFG.OVSP and OVS_CFG.OVST. [cite: 34, 35]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum OversamplingSetting {
    /// Number of averages: 1 (Default) [cite: 32]
    Avg1 = 0b000,
    /// Number of averages: 2
    Avg2 = 0b001,
    /// Number of averages: 4
    Avg4 = 0b010,
    /// Number of averages: 8
    Avg8 = 0b011,
    /// Number of averages: 16
    Avg16 = 0b100,
    /// Number of averages: 32
    Avg32 = 0b101,
    /// Number of averages: 64
    Avg64 = 0b110,
    /// Number of averages: 128
    Avg128 = 0b111,
}

/// Oversampling Settings (OVS_CFG) register.
/// Address: 0x09. Default: 0x00. Access: RW. [cite: 2, 32]
#[device_register(super::ENS220Common)]
#[register(address = 0x09, mode = "rw")] // Access: RW [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OvsCfg {
    /// Reserved bits. Must be set to default value (0b00). [cite: 32]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 2)]
    #[register(default = 0b00)] // Bits 7:6, Default 0b00
    pub reserved_x: u8,

    /// Oversampling of pressure measurements. (Default: Avg1) [cite: 32]
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = OversamplingSetting::Avg1)] // Bits 5:3, Default 0b000
    pub ovsp: OversamplingSetting,

    /// Oversampling of temperature measurements. (Default: Avg1) [cite: 32]
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = OversamplingSetting::Avg1)] // Bits 2:0, Default 0b000
    pub ovst: OversamplingSetting,
}

/// Moving average filter sample count for MAVG_CFG.MAVG. [cite: 40]
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum MovingAverageSamples {
    /// Samples: 1 (Default) [cite: 38]
    Samples1 = 0b000,
    /// Samples: 2
    Samples2 = 0b001,
    /// Samples: 4
    Samples4 = 0b010,
    /// Samples: 8
    Samples8 = 0b011,
    /// Samples: 16
    Samples16 = 0b100,
    /// Samples: 32 (represented by 0b101)
    Samples32_Val5 = 0b101,
    /// Samples: 32 (represented by 0b110)
    Samples32_Val6 = 0b110,
    /// Samples: 32 (represented by 0b111)
    Samples32_Val7 = 0b111,
}

/// Moving Average Configuration (MAVG_CFG) register.
/// Address: 0x0A. Default: 0x00. Access: RW. [cite: 2, 38]
#[device_register(super::ENS220Common)]
#[register(address = 0x0A, mode = "rw")] // Access: RW [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct MavgCfg {
    /// Reserved bits. Must be set to default value (0x00). [cite: 38]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x00)] // Bits 7:3, Default 0x00
    pub reserved_x: u8,

    /// Controls the number of samples used by the moving average filter. (Default: Samples1) [cite: 38]
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = MovingAverageSamples::Samples1)] // Bits 2:0, Default 0b000
    pub mavg: MovingAverageSamples,
}

/// Host Interface Configuration (INTF_CFG) register.
/// Address: 0x0B. Default: 0x00. Access: RWw. [cite: 2, 43]
#[device_register(super::ENS220Common)]
#[register(address = 0x0B, mode = "rw")] // Access: RWw [cite: 2, 3]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IntfCfg {
    /// Reserved bits. Must be set to default value (0x0). [cite: 43]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x0)] // Bits 7:3, Default 0x0
    pub reserved_x: u8,

    /// Interrupt enable.
    /// 0b1: INT/SDO is controlled by INT_STAT.IA.
    /// 0b0: INT/SDO is always low. (Default) [cite: 43]
    #[register(default = false)] // Bit 2, Default 0b0
    pub int_en: bool,

    /// Interrupt polarity.
    /// 0b1: INT/SDO low signals that interrupt is asserted.
    /// 0b0: INT/SDO high signals that interrupt is asserted. (Default) [cite: 43]
    #[register(default = false)] // Bit 1, Default 0b0
    pub int_ht_pol: bool, // Renamed from int_ht for clarity: false=active_high, true=active_low

    /// SPI mode control.
    /// 0b1: SPI works in 3-wire mode.
    /// 0b0: SPI works in 4-wire mode. (Default) [cite: 43]
    #[register(default = false)] // Bit 0, Default 0b0
    pub spi3_enable: bool,
}

/// Interrupt Mask Configuration (INT_CFG) register.
/// Address: 0x0C. Default: 0x7F. Access: RWw. [cite: 2, 46]
#[device_register(super::ENS220Common)]
#[register(address = 0x0C, mode = "rw")] // Access: RWw [cite: 2, 3]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IntCfg {
    /// Reserved bit. Must be set to default value (0b0). [cite: 46]
    #[allow(dead_code)]
    #[register(default = false)] // Bit 7, Default 0b0
    pub reserved_x: bool,

    /// Temperature data is ready interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 6, Default 0b1
    pub temp_ready_int_en: bool,

    /// Pressure FIFO reached high threshold interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 5, Default 0b1
    pub fifo_high_int_en: bool,

    /// Pressure FIFO is full interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 4, Default 0b1
    pub fifo_full_int_en: bool,

    /// Pressure FIFO is empty interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 3, Default 0b1
    pub fifo_empty_int_en: bool,

    /// Pressure data is available interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 2, Default 0b1
    pub pressure_ready_int_en: bool,

    /// Pressure high threshold interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 1, Default 0b1
    pub pressure_high_int_en: bool,

    /// Pressure low threshold interrupt enable. (Default: 0b1, enabled) [cite: 46]
    #[register(default = true)] // Bit 0, Default 0b1
    pub pressure_low_int_en: bool,
}

/// Low Pressure Threshold (PRESS_LO) register.
/// Address: 0x0D-0x0F. Default: 0x000000. Access: RW. [cite: 2, 49]
/// This 3-byte register sets the 24-bit low-pressure interrupt threshold in 1/64 Pa. Stored little-endian. [cite: 48, 7]
#[device_register(super::ENS220Common)]
#[register(address = 0x0D, mode = "rw")] // Access: RW [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressLoThreshold {
    /// Low pressure threshold value [23:0].
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x000000)]
    pub threshold: u32,
}

/// High Pressure Threshold (PRESS_HI) register.
/// Address: 0x10-0x12. Default: 0xFFFFFF. Access: RW. [cite: 2] Note: Table 27 [cite: 51] field defaults differ from Table 9[cite: 2]. Using Table 9 for overall default.
/// This 3-byte register sets the 24-bit high-pressure interrupt threshold in 1/64 Pa. Stored little-endian. [cite: 50, 7]
#[device_register(super::ENS220Common)]
#[register(address = 0x10, mode = "rw")] // Access: RW [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressHiThreshold {
    /// High pressure threshold value [23:0].
    #[bondrewd(bit_length = 24)]
    #[register(default = 0xFFFFFF)] // Default from Table 9 for PRESS_HI_XL, L, H [cite: 2]
    pub threshold: u32,
}

/// FIFO Configuration (FIFO_CFG) register.
/// Address: 0x13. Default: 0x00. Access: RW. [cite: 2, 54]
#[device_register(super::ENS220Common)]
#[register(address = 0x13, mode = "rw")] // Access: RW [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoCfg {
    /// Reserved bits. Must be set to default value (0b00). [cite: 54]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 2)]
    #[register(default = 0b00)] // Bits 7:6, Default 0b00
    pub reserved_x: u8,

    /// FIFO clear.
    /// 0b1: The content of the FIFO is cleared. Auto-cleared.
    /// 0b0: No operation. (Default) [cite: 54]
    #[register(default = false)] // Bit 5, Default 0b0
    pub fp_clear: bool,

    /// FIFO level threshold for interrupt/status generation (0-31). (Default: 0x00) [cite: 54]
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x00)] // Bits 4:0, Default 0x00
    pub fp_fill_th: u8,
}

/// Measurement Data Status (DATA_STAT) register.
/// Address: 0x14. Default: 0x00. Access: Rr. [cite: 2, 57]
#[device_register(super::ENS220Common)]
#[register(address = 0x14, mode = "r")] // Access: Rr (Read access, also in ultra-low power) [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct DataStat {
    /// Reserved bits. Reads 0. [cite: 57]
    #[allow(dead_code)]
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x0)] // Bits 7:4, Default 0x0
    pub reserved_x: u8,

    /// Pressure overwrite. Set if new P data arrives while FIFO full (FIFO enabled) or previous data not read.
    /// Cleared after reading PRESS_OUT_H or PRESS_OUT_F_H. (Default: 0b0) [cite: 57]
    #[register(default = false)] // Bit 3, Default 0b0
    pub pressure_overwrite: bool,

    /// Temperature overwrite. Set if new T data arrives and previous was not read.
    /// Cleared after reading TEMP_OUT_H. (Default: 0b0) [cite: 57]
    #[register(default = false)] // Bit 2, Default 0b0
    pub temp_overwrite: bool,

    /// Pressure ready. Set when new pressure data is available.
    /// Cleared after reading PRESS_OUT_H. (Default: 0b0) [cite: 57]
    #[register(default = false)] // Bit 1, Default 0b0
    pub pressure_ready: bool,

    /// Temperature ready. Set when new temperature data is available.
    /// Cleared after reading TEMP_OUT_H. (Default: 0b0) [cite: 57]
    #[register(default = false)] // Bit 0, Default 0b0
    pub temp_ready: bool,
}

/// FIFO Status (FIFO_STAT) register.
/// Address: 0x15. Default: 0x02. Access: R. [cite: 2, 60]
#[device_register(super::ENS220Common)]
#[register(address = 0x15, mode = "r")] // Access: R [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoStat {
    /// Fill level of the pressure FIFO (0-31). Value 0x1F for 31 or 32 elements. (Default: 0x0) [cite: 60]
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x00)] // Bits 7:3, Default 0x0
    pub fp_fill_level: u8,

    /// FIFO full flag. Set when FIFO is enabled and full (32 elements). (Default: 0b0) [cite: 60]
    #[register(default = false)] // Bit 2, Default 0b0
    pub fifo_full: bool,

    /// FIFO empty flag. Set when FIFO is enabled and empty (0 elements). (Default: 0b1) [cite: 60]
    #[register(default = true)] // Bit 1, Default 0b1
    pub fifo_empty: bool,

    /// FIFO high threshold met. Set when enabled and #elements > FP_FILL_TH. (Default: 0b0) [cite: 60]
    #[register(default = false)] // Bit 0, Default 0b0
    pub fifo_thresh_high_met: bool,
}

/// Interrupt Status (INT_STAT) register.
/// Address: 0x16. Default: 0x00 (based on Table 31 bit defaults[cite: 64], Table 9 lists 0x08 [cite: 2]). Access: Rr.
/// Reading this register clears all flags. [cite: 63]
#[device_register(super::ENS220Common)]
#[register(address = 0x16, mode = "r")] // Access: Rr [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IntStat {
    /// General interrupt flag (IA). Set if any enabled interrupt (bits 6:0) is active. (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 7, Default 0b0
    pub interrupt_active: bool,

    /// Temperature ready status (TR). (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 6, Default 0b0
    pub temp_ready_status: bool,

    /// FIFO high threshold status (FH). (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 5, Default 0b0
    pub fifo_high_thresh_status: bool,

    /// FIFO full status (FF). (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 4, Default 0b0
    pub fifo_full_status: bool,

    /// FIFO empty status (FE). (Default: 0b0, Note: Table 9 implies 0x08 reg default, i.e., this bit=1) [cite: 64, 2]
    #[register(default = false)] // Bit 3, Default 0b0
    pub fifo_empty_status: bool,

    /// Pressure ready status (PR). (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 2, Default 0b0
    pub pressure_ready_status: bool,

    /// Pressure high threshold status (PH). (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 1, Default 0b0
    pub pressure_high_status: bool,

    /// Pressure low threshold status (PL). (Default: 0b0) [cite: 64]
    #[register(default = false)] // Bit 0, Default 0b0
    pub pressure_low_status: bool,
}

/// Pressure Output (PRESS_OUT) register.
/// Address: 0x17-0x19. Default: 0x000000. Access: Rry. [cite: 2, 73]
/// Contains a 24-bit unsigned integer representing pressure in 1/64 Pa. Little-endian. [cite: 65, 7]
/// Reading extracts from FIFO if enabled (HP=1 required), else latest measurement. [cite: 65, 66, 67]
/// Returns 0x000000 if FIFO is empty and enabled. [cite: 66]
#[device_register(super::ENS220Common)]
#[register(address = 0x17, mode = "r")] // Access: Rry (Read, last value in ultra-low, y implies FIFO interaction) [cite: 2, 3]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressOut {
    /// Pressure value [23:0] in 1/64 Pa.
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x000000)]
    pub pressure_val: u32,
}

/// Temperature Output (TEMP_OUT) register.
/// Address: 0x1A-0x1B. Default: 0x0000. Access: Rr. [cite: 2, 84]
/// Contains a 16-bit unsigned integer representing temperature in 1/128 K. Little-endian. [cite: 75, 7]
/// Returns the latest measurement result. [cite: 76]
#[device_register(super::ENS220Common)]
#[register(address = 0x1A, mode = "r")] // Access: Rr [cite: 2]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct TempOut {
    /// Temperature value [15:0] in 1/128 K.
    #[bondrewd(bit_length = 16)]
    #[register(default = 0x0000)]
    pub temperature_val: u16,
}

/// FIFO Pressure Output (PRESS_OUT_F) register.
/// Address: 0x27-0x29. Default: 0x000000. Access: RFry. [cite: 2, 89]
/// Same as PRESS_OUT, but supports wrap-around reads for multiple FIFO entries in one transaction. [cite: 86]
/// HP bit in MODE_CFG must be 1 for FIFO access. [cite: 87] Little-endian. [cite: 7]
#[device_register(super::ENS220Common)]
#[register(address = 0x27, mode = "r")] // Access: RFry (Read FIFO, wraps, y implies specific conditions) [cite: 2, 3]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressOutF {
    /// FIFO Pressure value [23:0] in 1/64 Pa.
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x000000)]
    pub pressure_fifo_val: u32,
}
