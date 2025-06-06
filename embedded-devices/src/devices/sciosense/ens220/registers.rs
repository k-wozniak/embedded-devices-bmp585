use bondrewd::BitfieldEnum;
use embedded_devices_derive::device_register;
use embedded_registers::register;

/// The chip identification number (PART_ID).
/// This 16-bit number can be read as soon as the device is powered up.
/// It is stored in little-endian format across addresses 0x00 (LSB) and 0x01 (MSB).
/// Default value is 0x0321.
#[device_register(super::ENS220Common)]
#[register(address = 0x00, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct PartId {
    /// The 16-bit device identifier.
    #[bondrewd(bit_length = 16)]
    #[register(default = 0x0321)] // Chip power-on default is ENS220, todo check the default value.
    pub id: u16,
}

/// Unique ID (UID) of the device.
/// This 32-bit unique device identifier is read-only.
/// It is stored in little-endian format across addresses 0x02-0x05.
#[device_register(super::ENS220Common)]
#[register(address = 0x02, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 4)]
pub struct Uid {
    /// The 32-bit unique identifier.
    #[bondrewd(bit_length = 32)]
    pub unique_id: u32,
}

/// FIFO Mode settings for the MODE_CFG register.
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
}

/// Measurement selection (MEAS_T and MEAS_P bits).
/// See Table 13: Measurement selection with MEAS_T and MEAS_P.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum MeasurementSelection {
    /// Only pressure measurements are enabled; the device will begin with a temperature measurement, then continue measuring only pressure.
    PressureOnly = 0b01,
    /// Only temperature measurement is enabled.
    TemperatureOnly = 0b10,
    /// Pressure and temperature measurements are enabled. PT_RATE controls the temperature interleaving timer.
    PressureAndTemperature = 0b11,
}

/// Device Configuration (MODE_CFG) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x06, mode = "rw")] // Access: RWrw (Read/Write, some restrictions in low power)
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct ModeCfg {
    /// High Power Bit.
    /// 0b1: All registers accessible via SPI/I2C; high power consumption.
    /// 0b0: Limited registers accessible; low power consumption. (Default)
    #[register(default = false)]
    pub high_power: bool,

    /// Pressure data path configuration (FIFO mode).
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = FifoMode::DirectPath)]
    pub fifo_mode: FifoMode,

    /// Operating mode configuration (Start measurements).
    /// 0b0: Stop measurements (idle mode). (Default)
    /// 0b1: Start measurements (measurement mode).
    #[register(default = false)]
    pub start: bool,

    /// Device reset.
    /// 0b1: The device is reset to power-on configuration. Auto-cleared.
    #[register(default = false)]
    pub reset: bool,

    /// Reserved bit. Must be set to default value (0b0).
    #[allow(dead_code)]
    #[register(default = false)]
    pub reserved: bool,

    /// Measurement selection (MEAS_T and MEAS_P bits).
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = MeasurementSelection::PressureAndTemperature)]
    pub measurement_selection: MeasurementSelection,
}

/// Pressure ADC conversion time settings for MEAS_CFG.P_CONV.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum PressureConvTime {
    /// First conversion 4ms, subsequent 1ms.
    Ms4_1 = 0b00,
    /// First conversion 8ms, subsequent 2ms. (Default)
    Ms8_2 = 0b01,
    /// First conversion 16ms, subsequent 4ms.
    Ms16_4 = 0b10,
}

/// Ratio between Pressure and Temperature measurements for MEAS_CFG.PT_RATE.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum PressureTempRate {
    /// P/T rate: 1 (Default)
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
#[device_register(super::ENS220Common)]
#[register(address = 0x07, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct MeasCfg {
    /// Reserved bits. Must be set to default value (0b000).
    #[allow(dead_code)]
    #[bondrewd(bit_length = 3)]
    #[register(default = 0b000)]
    pub reserved: u8,

    /// Pressure ADC conversion time.
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    #[register(default = PressureConvTime::Ms8_2)]
    pub p_conv: PressureConvTime,

    /// Ratio between P and T measurements.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = PressureTempRate::Rate1)]
    pub pt_rate: PressureTempRate,
}

/// Standby duration settings for STBY_CFG.STBY_T.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum StandbyDuration {
    /// Continuous operation. (Default)
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
#[device_register(super::ENS220Common)]
#[register(address = 0x08, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct StandbyCfg {
    #[allow(dead_code)]
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x0)]
    pub reserved: u8,

    /// Standby duration in-between measurements.
    #[bondrewd(enum_primitive = "u8", bit_length = 4)]
    #[register(default = StandbyDuration::Continuous)]
    pub standby_duration: StandbyDuration,
}

/// Oversampling settings for OVS_CFG.OVSP and OVS_CFG.OVST.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum OversamplingSetting {
    /// Number of averages: 1 (Default)
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
#[device_register(super::ENS220Common)]
#[register(address = 0x09, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct OvsCfg {
    /// Reserved bits. Must be set to default value (0b00).
    #[allow(dead_code)]
    #[bondrewd(bit_length = 2)]
    #[register(default = 0b00)]
    pub reserved: u8,

    /// Oversampling of pressure measurements.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = OversamplingSetting::Avg1)]
    pub ovsp: OversamplingSetting,

    /// Oversampling of temperature measurements.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = OversamplingSetting::Avg1)]
    pub ovst: OversamplingSetting,
}

/// Moving average filter sample count for MAVG_CFG.MAVG.
#[derive(BitfieldEnum, Copy, Clone, PartialEq, Eq, Debug, defmt::Format)]
#[bondrewd_enum(u8)]
#[repr(u8)]
pub enum MovingAverageSamples {
    /// Samples: 1 (Default)
    Samples1 = 0b000,
    /// Samples: 2
    Samples2 = 0b001,
    /// Samples: 4
    Samples4 = 0b010,
    /// Samples: 8
    Samples8 = 0b011,
    /// Samples: 16
    Samples16 = 0b100,
    /// Samples: 32
    Samples32 = 0b101,
}

/// Moving Average Configuration (MAVG_CFG) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x0A, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct MavgCfg {
    /// Reserved bits. Must be set to default value (0x00).
    #[allow(dead_code)]
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x00)]
    pub reserved: u8,

    /// Controls the number of samples used by the moving average filter.
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    #[register(default = MovingAverageSamples::Samples1)]
    pub mavg: MovingAverageSamples,
}

/// Host Interface Configuration (INTF_CFG) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x0B, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IntfCfg {
    #[allow(dead_code)]
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x0)]
    pub reserved: u8,

    /// Interrupt enable.
    /// 0b1: INT/SDO is controlled by INT_STAT.IA.
    /// 0b0: INT/SDO is always low. (Default)
    #[register(default = false)]
    pub int_en: bool,

    /// Interrupt polarity.
    /// 0b1: INT/SDO low signals that interrupt is asserted.
    /// 0b0: INT/SDO high signals that interrupt is asserted. (Default)
    #[register(default = false)]
    pub int_ht_pol: bool, // Renamed from int_ht for clarity: false=active_high, true=active_low

    /// SPI mode control.
    /// 0b1: SPI works in 3-wire mode.
    /// 0b0: SPI works in 4-wire mode. (Default)
    #[register(default = false)] // Bit 0, Default 0b0
    pub spi3_enable: bool,
}

/// Interrupt Mask Configuration (INT_CFG) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x0C, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IntCfg {
    #[allow(dead_code)]
    #[register(default = false)]
    pub reserved: bool,

    /// Temperature data is ready interrupt enable.
    #[register(default = true)]
    pub temp_ready_int_en: bool,

    /// Pressure FIFO reached high threshold interrupt enable.
    #[register(default = true)]
    pub fifo_high_int_en: bool,

    /// Pressure FIFO is full interrupt enable.
    #[register(default = true)]
    pub fifo_full_int_en: bool,

    /// Pressure FIFO is empty interrupt enable.
    #[register(default = true)]
    pub fifo_empty_int_en: bool,

    /// Pressure data is available interrupt enable.
    #[register(default = true)]
    pub pressure_ready_int_en: bool,

    /// Pressure high threshold interrupt enable.
    #[register(default = true)]
    pub pressure_high_int_en: bool,

    /// Pressure low threshold interrupt enable.
    #[register(default = true)]
    pub pressure_low_int_en: bool,
}

/// Low Pressure Threshold (PRESS_LO) register.
/// This 3-byte register sets the 24-bit low-pressure interrupt threshold in 1/64 Pa.
#[device_register(super::ENS220Common)]
#[register(address = 0x0D, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressLowThreshold {
    /// Low pressure threshold value [23:0].
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x000000)]
    pub threshold: u32,
}

/// High Pressure Threshold (PRESS_HI) register.
/// This 3-byte register sets the 24-bit high-pressure interrupt threshold in 1/64 Pa. Stored little-endian.
#[device_register(super::ENS220Common)]
#[register(address = 0x10, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressHighThreshold {
    /// High pressure threshold value [23:0].
    #[bondrewd(bit_length = 24)]
    #[register(default = 0xFFFFFF)] // Default from Table 9 for PRESS_HI_XL, L, H
    pub threshold: u32,
}

/// FIFO Configuration (FIFO_CFG) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x13, mode = "rw")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoCfg {
    #[allow(dead_code)]
    #[bondrewd(bit_length = 2)]
    #[register(default = 0b00)]
    pub reserved: u8,

    /// FIFO clear.
    /// 0b1: The content of the FIFO is cleared. Auto-cleared.
    /// 0b0: No operation. (Default)
    #[register(default = false)]
    pub clear: bool,

    /// FIFO level threshold for interrupt/status generation (0-31).
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x00)]
    pub fill_threshold: u8,
}

/// Measurement Data Status (DATA_STAT) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x14, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct DataStat {
    #[allow(dead_code)]
    #[bondrewd(bit_length = 4)]
    #[register(default = 0x0)]
    pub reserved: u8,

    /// Pressure overwrite. Set if new P data arrives while FIFO full (FIFO enabled) or previous data not read.
    /// Cleared after reading PRESS_OUT_H or PRESS_OUT_F_H.
    #[register(default = false)]
    pub pressure_overwrite: bool,

    /// Temperature overwrite. Set if new T data arrives and previous was not read.
    /// Cleared after reading TEMP_OUT_H.
    #[register(default = false)]
    pub temp_overwrite: bool,

    /// Pressure ready. Set when new pressure data is available.
    /// Cleared after reading PRESS_OUT_H.
    #[register(default = false)]
    pub pressure_ready: bool,

    /// Temperature ready. Set when new temperature data is available.
    /// Cleared after reading TEMP_OUT_H.
    #[register(default = false)]
    pub temp_ready: bool,
}

/// FIFO Status (FIFO_STAT) register.
#[device_register(super::ENS220Common)]
#[register(address = 0x15, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct FifoStat {
    /// Fill level of the pressure FIFO (0-31). Value 0x1F for 31 or 32 elements.
    #[bondrewd(bit_length = 5)]
    #[register(default = 0x00)]
    pub fp_fill_level: u8,

    /// FIFO full flag. Set when FIFO is enabled and full (32 elements).
    #[register(default = false)]
    pub fifo_full: bool,

    /// FIFO empty flag. Set when FIFO is enabled and empty (0 elements).
    #[register(default = true)]
    pub fifo_empty: bool,

    /// FIFO high threshold met. Set when enabled and #elements > FP_FILL_TH.
    #[register(default = false)]
    pub fifo_thresh_high_met: bool,
}

/// Interrupt Status (INT_STAT) register.
/// Reading this register clears all flags.
#[device_register(super::ENS220Common)]
#[register(address = 0x16, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "be", enforce_bytes = 1)]
pub struct IntStat {
    /// General interrupt flag (IA). Set if any enabled interrupt (bits 6:0) is active.
    #[register(default = false)]
    pub interrupt_active: bool,

    /// Temperature ready status (TR).
    #[register(default = false)]
    pub temp_ready_status: bool,

    /// FIFO high threshold status (FH).
    #[register(default = false)]
    pub fifo_high_thresh_status: bool,

    /// FIFO full status (FF).
    #[register(default = false)]
    pub fifo_full_status: bool,

    /// FIFO empty status (FE).
    #[register(default = false)]
    pub fifo_empty_status: bool,

    /// Pressure ready status (PR).
    #[register(default = false)]
    pub pressure_ready_status: bool,

    /// Pressure high threshold status (PH).
    #[register(default = false)]
    pub pressure_high_status: bool,

    /// Pressure low threshold status (PL).
    #[register(default = false)]
    pub pressure_low_status: bool,
}

/// Pressure Output (PRESS_OUT) register.
/// Contains a 24-bit unsigned integer representing pressure in 1/64 Pa.
/// Reading extracts from FIFO if enabled (HP=1 required), else latest measurement.
/// Returns 0x000000 if FIFO is empty and enabled.
#[device_register(super::ENS220Common)]
#[register(address = 0x17, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressOut {
    /// Pressure value [23:0] in 1/64 Pa.
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x000000)]
    pub pressure_val: u32,
}
/// Temperature Output (TEMP_OUT) register.
/// Contains a 16-bit unsigned integer representing temperature in 1/128 K.
/// Returns the latest measurement result.
/// Stored in little-endian format across 0x1A (LSB) and 0x1B (MSB).
#[device_register(super::ENS220Common)]
#[register(address = 0x1A, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 2)]
pub struct TempOut {
    /// Temperature value [15:0] in 1/128 K.
    #[bondrewd(bit_length = 16)]
    #[register(default = 0x0000)]
    pub temperature_val: u16,
}

/// FIFO Pressure Output (PRESS_OUT_F) register.
/// Same as PRESS_OUT, but supports wrap-around reads for multiple FIFO entries in one transaction.
/// HP bit in MODE_CFG must be 1 for FIFO access.
#[device_register(super::ENS220Common)]
#[register(address = 0x27, mode = "r")]
#[bondrewd(read_from = "msb0", default_endianness = "le", enforce_bytes = 3)]
pub struct PressOutFifo {
    /// FIFO Pressure value [23:0] in 1/64 Pa.
    #[bondrewd(bit_length = 24)]
    #[register(default = 0x000000)]
    pub pressure_fifo_val: u32,
}
