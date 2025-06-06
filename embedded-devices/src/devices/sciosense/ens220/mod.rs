//! # ENS220
//!
//! The ENS220 is an ultra-low-power, high-accuracy barometric pressure and temperature
//! sensor from ScioSense. It is designed for applications such as activity tracking,
//! indoor navigation/localization, and liquid-level detection.
//!
//! ## Features
//!
//! - Absolute pressure range: 300 … 1200 hPa
//! - Temperature range: -40 … +85 °C
//! - Relative pressure accuracy: ±0.025 hPa (equivalent to ±20 cm)
//! - Absolute pressure accuracy: ±0.5 hPa
//! - Low noise: down to 0.1 Pa RMS
//! - Power-efficient: 0.1 µA idle, 0.8 µA at 1/60 Hz
//! - Configurable oversampling, moving average filter (for pressure), FIFO buffer, and interrupt functionality
//! - Supported interfaces: I²C, SPI (3- or 4-wire)
//!
//! ## Current Implementation Status
//!
//! ✅ Implemented:
//! - I²C and SPI (4-wire, auto-increment enabled by default) communication
//! - Basic initialization and configuration
//! - Temperature and pressure measurement (One-Shot and Continuous/Pulsed modes)
//! - Support for configurable oversampling (P & T) and moving average filter (P)
//! - Basic power/operation mode configuration
//!
//! ⚠️ Not yet implemented:
//! - FIFO readout and detailed configuration
//! - Advanced interrupt configuration (thresholds, FIFO events etc.)
//! - SPI 3-wire mode
//! - Dynamic control of SPI M/S bit (auto-increment always enabled for SPI in this driver)
//! - Ultra-low power mode (HP bit management) beyond basic configuration needs
//!
//! ## Usage (sync) - I2C Example
//!
//! ```rust, only_if(sync)
//! use ens220_driver::{ENS220Sync, address::Address, Configuration, OperationMode}; // Replace ens220_driver
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::si::pressure::pascal;
//! use uom::num_traits::ToPrimitive;
//!
//! let mut config = Configuration::default();
//! config.operation_mode = OperationMode::OneShot; // Configure for a single measurement
//!
//! let mut ens220 = ENS220Sync::new_i2c(i2c, Address::Fixed, config);
//! ens220.init(&mut delay).await?; // Use .await if async, remove if sync
//!
//! let data = ens220.measure(&mut delay).await?; // Use .await if async, remove if sync
//! let temp_c = data.temperature.get::<degree_celsius>().to_f32().unwrap_or(f32::NAN);
//! let press_pa = data.pressure.get::<pascal>().to_f32().unwrap_or(f32::NAN);
//! println!("Temperature: {:.2} °C, Pressure: {:.2} Pa", temp_c, press_pa);
//! # Ok(())
//! # }
//! ```
//!
//! ## Notes
//! - Ensure you call `init()` after creating the driver to reset and configure the sensor.
//! - The `measure()` method will trigger a one-shot measurement if `OperationMode::OneShot` is configured.
//! - For `OperationMode::Continuous` or `OperationMode::Pulsed`, `configure()` sets the mode, and `measure()` reads the latest data. The user should ensure `MODE_CFG.START` is set appropriately after configuration for these modes.

use embedded_devices_derive::{device, device_impl};

use uom::num_rational::Rational32;
use uom::si::pressure::pascal;
use uom::si::rational32::{Pressure, ThermodynamicTemperature};
use uom::si::thermodynamic_temperature::kelvin; // Using Kelvin for internal consistency with datasheet

pub mod address;
pub mod registers;

use self::address::Address;
use self::registers::*;

// For I2C: Standard 1-byte register address codec
type ENS220I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

// For SPI: Command byte [AD5 AD4 AD3 AD2 AD1 AD0 M/S R/W]
// This codec assumes M/S bit (bit 1) is always 0 (auto-increment enabled).
// ADDR_BYTES = 1: Command is one byte.
// ADDR_BITS = 6: Register address AD[5:0].
// ADDR_SHIFT = 2: Address AD[5:0] is in bits 7:2 of the command byte.
// RW_BIT = 0: R/W flag is bit 0.
// MSB_FIRST = true: Command byte and data are MSB first.
// DUMMY_CYCLES = 1: One dummy byte needed for reads before data appears on SDO.
type ENS220SpiCodec = embedded_registers::spi::codecs::SimpleCodec<1, 6, 2, 0, true, 1>;

/// All possible errors that may occur when using this device.
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus communication error.
    Bus(BusError),
    /// Invalid Chip ID was encountered during `init`.
    InvalidChip(u16),
    /// Device reset failed.
    ResetFailed,
    /// Data not ready after timeout (relevant for one-shot measurements).
    DataNotReady,
}

/// Main driver struct for the ENS220 sensor.
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct ENS220Common<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device.
    interface: I,
    /// Sensor configuration.
    pub(super) config: Configuration,
}

/// The ENS220 is an ultra-low-power, high-accuracy barometric pressure and temperature sensor from ScioSense.
/// It is designed for applications such as activity tracking, indoor navigation/localization, and liquid-level detection.
/// The sensor features a compact package and low power consumption, making it suitable for battery-powered devices.
#[cfg(feature = "sync")]
pub type ENS220Sync<I> = ENS220CommonSync<I>;

/// The ENS220 is an ultra-low-power, high-accuracy barometric pressure and temperature sensor from ScioSense.
/// It is designed for applications such as activity tracking, indoor navigation/localization, and liquid-level detection.
/// The sensor features a compact package and low power consumption, making it suitable for battery-powered devices.
#[cfg(feature = "async")]
pub type ENS220Async<I> = ENS220CommonAsync<I>;

/// Structure to hold measurement data from the ENS220.
#[derive(Debug, Clone, Copy)]
pub struct Measurements {
    /// Temperature reading.
    pub temperature: Option<ThermodynamicTemperature>,
    /// Pressure reading.
    pub pressure: Option<Pressure>,
}

/// Configuration for the ENS220 sensor.
#[derive(Debug, Clone)]
pub struct Configuration {
    /// Desired operation mode.
    pub operation_mode: StandbyDuration,
    /// Oversampling setting for temperature measurements.
    pub temperature_oversampling: OversamplingSetting,
    /// Oversampling setting for pressure measurements.
    pub pressure_oversampling: OversamplingSetting,
    /// Moving average filter samples for pressure data.
    pub pressure_moving_average: MovingAverageSamples,
    /// Pressure ADC conversion time setting.
    pub pressure_conv_time: PressureConvTime,
    /// Ratio between pressure and temperature measurements.
    pub pressure_temp_rate: PressureTempRate,
    /// Enable/disable pressure & temperature measurements.
    pub measurement_selection: MeasurementSelection,
    /// Data path selection (Direct, FIFO, MovingAverage).
    /// Note: `pressure_moving_average` setting implies `FifoMode::MovingAverage`.
    /// If FIFO is desired, this should be set to `FifoMode::Fifo`.
    pub fifo_mode_selection: FifoMode,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            operation_mode: StandbyDuration::OneShot,
            temperature_oversampling: OversamplingSetting::Avg1, // Default OVST
            pressure_oversampling: OversamplingSetting::Avg1,    // Default OVSP
            pressure_moving_average: MovingAverageSamples::Samples1, // Default MAVG
            pressure_conv_time: PressureConvTime::Ms8_2,         // Default P_CONV in MEAS_CFG
            pressure_temp_rate: PressureTempRate::Rate1,         // Default PT_RATE in MEAS_CFG
            measurement_selection: MeasurementSelection::PressureAndTemperature, // Default MEAS_P and MEAS_T in MODE_CFG
            fifo_mode_selection: FifoMode::DirectPath,                           // Default FIFO_MODE in MODE_CFG
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> ENS220<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, ENS220I2cCodec>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new ENS220 device with the given I²C address and configuration.
    /// The ENS220 has a fixed I²C address of 0x20.
    ///
    /// Call `init()` to perform sensor reset and apply initial configuration.
    #[inline]
    pub fn new_i2c(interface: I, address: Address, config: Configuration) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
            config,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> ENS220Common<embedded_registers::spi::SpiDevice<I, ENS220SpiCodec>>
where
    I: hal::spi::r#SpiDevice,
{
    /// Initializes a new ENS220 device from the specified SPI device and configuration.
    ///
    /// Call `init()` to perform sensor reset and apply initial configuration.
    #[inline]
    pub fn new_spi(interface: I, config: Configuration) -> Self {
        Self {
            interface: embedded_registers::spi::SpiDevice::new(interface),
            config,
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> ENS220Common<I> {
    /// Initializes the ENS220 sensor.
    /// This performs a soft-reset, verifies the chip ID, and applies the stored configuration.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        self.soft_reset(delay).await?;

        // Check the device ID to ensure we are communicating with the correct chip.
        // let part_id_reg = self.read_register::<PartId>().await.map_err(Error::Bus)?;
        // if part_id_reg.read_id() != address::DEVICE_IDENTIFIER {
        //     return Err(Error::InvalidChip(part_id_reg.read_id()));
        // }

        // Set the initial configuration after reset.
        self.configure(&self.config.clone(), delay).await?;

        Ok(())
    }

    /// Performs a soft-reset of the device.
    /// Waits for a short period for the reset to complete.
    pub async fn soft_reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        // Suggested reset sequence part 1: set RESET=1, HP=0. Value 0x08 for MODE_CFG.
        // This also sets START=0, MEAS_T=0, MEAS_P=0, FIFO_MODE=00.
        self.write_register(
            ModeCfg::default()
                .with_high_power(false)
                .with_fifo_mode(FifoMode::DirectPath)
                .with_start(false)
                .with_reset(true)
                .with_measurement_selection(MeasurementSelection::PressureAndTemperature),
        )
        .await
        .map_err(Error::Bus)?;

        // Datasheet suggests 0.5ms after setting RESET. Using 2ms for safety.
        delay.delay_ms(2).await;

        // RESET bit is auto-cleared.
        Ok(())
    }

    /// Set sensor to idle mode.
    /// This is typically done by setting `MODE_CFG.START = 0`.
    /// This method is useful before reconfiguring the sensor or performing a reset.
    /// Changes Power Mode to High Power (HP=1) if not already set, to ensure all registers are writable.
    pub async fn set_idle<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        // Read the current mode configuration.
        let mut mode_cfg = self.read_register::<ModeCfg>().await.map_err(Error::Bus)?;

        // Update required fields for idle mode.
        mode_cfg.write_high_power(true);
        mode_cfg.write_start(false);

        // Set the new configuration.
        self.write_register(mode_cfg).await.map_err(Error::Bus)?;

        // Short delay after enabling High Power mode (0.5ms required)
        delay.delay_us(500).await;

        Ok(())
    }

    /// Configures the sensor with the provided settings.
    pub async fn configure<D: hal::delay::DelayNs>(
        &mut self,
        config: &Configuration,
        delay: &mut D,
    ) -> Result<(), Error<I::Error>> {
        // Ensure the sensor is in idle mode before applying configuration.
        self.set_idle(delay).await?;

        // Configure MEAS_CFG: Pressure conversion time and P/T rate
        self.write_register(
            MeasCfg::default()
                .with_p_conv(config.pressure_conv_time)
                .with_pt_rate(config.pressure_temp_rate),
        )
        .await
        .map_err(Error::Bus)?;

        // Configure OVS_CFG: Oversampling for P and T
        self.write_register(
            OvsCfg::default()
                .with_ovsp(config.pressure_oversampling)
                .with_ovst(config.temperature_oversampling),
        )
        .await
        .map_err(Error::Bus)?;

        // Configure MAVG_CFG: Moving average for P
        self.write_register(MavgCfg::default().with_mavg(config.pressure_moving_average))
            .await
            .map_err(Error::Bus)?;

        // Configure STBY_CFG: Standby duration (operation mode)
        self.write_register(StandbyCfg::default().with_standby_duration(config.operation_mode))
            .await
            .map_err(Error::Bus)?;

        // Configure MODE_CFG: Measurement enables, FIFO mode (HP and START handled separately)
        let mut mode_cfg = self.read_register::<ModeCfg>().await.map_err(Error::Bus)?; // Read current HP, START, RESET state
        mode_cfg.write_measurement_selection(config.measurement_selection);
        mode_cfg.write_fifo_mode(config.fifo_mode_selection);

        if config.operation_mode != StandbyDuration::OneShot {
            mode_cfg.write_high_power(true);
            mode_cfg.write_start(true);
        }

        // START bit is not set here; user should set it to begin continuous/pulsed measurements.
        // For OneShot, measure() will set START.
        self.write_register(mode_cfg).await.map_err(Error::Bus)?;

        // Store the applied configuration
        self.config = config.clone();

        Ok(())
    }

    /// Performs a measurement.
    /// - If mode is `OneShot`, triggers a measurement, waits, and returns data.
    /// - If mode is `Continuous` or `Pulsed`, reads the latest available data.
    ///   (Assumes `MODE_CFG.START = 1` has been set by the user after `configure` for these modes).
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        // If not in continuous or pulsed mode, set a single measurement.
        if self.config.operation_mode == StandbyDuration::OneShot {
            // Start a single measurement
            let mut mode_cfg = self.read_register::<ModeCfg>().await.map_err(Error::Bus)?;
            mode_cfg.write_start(true);
            self.write_register(mode_cfg).await.map_err(Error::Bus)?;

            // Calculate and wait for measurement duration
            let wait_us = self.calculate_measurement_delay_us();
            delay.delay_us(wait_us as u32).await; // Cast to u32, ensure it fits

            // Check DATA_STAT register for data ready flags
            let mut data_ready = false;
            let max_retries = 10; // Max retries for checking data ready status

            for _ in 0..max_retries {
                let data_stat = self.read_register::<DataStat>().await.map_err(Error::Bus)?;
                
                data_ready = match self.config.measurement_selection {
                    MeasurementSelection::PressureAndTemperature => {
                        data_stat.read_pressure_ready() && data_stat.read_temp_ready()
                    }
                    MeasurementSelection::PressureOnly => data_stat.read_pressure_ready(),
                    MeasurementSelection::TemperatureOnly => data_stat.read_temp_ready(),
                };

                if data_ready {
                    break;
                }

                delay.delay_ms(1).await; // Short delay before retrying
            }

            if !data_ready
            {
                return Err(Error::DataNotReady);
            }
        }

        self.read_sensor_data().await
    }

    /// Helper function to calculate estimated delay for a P+T measurement cycle.
    fn calculate_measurement_delay_us(&self) -> u64 {
        let mut total_delay_us: u64 = 0;

        if self.config.measurement_selection == MeasurementSelection::PressureAndTemperature || self.config.measurement_selection == MeasurementSelection::PressureOnly {
            let p_conv_next_us: u64 = match self.config.pressure_conv_time {
                PressureConvTime::Ms4_1 => 1_000,  // Next conversion 1ms
                PressureConvTime::Ms8_2 => 2_000,  // Next conversion 2ms
                PressureConvTime::Ms16_4 => 4_000, // Next conversion 4ms
            };
            let ovsp_factor: u64 = 1 << self.config.pressure_oversampling as u8; // 2^OVSP
            // Formula from Figure 6 interpretation: (3 + 2^OVSP) * t_P_next
            // The '3 * t_P_next' part accounts for the "first conversion" overhead.
            total_delay_us += (3 + ovsp_factor) * p_conv_next_us;
        }

        if self.config.measurement_selection == MeasurementSelection::PressureAndTemperature || self.config.measurement_selection == MeasurementSelection::TemperatureOnly {
            let t_conv_next_us: u64 = 1_000; // Additional temp conversion takes 1ms 
            let ovst_factor: u64 = 1 << self.config.temperature_oversampling as u8; // 2^OVST
            // Formula from Figure 10 interpretation: (3 + 2^OVST) * t_T_next_equivalent
            // The '3 * t_T_next_equivalent' part accounts for base conversion time (approx 4ms)
            total_delay_us += (3 + ovst_factor) * t_conv_next_us;
        }

        total_delay_us
    }

    /// Helper to read and convert pressure and temperature data.
    async fn read_sensor_data(&mut self) -> Result<Measurements, Error<I::Error>> {
        let pressure = if self.config.measurement_selection == MeasurementSelection::PressureAndTemperature || self.config.measurement_selection == MeasurementSelection::PressureOnly {
            let press_out = self.read_register::<PressOut>().await.map_err(Error::Bus)?;
            let press_val_raw = press_out.read_pressure_val();
            // Pressure: val / 64.0 (raw is in 1/64 Pa)
            Some(Pressure::new::<pascal>(Rational32::new_raw(press_val_raw as i32, 64)))
        } else {
            None
        };
        
        let temperature = if self.config.measurement_selection == MeasurementSelection::PressureAndTemperature || self.config.measurement_selection == MeasurementSelection::TemperatureOnly {
            let temp_out = self.read_register::<TempOut>().await.map_err(Error::Bus)?;
            let temp_val_raw = temp_out.read_temperature_val();
            // Temperature: val / 128.0 (raw is in 1/128 K)
            Some(ThermodynamicTemperature::new::<kelvin>(Rational32::new_raw(temp_val_raw as i32, 128)))
        } else {
            None
        };
        
        Ok(Measurements { temperature, pressure })
    }
}
