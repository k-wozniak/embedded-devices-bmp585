//! # BMP585
//!
//! The BMP585 is a high-performance barometric pressure sensor from Bosch Sensortec. It provides absolute pressure and
//! temperature measurements with outstanding accuracy and ultra-low noise. It is housed in a compact 8-pin metal-lid LGA
//! package (3.25 × 3.25 mm², max height 1.96 mm), designed for applications like indoor navigation, drones, wearables,
//! and outdoor sports equipment.
//!
//! ## Features
//!
//! - Absolute pressure range: 30 … 125 kPa
//! - Temperature range: -40 … +85 °C
//! - Relative pressure accuracy: ±6 Pa (typ)
//! - Absolute pressure accuracy: ±30 Pa (typ)
//! - Low noise: down to 0.08 Pa RMS with high oversampling
//! - Power-efficient: down to 1.3 μA at 1 Hz (lowest power mode)
//! - Configurable IIR filter, FIFO buffer, and interrupt functionality
//! - Supported interfaces: I²C, SPI (3- or 4-wire)
//!
//! ## Current Implementation Status
//!
//! ✅ Implemented:
//! - I²C and SPI (4-wire) communication
//! - Basic initialization and configuration
//! - Temperature and pressure measurement
//! - Support for configurable oversampling and IIR filter
//! - Power modes (Standby, Forced, Normal, Continuous)
//!
//! ⚠️ Not yet implemented:
//! - FIFO readout and configuration
//! - Advanced interrupt configuration (data ready, threshold, out-of-range, FIFO full)
//! - Register paging (only default page 0 is currently supported)
//! - NVM read/write
//! - I3C interface
//! - Out-of-range detection
//!
//! ## Usage (sync)
//!
//! ```rust, only_if(sync)
//! # fn example<I, D>(i2c: I, mut delay: D) -> Result<(), embedded_devices::devices::bosch::bmp585::Error<I::Error>>
//! # where
//! #   I: embedded_hal::i2c::I2c + embedded_hal::i2c::ErrorType,
//! #   D: embedded_hal::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp585::{BMP585Sync, address::Address, Configuration};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! let config = Configuration::default();
//! let mut bmp585 = BMP585Sync::new_i2c(i2c, Address::Primary, config);
//! bmp585.init(&mut delay)?;
//!
//! let data = bmp585.measure(&mut delay)?;
//! let temp_c = data.temperature.get::<degree_celsius>().to_f32();
//! println!("Temperature: {} °C", temp_c);
//! # Ok(())
//! # }
//! ```
//!
//! ## Usage (async)
//!
//! ```rust, only_if(async)
//! # async fn example<I, D>(i2c: I, mut delay: D) -> Result<(), embedded_devices::devices::bosch::bmp585::Error<I::Error>>
//! # where
//! #   I: embedded_hal_async::i2c::I2c + embedded_hal_async::i2c::ErrorType,
//! #   D: embedded_hal_async::delay::DelayNs
//! # {
//! use embedded_devices::devices::bosch::bmp585::{BMP585Async, address::Address, Configuration};
//! use uom::si::thermodynamic_temperature::degree_celsius;
//! use uom::num_traits::ToPrimitive;
//!
//! let config = Configuration::default();
//! let mut bmp585 = BMP585Async::new_i2c(i2c, Address::Primary, config);
//! bmp585.init(&mut delay).await?;
//!
//! let data = bmp585.measure(&mut delay).await?;
//! let temp_c = data.temperature.get::<degree_celsius>().to_f32();
//! println!("Temperature: {} °C", temp_c);
//! # Ok(())
//! # }
//! ```
//!
//! ## Notes
//! - Ensure you call `init()` after creating the driver to reset and configure the sensor.
//! - Measurements in Standby mode trigger a one-shot Forced mode conversion.
//! - Use appropriate oversampling settings for your accuracy vs power tradeoff.

use embedded_devices_derive::{device, device_impl};
use uom::num_rational::Rational32;
use uom::si::pressure::pascal;
use uom::si::rational32::{Pressure, ThermodynamicTemperature};
use uom::si::thermodynamic_temperature::degree_celsius;

pub mod address;
pub mod registers;

use self::address::Address;
use self::registers::*;

// TODO: Confirm these are valid values.
type BMP585SpiCodec = embedded_registers::spi::codecs::SimpleCodec<1, 6, 0, 7, true, 1>;
type BMP585I2cCodec = embedded_registers::i2c::codecs::OneByteRegAddrCodec;

/// All possible errors that may occur when using this device
/// TODO: Most likely not all errors are covered here and not the same.
#[derive(Debug, defmt::Format)]
pub enum Error<BusError> {
    /// Bus error
    Bus(BusError),
    /// Invalid ChipId was encountered in `init`
    InvalidChip(Chip),
    /// Status register indicates that the device is not calibrated.
    BadStatus,
    /// NVM data copy is still in progress.
    ResetFailed,
    /// Invalid OSR/ODR configuration.
    InvalidConfig,
}

// TODO: Add docs
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BMP585<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I,

    /// The configuration the sensor should use during the init.
    config: Configuration,
}

/// Measurement data
#[derive(Debug)]
pub struct Measurements {
    /// Current temperature
    pub temperature: ThermodynamicTemperature,
    /// Current pressure
    pub pressure: Pressure,
}

/// Common configuration values for the BMP585 sensor.
/// The power-on-reset default is to set all oversampling settings to 1X
/// and use no IIR filter.
#[derive(Debug, Clone)]
pub struct Configuration {
    /// Output data rate configuration and type
    pub odr: OdrConfig,
    /// The oversampling rate for temperature mesurements
    pub temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure mesurements
    pub pressure_oversampling: Oversampling,
    /// Pressure IIR filter to use
    pub pressure_iir: IIRFilter,
    /// Temperature IIR filter to use
    pub temperature_iir: IIRFilter,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            odr: OdrConfig::default(),
            temperature_oversampling: Oversampling::X1,
            pressure_oversampling: Oversampling::X1,
            pressure_iir: IIRFilter::Disabled,
            temperature_iir: IIRFilter::Disabled,
        }
    }
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), I2cDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I> BMP585<embedded_registers::i2c::I2cDevice<I, hal::i2c::SevenBitAddress, BMP585I2cCodec>>
where
    I: hal::i2c::I2c<hal::i2c::SevenBitAddress> + hal::i2c::ErrorType,
{
    /// Initializes a new device with the given address on the specified bus.
    /// This consumes the I2C bus `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
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
impl<I> BMP585<embedded_registers::spi::SpiDevice<I, BMP585SpiCodec>>
where
    I: hal::spi::r#SpiDevice,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// Before using this device, you must call the [`Self::init`] method which
    /// initializes the device and ensures that it is working correctly.
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
impl<I: embedded_registers::RegisterInterface> BMP585<I> {
    /// Initialize the sensor by performing a soft-reset, verifying its chip ID and reading calibration data.
    pub async fn init<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        // Soft-reset device.
        self.reset(delay).await?;

        // Verify chip ID.
        let chip = self.read_register::<ChipId>().await.map_err(Error::Bus)?.read_chip();
        if let self::registers::Chip::Invalid(_) = chip {
            return Err(Error::InvalidChip(chip));
        }

        // Check the NVM status.
        let status = self.read_register::<Status>().await.map_err(Error::Bus)?;
        if !status.read_nvm_ready() && status.read_nvm_error() {
            return Err(Error::ResetFailed);
        }

        // Configure the sensor.
        let config = self.config.clone();
        self.configure::<D>(&config).await?;

        Ok(())
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 10ms, which is automatically awaited before allowing further communication.
    ///
    /// This will try resetting up to 5 times in case of an error.
    pub async fn reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        const TRIES: u8 = 5;
        for _ in 0..TRIES {
            match self.try_reset(delay).await {
                Ok(()) => return Ok(()),
                Err(Error::<I::Error>::ResetFailed) => continue,
                Err(e) => return Err(e),
            }
        }

        Err(Error::<I::Error>::ResetFailed)
    }

    /// Performs a soft-reset of the device. The datasheet specifies a start-up time
    /// of 10ms, which is automatically awaited before allowing further communication.
    ///
    /// This will check the status register for success, returning an error otherwise.
    pub async fn try_reset<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<I::Error>> {
        self.write_register(self::registers::Command::default().with_command(Cmd::Reset))
            .await
            .map_err(Error::Bus)?;

        // Table 4 & 4.3.10 specify to wait 2 ms after soft reset.
        delay.delay_ms(2).await;

        if !self
            .read_register::<self::registers::InterruptStatus>()
            .await
            .map_err(Error::Bus)?
            .read_reset_occurred()
        {
            return Err(Error::ResetFailed);
        }

        Ok(())
    }

    /// Configures common sensor settings. Sensor must be in sleep mode for this to work.
    /// Check sensor mode beforehand and call [`Self::reset`] if necessary. To configure
    /// advanced settings, please directly update the respective registers.
    pub async fn configure<D: hal::delay::DelayNs>(&mut self, config: &Configuration) -> Result<(), Error<I::Error>> {
        // Enter standby mode to configure the sensor.
        self.write_register(OdrConfig::default().with_power_mode(PowerMode::Standby))
            .await
            .map_err(Error::Bus)?;

        self.write_register(
            OversamplingConfig::default()
                .with_temperature(config.temperature_oversampling)
                .with_pressure(config.pressure_oversampling),
        )
        .await
        .map_err(Error::Bus)?;

        self.write_register(
            IIRFilterConfig::default()
                .with_pressure_iir(config.pressure_iir)
                .with_temperature_iir(config.temperature_iir),
        )
        .await
        .map_err(Error::Bus)?;

        // Set the power mode.
        self.write_register(config.odr).await.map_err(Error::Bus)?;

        // Check if OSR-ODR configuration is valid.
        let eff = self.read_register::<OsrEffective>().await.map_err(Error::Bus)?;
        if !eff.read_odr_is_valid() {
            return Err(Error::InvalidConfig);
        }

        Ok(())
    }

    /// Performs a measurement depending on current mode.
    /// It assumes the device was previously configured and is in the correct mode.
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        match self.config.odr.read_power_mode() {
            // If in Standby, trigger a new measurement using Forced mode.
            PowerMode::Standby | PowerMode::Forced => self.force_read(delay).await,

            // If in Normal or Continuous mode, read the latest available data.
            PowerMode::Normal | PowerMode::Continous => self.normal_read().await,
        }
    }

    async fn force_read<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        self.write_register(OdrConfig::default().with_power_mode(PowerMode::Forced))
            .await
            .map_err(Error::Bus)?;

        // Read current oversampling config to determine required measurement delay
        let oversample_temp_us = self.config.temperature_oversampling.pressure_conversion_time_();
        let oversample_press_us = self.config.pressure_oversampling.pressure_conversion_time_();

        // Maximum time required to perform the measurement.
        delay.delay_us(oversample_temp_us + oversample_press_us).await;

        // Read the latest available data.
        self.normal_read().await
    }

    async fn normal_read(&mut self) -> Result<Measurements, Error<I::Error>> {
        // Read the latest available data.
        let raw = self
            .read_register::<registers::Measurement>()
            .await
            .map_err(Error::Bus)?;

        let temperature = Rational32::new_raw(raw.read_temperature() as i32, 1 << 16);
        let pressure = Rational32::new_raw(raw.read_pressure() as i32, 1 << 6);

        Ok(Measurements {
            temperature: ThermodynamicTemperature::new::<degree_celsius>(temperature),
            pressure: Pressure::new::<pascal>(pressure),
        })
    }
}
