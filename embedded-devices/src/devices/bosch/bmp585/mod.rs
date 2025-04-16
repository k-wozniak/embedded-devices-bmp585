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
    /// The oversampling rate for temperature mesurements
    pub temperature_oversampling: Oversampling,
    /// The oversampling rate for pressure mesurements
    pub pressure_oversampling: Oversampling,
    /// The iir filter to use
    pub iir_filter: IIRFilter,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            temperature_oversampling: Oversampling::X1,
            pressure_oversampling: Oversampling::X1,
            iir_filter: IIRFilter::Disabled,
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
    pub fn new_i2c(interface: I, address: Address) -> Self {
        Self {
            interface: embedded_registers::i2c::I2cDevice::new(interface, address.into()),
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
    pub fn new_spi(interface: I) -> Self {
        Self {
            interface: embedded_registers::spi::SpiDevice::new(interface),
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
        delay.delay_ms(10).await;

        // todo: C code indicates a 2 ms delay after reset.
        // todo: C code performs a read of the chip ID (most likely to clear the buffer)

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
        self.write_register(
            OversamplingConfig::default()
                .with_temperature(config.temperature_oversampling)
                .with_pressure(config.pressure_oversampling),
        )
        .await
        .map_err(Error::Bus)?;

        self.write_register(
            IIRFilterConfig::default()
                .with_pressure_iir(config.iir_filter)
                .with_temperature_iir(config.iir_filter), // todo: Separate.
        )
        .await
        .map_err(Error::Bus)?;

        Ok(())
    }

    /// Performs a one-shot measurement. This will transition the device into forced mode,
    /// which will cause it to take a measurement and return to sleep mode afterwards.
    ///
    /// This function will wait until the data is acquired, perform a burst read
    /// and compensate the returned raw data using the calibration data.
    pub async fn measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>> {
        self.write_register(
            PowerControl::default()
                .with_sensor_mode(SensorMode::Forced)
                .with_temperature_enable(true)
                .with_pressure_enable(true),
        )
        .await
        .map_err(Error::Bus)?;

        // Read current oversampling config to determine required measurement delay
        let reg_ctrl_m = self.read_register::<OversamplingControl>().await.map_err(Error::Bus)?;
        let o_t = reg_ctrl_m.read_temperature_oversampling();
        let o_p = reg_ctrl_m.read_pressure_oversampling();

        // Maximum time required to perform the measurement.
        // See section 3.9.2 of the datasheet for more information.
        let max_measurement_delay_us = 234 + (392 + 2020 * o_p.factor()) + (163 + o_t.factor() * 2020);
        delay.delay_us(max_measurement_delay_us).await;

        let raw_data = self
            .read_register::<BurstMeasurements>()
            .await
            .map_err(Error::Bus)?
            .read_all();
        let Some(ref cal) = self.calibration_data else {
            return Err(Error::NotCalibrated);
        };

        let (temperature, t_fine) = raw_data.temperature.temperature;
        let pressure = raw_data.pressure.pressure;

        Ok(Measurements { temperature, pressure })
    }

    pub async fn force_measure<D: hal::delay::DelayNs>(&mut self, delay: &mut D) -> Result<Measurements, Error<I::Error>>{
        Ok(Measurements { 0.0, 0.0 })
    }
}
