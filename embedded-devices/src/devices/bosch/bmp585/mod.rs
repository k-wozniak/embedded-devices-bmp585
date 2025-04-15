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
    /// The calibration data was not yet read from the device, but a measurement was requested. Call `init` or `calibrate` first.
    NotCalibrated,
    /// NVM data copy is still in progress.
    ResetFailed,
}

/// Fine temperature coefficient calculated when compensating temperature
/// and required to compensate pressure
#[derive(Debug, Clone, Copy)]
pub(super) struct TFine(i32);

// #[derive(Debug)]
// pub(super) struct CalibrationData(TrimmingCoefficientsBitfield);

// impl CalibrationData {
//     pub(super) fn compensate_temperature(&self, uncompensated: u32) -> (ThermodynamicTemperature, TFine) {
//         let v1: u64 = (uncompensated as u64) - ((self.0.par_t1 as u64) << 8);
//         let v2: u64 = self.0.par_t2 as u64 * v1;
//         let v3: u64 = v1 * v1;
//         let v4: i64 = (v3 as i64) * (self.0.par_t3 as i64);
//         let v5: i64 = ((v2 as i64) << 18) + v4;
//         let t_fine: i32 = (v5 >> 32) as i32;
//         let temperature = Rational32::new_raw(t_fine * 25, 100 << 14);

//         (
//             ThermodynamicTemperature::new::<degree_celsius>(temperature),
//             TFine(t_fine),
//         )
//     }

//     pub(super) fn compensate_pressure(&self, uncompensated: u32, t_fine: TFine) -> Pressure {
//         let t_fine = t_fine.0;

//         let v1 = t_fine as i64 * t_fine as i64;
//         let v3 = ((v1 >> 6) * t_fine as i64) >> 8;
//         let v4 = (self.0.par_p8 as i64 * v3) >> 5;
//         let v5 = (self.0.par_p7 as i64 * v1) << 4;
//         let v6 = (self.0.par_p6 as i64 * t_fine as i64) << 22;
//         let offset = ((self.0.par_p5 as i64) << 47) + v4 + v5 + v6;
//         let v2 = ((self.0.par_p4 as i64) * v3) >> 5;
//         let v4 = (self.0.par_p3 as i64 * v1) << 2;
//         let v5 = ((self.0.par_p2 as i64 - 16384) * t_fine as i64) << 21;
//         let sensitivity = (((self.0.par_p1 as i64 - 16384) << 46) + v2 + v4 + v5) >> 24;
//         let v1 = (sensitivity * uncompensated as i64) >> 13;
//         let v2 = (self.0.par_p10 as i64) * (t_fine as i64);
//         let v3 = v2 + ((self.0.par_p9 as i64) << 16);
//         let v4 = (v3 * uncompensated as i64) >> 13;
//         let v5 = (v4 * uncompensated as i64) >> 9;
//         let v6 = uncompensated as i64 * uncompensated as i64;
//         let v2 = ((self.0.par_p11 as i64) * v6) >> 16;
//         let v3 = (v2 * uncompensated as i64) >> 7;
//         let v4 = (offset / 4) + v1 + v5 + v3;
//         let pressure = ((v4 >> 32) * 25) as i32;
//         // TODO this is wrong ^------------
//         // actually pressure in 0.01° = (v4 * 25) >> 40, but we save some extra precision
//         // by only shifting by 32 and doing / 256 in the denominator
//         let pressure = Rational32::new_raw(pressure, 100 << 8);
//         Pressure::new::<pascal>(pressure)
//     }
// }

// TODO: Add docs
#[device]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct BMP585<I: embedded_registers::RegisterInterface> {
    /// The interface to communicate with the device
    interface: I, // Calibration data
                  // pub(super) calibration_data: Option<CalibrationData>,
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
            //calibration_data: None,
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
            //calibration_data: None,
        }
    }
}

#[device_impl]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I: embedded_registers::RegisterInterface> BMP585<I> {}
