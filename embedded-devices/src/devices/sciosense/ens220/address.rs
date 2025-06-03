use defmt::Format;

/// Device identifier for the ENS220.
pub const DEVICE_IDENTIFIER: u16 = 0x220;

/// The fixed I²C slave address for the ENS220.
const PRIMARY_ADDRESS: u8 = 0x20;

/// Represents the I²C address for the ENS220 sensor.
///
/// The ENS220 has a fixed 7-bit I²C slave address of 0x20[cite: 130].
/// Interface selection (I²C/SPI) is controlled by the CSN pin[cite: 124].
/// For I²C operation, CSN should be tied high (to VDD)[cite: 125, 293, 301].
#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum Address {
    /// The fixed I²C address for the ENS220 (0x20).
    Primary,
    /// Allows for a custom address, for scenarios such as using an I²C address translator,
    /// though the ENS220 device itself only responds to 0x20.
    Custom(u8),
}

impl From<Address> for u8 {
    /// Converts the `Address` enum to its 7-bit `u8` representation.
    fn from(address: Address) -> Self {
        match address {
            Address::Primary => PRIMARY_ADDRESS,
            Address::Custom(x) => x,
        }
    }
}
