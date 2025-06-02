use defmt::Format;

/// The fixed I²C slave address for the ENS220.
const FIXED_ADDRESS: u8 = 0x20;

/// Represents the I²C address for the ENS220 sensor.
///
/// The ENS220 has a fixed 7-bit I²C slave address of 0x20[cite: 130].
/// Interface selection (I²C/SPI) is controlled by the CSN pin[cite: 124].
/// For I²C operation, CSN should be tied high (to VDD)[cite: 125, 293, 301].
#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum Address {
    /// The fixed I²C address for the ENS220 (0x20).
    Fixed,
    /// Allows for a custom address, for scenarios such as using an I²C address translator,
    /// though the ENS220 device itself only responds to 0x20.
    Custom(u8),
}

impl Default for Address {
    /// Returns the default fixed address for the ENS220.
    fn default() -> Self {
        Address::Fixed
    }
}

impl From<Address> for u8 {
    /// Converts the `Address` enum to its 7-bit `u8` representation.
    fn from(address: Address) -> Self {
        match address {
            Address::Fixed => FIXED_ADDRESS,
            Address::Custom(x) => x,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_address_is_fixed() {
        assert_eq!(Address::default(), Address::Fixed);
    }

    #[test]
    fn fixed_address_to_u8() {
        let addr: u8 = Address::Fixed.into();
        assert_eq!(addr, FIXED_ADDRESS);
        assert_eq!(addr, 0x20);
    }

    #[test]
    fn custom_address_to_u8() {
        let custom_val: u8 = 0x77;
        let addr: u8 = Address::Custom(custom_val).into();
        assert_eq!(addr, custom_val);
    }
}