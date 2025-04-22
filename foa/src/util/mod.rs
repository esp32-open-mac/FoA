//! This module contains utilities, that aren't directly part of Ferris-on-Air, but can be of use
//! when implementing an interface.
//!
//! For the documentation of specific utilities, please refer to their module level documentations.

use esp_hal::rng::Rng;

pub mod operations;
pub mod rx_router;

/// Generate a random valid MAC address.
pub fn random_mac_address() -> [u8; 6] {
    let mut mac_address = [0u8; 6];
    Rng::new().read(&mut mac_address);
    // Clear the group bit and set the locally administered bit.
    mac_address[0] &= !(1);
    mac_address[0] |= 2;

    mac_address
}
