use core::{sync::atomic::Ordering, usize};

use esp_wifi_hal::{CipherParameters, WiFi, WiFiResult};
use ieee80211::macro_bits::{bit, check_bit};
use portable_atomic::AtomicUsize;

use super::SharedLMacState;

/// A cryptographic key slot.
pub struct KeySlot<'res> {
    pub(crate) shared_state: &'res SharedLMacState,
    pub(crate) key_slot: u8,
    pub(crate) interface: u8,
}
impl KeySlot<'_> {
    /// Get the underlying hardware key slot.
    pub const fn key_slot(&self) -> u8 {
        self.key_slot
    }
    /// Set the key used by this slot.
    ///
    /// This is a passthrough for [WiFi::set_key], refer to it's documentation for further
    /// information.
    pub fn set_key(
        &mut self,
        key_id: u8,
        address: [u8; 6],
        cipher_parameters: CipherParameters<'_>,
    ) -> WiFiResult<()> {
        self.shared_state.wifi.set_key(
            self.key_slot as usize,
            self.interface as usize,
            key_id,
            address,
            cipher_parameters,
        )
    }
}
impl Drop for KeySlot<'_> {
    fn drop(&mut self) {
        let _ = self.shared_state.wifi.delete_key(self.key_slot as usize);
        self.shared_state.key_slot_manager.release_key_slot(self.key_slot);
        debug!("Key Slot {} was released.", self.key_slot);
    }
}
pub struct KeySlotManager {
    /// A bit mask indicating, which slots are free.
    key_slot_state: AtomicUsize,
}
impl KeySlotManager {
    /// Create a new key slot manager.
    pub const fn new() -> Self {
        Self {
            key_slot_state: AtomicUsize::new(usize::MAX)
        }
    }
    /// Acquire a free key slot.
    pub fn acquire_key_slot(&self) -> Option<u8> {
        // We don't have to worry about race conditions here, since this function is sync and the
        // entire stack runs on one core.
        let key_slot_state = self.key_slot_state.load(Ordering::Relaxed);
        (0..WiFi::KEY_SLOT_COUNT as u8)
            .filter(|i| check_bit!(key_slot_state, bit!(i)))
            .next()
            .inspect(|key_slot| {
                self.key_slot_state
                    .store(key_slot_state & bit!(key_slot), Ordering::Relaxed);
            })
    }
    /// Release a key slot.
    fn release_key_slot(&self, key_slot: u8) {
        self.key_slot_state.fetch_or(bit!(key_slot), Ordering::Relaxed);
    }
}
