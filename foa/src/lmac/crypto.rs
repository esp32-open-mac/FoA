use core::cell::RefCell;

use embassy_sync::blocking_mutex;
use esp_wifi_hal::prelude::*;
use ieee80211::macro_bits::{bit, check_bit};
/// A cryptographic key slot.
pub struct KeySlot<'res> {
    pub(crate) key_slot_manager: &'res blocking_mutex::NoopMutex<RefCell<KeySlotManager>>,
    pub(crate) key_slot: u8,
    pub(crate) interface: u8,
}
impl KeySlot<'_> {
    /// Get the underlying hardware key slot.
    pub const fn key_slot(&self) -> usize {
        self.key_slot as usize
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
    ) -> Result<(), CryptoError> {
        self.key_slot_manager.lock(|ref_cell| {
            ref_cell.borrow_mut().crypto_controller.set_key(
                self.key_slot as _,
                self.interface as _,
                key_id,
                address,
                cipher_parameters,
            )
        })
    }
}
impl Drop for KeySlot<'_> {
    fn drop(&mut self) {
        self.key_slot_manager.lock(|ref_cell| {
            let mut key_slot_manager = ref_cell.borrow_mut();
            let _ = key_slot_manager
                .crypto_controller
                .delete_key(self.key_slot());
            key_slot_manager.release_key_slot(self.key_slot());
        });
        debug!("Key Slot {} was released.", self.key_slot);
    }
}
pub(crate) struct KeySlotManager {
    /// A bit mask indicating, which slots are free.
    key_slot_state: u32,
    /// Crypto controller of the Wi-Fi driver.
    pub(crate) crypto_controller: CryptoController<'static>,
}
impl KeySlotManager {
    /// Create a new key slot manager.
    pub const fn new(crypto_controller: CryptoController<'static>) -> Self {
        Self {
            key_slot_state: u32::MAX,
            crypto_controller,
        }
    }
    /// Acquire a free key slot.
    pub fn acquire_key_slot(&mut self) -> Option<usize> {
        // We don't have to worry about race conditions here, since this function is sync and the
        // entire stack runs on one core.
        (0..KEY_SLOT_COUNT).find(|i| check_bit!(self.key_slot_state, bit!(i)))
            .inspect(|key_slot| {
                self.key_slot_state &= !bit!(*key_slot) as u32;
            })
    }
    /// Release a key slot.
    fn release_key_slot(&mut self, key_slot: usize) {
        self.key_slot_state |= bit!(key_slot) as u32;
        let _ = self.crypto_controller.delete_key(key_slot);
    }
}
