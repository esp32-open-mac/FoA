//! This module implements the lower MAC (LMAC).
//!
//! The LMAC's job is it to divide access to the medium between all interfaces.
//! This achieved, by allowing an interface to lock the channel, on which the stack operates. This
//! prevents the other interface from changing the channel. If both interfaces attempt to lock the
//! same channel, they'll both have the lock on the channel and neither can change it, without the
//! other unlocking the channel beforehand. This is done through the [LMacInterfaceControl::lock_channel]
//! and [LMacInterfaceControl::unlock_channel] APIs.
//!
//! The issue with this design is, that it makes scanning impossible, if the channel is already
//! locked by another interfaces. To fix this, off channel operations are implemented, which allow
//! an interface to request permission from the other interface to temporarily change the channel
//! and scanning mode. This request maybe granted or rejected, by the other interface. To request
//! an off channel operation, the [LMacInterfaceControl::begin_interface_off_channel_operation] API
//! can be used. If the other interface currently holds the channel lock, it will receive an [OffChannelRequest] from [LMacInterfaceControl::wait_for_off_channel_request].
//! Which can then be granted or rejected.
use core::{
    array,
    cell::RefCell,
};

use crypto::KeySlotManager;
use embassy_futures::join::join_array;
use embassy_sync::{
    blocking_mutex::{self, raw::NoopRawMutex},
    mutex::Mutex,
    watch::{DynReceiver, Watch},
};
use esp_hal::efuse::Efuse;
use esp_wifi_hal::{
    BorrowedBuffer, RxFilterBank, ScanningMode, TxErrorBehaviour, TxParameters,
    WiFi, WiFiRate, WiFiResult,
};
use off_channel::OffChannelRequester;
pub use off_channel::{OffChannelOperation, OffChannelRequest};
pub use crypto::KeySlot;

use crate::tx_buffer_management::{DynTxBufferManager, TxBuffer};

mod off_channel;
mod crypto;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// The status of the channel lock.
struct ChannelState {
    locks: Option<([bool; WiFi::INTERFACE_COUNT], u8)>,
    off_channel_operation_interface: Option<usize>,
}
impl ChannelState {
    /// Try to lock the channel for the specified interface.
    ///
    /// Returns `true` if the channel changed.
    pub fn lock_channel(&mut self, interface: usize, channel: u8) -> Result<bool, LMacError> {
        match self.locks {
            Some((ref mut interfaces, ref mut locked_channel)) => {
                // First we check, whether the channel is already locked by this interface.
                let already_locked_by_interface = interfaces[interface];
                // Then we check if the requested channel is the one that's already locked.
                let same_channel = *locked_channel == channel;
                // Then we check how many interfaces have locked the channel as well.
                let interface_lock_count = interfaces.iter().filter(|locked| **locked).count();
                let mut changed = false;
                if !same_channel && (!already_locked_by_interface || interface_lock_count > 1) {
                    // Switching the channel is not possible if another interface already holds
                    // lock.
                    return Err(LMacError::ChannelLockedByOtherInterface);
                } else if already_locked_by_interface && interface_lock_count == 1 {
                    // The interface already has exclusive lock on the channel and can therefore
                    // freely change it.
                    *locked_channel = channel;
                    changed = true;
                } else if !already_locked_by_interface && same_channel {
                    // The interface does not have lock on the interface yet, but attempts to lock
                    // the same channel.
                    interfaces[interface] = true;
                }
                Ok(changed)
            }
            None => {
                // No other interface has lock currently.
                self.locks = Some((array::from_fn(|i| i == interface), channel));
                Ok(true)
            }
        }
    }
    /// Unlock the channel for the specified interface.
    pub fn unlock_channel(&mut self, interface: usize) {
        if let Some((ref mut interfaces, _)) = self.locks {
            if interfaces[interface] {
                interfaces[interface] = false;
                if *interfaces == [false; WiFi::INTERFACE_COUNT] {
                    self.locks = None;
                }
            }
        }
    }
    /// Check whether an off channel operation is in progress.
    pub fn is_off_channel_operation_in_progress(&self) -> bool {
        self.off_channel_operation_interface.is_some()
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// An error returned by the LMAC.
pub enum LMacError {
    /// The channel is invalid.
    InvalidChannel,
    /// The channel is disabled.
    ChannelDisabled,
    /// The channel is locked by the other interface.
    ChannelLockedByOtherInterface,
    /// The off channel request was rejected.
    OffChannelRequestRejected,
    /// Another off channel operation is already in progress.
    OffChannelOperationAlreadyInProgress,
}

pub(crate) struct SharedLMacState {
    wifi: WiFi<'static>,
    // Channel Management
    /// The current channel state.
    ///
    /// Throughout the codebase, it's assumed, that inside one lock, you don't reacquire it again.
    /// This is used to prevent unnecessary panic code being generated.
    channel_state: blocking_mutex::NoopMutex<RefCell<ChannelState>>,
    off_channel_requesters: [OffChannelRequester; WiFi::INTERFACE_COUNT],
    off_channel_completion_signal: Watch<NoopRawMutex, (), { WiFi::INTERFACE_COUNT }>,

    key_slot_manager: KeySlotManager
}
impl SharedLMacState {
    pub const fn new(wifi: WiFi<'_>) -> Self {
        Self {
            wifi: unsafe { core::mem::transmute::<WiFi<'_>, WiFi<'static>>(wifi) },
            channel_state: blocking_mutex::NoopMutex::new(RefCell::new(ChannelState {
                locks: None,
                off_channel_operation_interface: None,
            })),
            off_channel_requesters: [const { OffChannelRequester::new() }; WiFi::INTERFACE_COUNT],
            off_channel_completion_signal: Watch::new(),
            key_slot_manager: KeySlotManager::new()
        }
    }
    pub fn split<'res>(
        &'res mut self,
        dyn_tx_buffer_manager: DynTxBufferManager<'res>,
    ) -> (
        LMacReceiveEndpoint<'res>,
        [LMacInterfaceControl<'res>; WiFi::INTERFACE_COUNT],
    ) {
        (
            LMacReceiveEndpoint { shared_state: self },
            core::array::from_fn(|i| {
                LMacInterfaceControl {
                shared_state: self,
                interface: i,
                dyn_tx_buffer_manager,
                off_channel_completion_listener: Mutex::new(
                    self.off_channel_completion_signal.dyn_receiver().expect("This shouldn't be able to fail, since we have as many receivers as interface controls."),
                ),
            }
            }),
        )
    }
    fn get_channel_state(&self) -> ChannelState {
        self.channel_state.lock(|rc| {
            rc.try_borrow()
                .map(|channel_state| *channel_state)
                .unwrap_or_default()
        })
    }
}

pub(crate) struct LMacReceiveEndpoint<'res> {
    shared_state: &'res SharedLMacState,
}
impl<'res> LMacReceiveEndpoint<'res> {
    /// Wait for a frame to arrive from the driver.
    pub async fn receive(&mut self) -> BorrowedBuffer<'res> {
        self.shared_state.wifi.receive().await
    }
}
/// An interface bringup operation.
///
/// This will unlock the interface if dropped and can therefore be used to make a connection
/// bringup cancel safe. Calling [InterfaceBringupOperation::complete] indicates, that the
/// interface has been successfully brought up.
///
/// # Example
/// Consider an STA implementation. When connecting to a network, the implementation will attempt
/// to acquire channel lock before the connection setup. If that setup fails or the future is
/// cancelled, the channel should be unlocked again.
pub struct InterfaceBringupOperation<'res, 'a> {
    lmac_interface_control: &'a LMacInterfaceControl<'res>,
}
impl InterfaceBringupOperation<'_, '_> {
    /// Mark the interface bringup as completed, without unlocking the channel.
    pub fn complete(self) {
        debug!(
            "Interface bringup operation completed for interface {}.",
            self.lmac_interface_control.interface
        );
        core::mem::forget(self);
    }
}
impl Drop for InterfaceBringupOperation<'_, '_> {
    fn drop(&mut self) {
        debug!(
            "Releasing channel lock from interface {}, since bringup operation failed.",
            self.lmac_interface_control.get_filter_interface()
        );
        self.lmac_interface_control.unlock_channel_internal();
    }
}
/// Control over the interface.
pub struct LMacInterfaceControl<'res> {
    shared_state: &'res SharedLMacState,
    /// The RX filter interface, this logical interface corresponds to.
    interface: usize,
    /// Access to the TX buffer manager.
    dyn_tx_buffer_manager: DynTxBufferManager<'res>,
    off_channel_completion_listener: Mutex<NoopRawMutex, DynReceiver<'res, ()>>,
}
impl<'res> LMacInterfaceControl<'res> {
    /// Some default transmission parameters.
    pub const DEFAULT_TX_PARAMETERS: TxParameters = TxParameters {
        rate: WiFiRate::PhyRate1ML,
        override_seq_num: true,
        tx_error_behaviour: TxErrorBehaviour::RetryUntil(7),
        ack_timeout: 10,
        key_slot: None,
    };
    /// Transmit a frame.
    ///
    /// Returns the number of retries.
    /// For an exact description see [WiFi::transmit], as this is just a passthrough.
    /// NOTE: This will not check, whether an off channel operation is currently in progress.
    pub async fn transmit(
        &self,
        buffer: &mut [u8],
        tx_parameters: &TxParameters,
        wait_for_ack: bool,
    ) -> WiFiResult<usize> {
        self.shared_state
            .wifi
            .transmit(
                buffer,
                tx_parameters,
                wait_for_ack.then_some(self.interface),
            )
            .await
    }
    /// Transmit a frame and execute the provided closure right before tranmission.
    ///
    /// Returns the number of retries.
    /// For an exact description see [WiFi::transmit_with_hook], as this is just a passthrough.
    /// NOTE: This will not check, whether an off channel operation is currently in progress.
    pub async fn transmit_with_hook(
        &self,
        buffer: &mut [u8],
        tx_parameters: &TxParameters,
        wait_for_ack: bool,
        pre_transmit_hook: impl FnMut(&mut [u8]),
    ) -> WiFiResult<usize> {
        self.shared_state
            .wifi
            .transmit_with_hook(
                buffer,
                tx_parameters,
                wait_for_ack.then_some(self.interface),
                pre_transmit_hook,
            )
            .await
    }
    /// Allocate a [TxBuffer] from the buffer manager.
    pub async fn alloc_tx_buf(&self) -> TxBuffer {
        self.dyn_tx_buffer_manager.alloc().await
    }

    fn lock_channel_internal(&self, channel: u8) -> Result<(), LMacError> {
        // This will attempt to first lock the channel and then go to that channel, if it changed.
        if self.shared_state.channel_state.lock(|cs| {
            // It's again assumed here, that no other lock on the channel state is currently being
            // held, so this will never fail.
            cs.try_borrow_mut().map_or(Ok(false), |mut channel_state| {
                channel_state.lock_channel(self.interface, channel)
            })
        })? {
            if self.shared_state.wifi.set_channel(channel).is_ok() {
                debug!(
                    "Switched to channel {} while acquiring channel lock for interface {}.",
                    channel, self.interface
                );
            } else {
                debug!("Attempted to acquire channel lock for interface {} on channel {}, which is invalid.", self.interface, channel);
                self.unlock_channel_internal();
                return Err(LMacError::InvalidChannel);
            }
        }
        Ok(())
    }
    fn unlock_channel_internal(&self) {
        self.shared_state.channel_state.lock(|rc| {
            let _ = rc
                .try_borrow_mut()
                .map(|mut channel_state| channel_state.unlock_channel(self.interface));
        });
    }
    /// Begin an interface bringup operation.
    ///
    /// This allows implementing cancel safe connection bringup and is otherwise the same as
    /// [LMacInterfaceControl::lock_channel].
    /// NOTE: This does not ensure, that no off channel operation is currently in progress, you'll
    /// have to use [LMacInterfaceControl::wait_for_off_channel_completion] to wait for that.
    ///
    /// ```ignore
    /// let bringup_operation = interface_control.begin_interface_bringup_operation(6);
    ///
    /// /* Do connection setup */
    ///
    /// // If the future was dropped before this, the lock would be released.
    /// bringup_operation.complete();
    /// ```
    pub fn begin_interface_bringup_operation<'a>(
        &'a self,
        channel: u8,
    ) -> Result<InterfaceBringupOperation<'res, 'a>, LMacError> {
        match self.lock_channel_internal(channel) {
            Ok(_) => {
                debug!(
                    "Successfully started interface bringup operation for interface {} on channel: {}.",
                    self.interface,
                    channel
                );
            }
            Err(err) => {
                debug!(
                    "Failed to start interface bringup operation for interface {} on channel {}.",
                    self.interface, channel
                );
                return Err(err);
            }
        }
        Ok(InterfaceBringupOperation {
            lmac_interface_control: self,
        })
    }
    /// Try to acquire channel lock for this interface and go to the specified channel.
    pub fn lock_channel(&self, channel: u8) -> Result<(), LMacError> {
        let res = self.lock_channel_internal(channel);

        #[allow(clippy::if_same_then_else)]
        if res.is_ok() {
            debug!(
                "Successfully acquired lock on channel {} for interface {}.",
                channel, self.interface
            );
        } else {
            debug!(
                "Failed to acquire lock on channel {} for interface {}.",
                channel, self.interface
            );
        }

        res
    }
    /// Release channel lock from this interface.
    pub fn unlock_channel(&self) {
        debug!(
            "Releasing channel lock for interface {}.",
            self.interface
        );
        self.unlock_channel_internal();
    }
    /// Wait for any pending off channel operations to complete.
    pub async fn wait_for_off_channel_completion(&self) {
        if !self
            .shared_state
            .get_channel_state()
            .is_off_channel_operation_in_progress()
        {
            return;
        }
        self.off_channel_completion_listener
            .lock()
            .await
            .get()
            .await
    }
    /// Try to start an interface off channel operation.
    ///
    /// This is the same as [Self::begin_interface_off_channel_operation], but won't wait for other
    /// operations to complete first and instead return [LMacError::OffChannelOperationAlreadyInProgress].
    pub async fn try_begin_interface_off_channel_operation<'a>(
        &'a self,
    ) -> Result<OffChannelOperation<'a, 'res>, LMacError> {
        if self.off_channel_operation_in_progress() {
            return Err(LMacError::OffChannelOperationAlreadyInProgress);
        }
        // At this point we know that no off channel operation is active.
        let channel_state = self.shared_state.get_channel_state().locks;
        if let Some((locks, _)) = channel_state {
            debug!("Requesting off channel grants from interfaces that have lock.");
            let mut i = 0;
            // We request off channel operations from all interfaces, that currently have lock.
            let results = join_array(locks.map(|locked| {
                let interface = i;
                i += 1;
                async move {
                    if locked {
                        self.shared_state.off_channel_requesters[interface]
                            .request()
                            .await
                            .inspect_err(|_| {
                                debug!("Off channel request was rejected by interface {}.", i)
                            })
                    } else {
                        Ok(())
                    }
                }
            }))
            .await;
            // If any request fails, we bail out here.
            for result in results {
                result?;
            }
        }
        self.shared_state.channel_state.lock(|cs| {
            if let Ok(mut channel_state) = cs.try_borrow_mut() {
                channel_state.off_channel_operation_interface = Some(self.interface);
            }
        });
        debug!(
            "Successfully started off channel operation for interface {}.",
            self.interface
        );
        Ok(OffChannelOperation {
            interface_control: self,
            rx_filter_interface: self.interface,
            previously_locked_channel: self
                .shared_state
                .get_channel_state()
                .locks
                .map(|(_, channel)| channel),
        })
    }
    /// Begin an interface off channel operation and wait for any previous off channel operations
    /// to complete.
    ///
    /// This will request grants from all interfaces, that currently have lock on the channel.
    /// When or if an interface responds to such a request is up to the specific implementation. It
    /// is therefore advised to wrap this in a timeout.
    pub async fn begin_interface_off_channel_operation<'a>(
        &'a self,
    ) -> Result<OffChannelOperation<'a, 'res>, LMacError> {
        // Before we start any operation, we need to wait for other pending ones to finish.
        self.wait_for_off_channel_completion().await;
        self.try_begin_interface_off_channel_operation().await
    }
    /// Wait for an off channel operations request to arrive, which can then be granted or rejected.
    pub async fn wait_for_off_channel_request(&self) -> OffChannelRequest<'_> {
        self.shared_state.off_channel_requesters[self.interface]
            .wait_for_request()
            .await
    }
    /// Check if an off channel operation is in progress.
    pub fn off_channel_operation_in_progress(&self) -> bool {
        self.shared_state
            .get_channel_state()
            .is_off_channel_operation_in_progress()
    }
    /// Returns the ID of the interface currently performing an off channel operation.
    pub fn off_channel_operation_interface(&self) -> Option<usize> {
        self.shared_state
            .get_channel_state()
            .off_channel_operation_interface
    }
    /// Returns the currently locked channel.
    pub fn home_channel(&self) -> Option<u8> {
        self.shared_state
            .get_channel_state()
            .locks
            .map(|(_locks, channel)| channel)
    }

    /// Returns the id of the filter interface.
    pub fn get_filter_interface(&self) -> usize {
        self.interface
    }
    /// Get the factory MAC address for this interface.
    ///
    /// NOTE: This reads the base MAC address from the efuses and adds the interface index to the
    /// last octet.
    pub fn get_factory_mac_for_interface(&self) -> [u8; 6] {
        let mut base_mac = Efuse::read_base_mac_address();
        base_mac[5] += self.interface as u8;
        base_mac
    }
    /// Get the current channel.
    pub fn get_current_channel(&self) -> u8 {
        self.shared_state.wifi.get_channel()
    }
    /// Set the filter parameters.
    ///
    /// NOTE: You need to set **and** enable the filter.
    pub fn set_filter_parameters(
        &self,
        bank: RxFilterBank,
        mac_address: [u8; 6],
        mask: Option<[u8; 6]>,
    ) {
        let mask = mask.unwrap_or([0xff; 6]);
        let _ =
            self.shared_state
                .wifi
                .set_filter(bank, self.interface, mac_address, mask);
        trace!(
            "Set {:?} filter for interface {} to address {:?} and mask {:?}.",
            bank,
            self.interface,
            mac_address,
            mask
        );
    }
    /// Set the filter status.
    ///
    /// NOTE: You need to set **and** enable the filter.
    pub fn set_filter_status(&self, bank: RxFilterBank, enabled: bool) {
        let _ = self
            .shared_state
            .wifi
            .set_filter_status(bank, self.interface, enabled);
        trace!(
            "{} {:?} filter for interface {}.",
            bank,
            if enabled { "Enabled" } else { "Disabled" },
            self.interface
        );
    }
    /// Set the scanning mode of the RX filter interface.
    ///
    /// Enabling this, tells the hardware to forward any broadcast MPDUs to this interface.
    pub fn set_scanning_mode(&self, scanning_mode: ScanningMode) {
        let _ = self
            .shared_state
            .wifi
            .set_scanning_mode(self.interface, scanning_mode);
        trace!(
            "Set scanning mode for interface {} to {:?}.",
            self.interface,
            scanning_mode
        );
    }
    /// This decides, whether the hardware will assume, that the BSSID is the same as the RA, if
    /// the BSSID filter is disabled.
    ///
    /// For further documentation see [WiFi::set_filter_bssid_check].
    pub fn set_filter_bssid_check(&self, enabled: bool) {
        let _ = self
            .shared_state
            .wifi
            .set_filter_bssid_check(self.interface, enabled);
        trace!(
            "{} BSSID check for interface {}.",
            if enabled { "Enabled" } else { "Disabled" },
            self.interface
        );
    }
    /// Get the current MAC time from the Wi-Fi driver.
    ///
    /// For further documentation see [WiFi::mac_time].
    pub fn mac_time(&self) -> u32 {
        self.shared_state.wifi.mac_time()
    }
    /// Attempt to acquire a key slot.
    pub fn acquire_key_slot(&self) -> Option<KeySlot<'res>> {
        self.shared_state.key_slot_manager.acquire_key_slot().inspect(|key_slot| {
            debug!("Key Slot {} was acquired by interface {}.", key_slot, self.interface);
        }).map(|key_slot| KeySlot {
            shared_state: self.shared_state,
            key_slot,
            interface: self.interface as u8,
        })
    }
}
