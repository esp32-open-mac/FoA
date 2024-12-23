//! This module implements the lower MAC (LMAC).
//!
//! The LMAC's job is it to divide access to the medium between all interfaces.
//! This achieved, by allowing an interface to lock the channel, on which the stack operates. This
//! prevents the other interface from changing the channel. If both interfaces attempt to lock the
//! same channel, they'll both have the lock on the channel and neither can change it, without the
//! other unlocking the channel beforehand. This is done through the [LMacInterfaceControl::set_and_lock_channel] and [LMacInterfaceControl::unlock_channel] APIs.
//!
//! The issue with this design is, that it makes scanning impossible, if the channel is already
//! locked by another interfaces. To fix this, off channel operations are implemented, which allow
//! an interface to request permission from the other interface to temporarily change the channel
//! and scanning mode. This request maybe granted or rejected, by the other interface. To request
//! an off channel operation, the [LMacInterfaceControl::begin_interface_off_channel_operation] API
//! can be used. If the other interface currently holds the channel lock, it will receive an [OffChannelRequest] from [LMacInterfaceControl::wait_for_off_channel_request].
//! Which can then be granted or rejected.
//!
//! As you may have noticed, there is now public type providing receive access to the medium. This
//! is due to shared RX access being very hard to implement. Instead, frames addressed to an
//! interface will be passed to [interface_input](crate::interface::InterfaceInput::interface_input), by the MAC task.
use core::{
    cell::{Cell, RefCell},
    future::poll_fn,
    sync::atomic::{AtomicU16, Ordering},
    task::Poll,
};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    mutex::Mutex,
    waitqueue::AtomicWaker,
    watch::{DynReceiver, Watch},
};
use esp32_wifi_hal_rs::{
    BorrowedBuffer, RxFilterBank, RxFilterInterface, TxErrorBehaviour, TxParameters, WiFi,
    WiFiRate, WiFiResult,
};

use crate::tx_buffer_management::{DynTxBufferManager, TxBuffer};

fn get_opposite_interface(rx_filter_interface: RxFilterInterface) -> RxFilterInterface {
    if rx_filter_interface == RxFilterInterface::Zero {
        RxFilterInterface::One
    } else {
        RxFilterInterface::Zero
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The status of the channel lock.
enum ChannelState {
    /// The channel is not locked.
    NotLocked,
    /*
    /// The channel isn't fully locked yet, since the specified interface is currently performing
    /// actions to bring the interface into a state, where the channel can be locked.
    InBringUpByInterface {
        rx_filter_interface: RxFilterInterface,
        channel: u8,
    },
    */
    /// The channel is locked by the specified interface.
    LockedByInterface(RxFilterInterface),
    /// Both interfaces are locked to the same channel.
    LockedByBothInterfaces,
    /// An off channel operation is in progress.
    OffChannelOperationInProgress,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum OffChannelRequestState {
    #[default]
    NotRequested,
    Requested,
    Granted,
    Rejected,
}

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
}

/// An off channel request.
///
/// This can be used, to grant an off channel operation, after it has been requested.
/// If this is dropped, the off channel request will be automatically rejected.
pub struct OffChannelRequest<'a> {
    requester: &'a OffChannelRequester,
}
impl OffChannelRequest<'_> {
    /// Grant the request.
    pub fn grant(self) {
        self.requester.status.set(OffChannelRequestState::Granted);
        self.requester.response_waker.wake();
        core::mem::forget(self);
    }
    /// Reject the off channel request.
    pub fn reject(self) {
        // Dropping will automatically reject the request.
    }
}
impl Drop for OffChannelRequest<'_> {
    fn drop(&mut self) {
        self.requester.status.set(OffChannelRequestState::Rejected);
        self.requester.response_waker.wake();
    }
}

struct OffChannelRequester {
    status: Cell<OffChannelRequestState>,
    request_waker: AtomicWaker,
    response_waker: AtomicWaker,
}
impl OffChannelRequester {
    pub const fn new() -> Self {
        Self {
            status: Cell::new(OffChannelRequestState::NotRequested),
            request_waker: AtomicWaker::new(),
            response_waker: AtomicWaker::new(),
        }
    }
    pub async fn request(&self) -> Result<(), LMacError> {
        self.status.set(OffChannelRequestState::Requested);
        self.request_waker.wake();
        poll_fn(|cx| {
            let status = self.status.get();
            match status {
                OffChannelRequestState::Granted => Poll::Ready(Ok(())),
                OffChannelRequestState::Rejected => {
                    Poll::Ready(Err(LMacError::OffChannelRequestRejected))
                }
                _ => {
                    self.response_waker.register(cx.waker());
                    Poll::Pending
                }
            }
        })
        .await?;
        self.status.set(OffChannelRequestState::NotRequested);
        Ok(())
    }
    pub async fn wait_for_request(&self) -> OffChannelRequest<'_> {
        poll_fn(|cx| {
            let status = self.status.get();
            if status == OffChannelRequestState::Requested {
                Poll::Ready(OffChannelRequest { requester: self })
            } else {
                self.request_waker.register(cx.waker());
                Poll::Pending
            }
        })
        .await
    }
}

pub(crate) struct SharedLMacState<'res> {
    wifi: WiFi<'res>,
    // Channel Management
    channel_state: critical_section::Mutex<RefCell<ChannelState>>,
    if_zero_off_channel_requester: OffChannelRequester,
    if_one_off_channel_requester: OffChannelRequester,
    off_channel_completion_signal: Watch<NoopRawMutex, (), 2>,
}
impl<'res> SharedLMacState<'res> {
    pub const fn new(wifi: WiFi<'res>) -> Self {
        Self {
            wifi,
            channel_state: critical_section::Mutex::new(RefCell::new(ChannelState::NotLocked)),
            if_zero_off_channel_requester: OffChannelRequester::new(),
            if_one_off_channel_requester: OffChannelRequester::new(),
            off_channel_completion_signal: Watch::new(),
        }
    }
    pub fn split(
        &'res mut self,
        dyn_tx_buffer_manager: DynTxBufferManager<'res>,
    ) -> (
        LMacReceiveEndpoint<'res>,
        LMacTransmitEndpoint<'res>,
        LMacInterfaceControl<'res>,
        LMacInterfaceControl<'res>,
    ) {
        (
            LMacReceiveEndpoint { shared_state: self },
            LMacTransmitEndpoint {
                shared_state: self,
                dyn_tx_buffer_manager,
            },
            LMacInterfaceControl {
                shared_state: self,
                rx_filter_interface: RxFilterInterface::Zero,
                off_channel_completion_receiver: Mutex::new(
                    self.off_channel_completion_signal.dyn_receiver().unwrap(),
                ),
                sequence_number: AtomicU16::new(0),
            },
            LMacInterfaceControl {
                shared_state: self,
                rx_filter_interface: RxFilterInterface::One,
                off_channel_completion_receiver: Mutex::new(
                    self.off_channel_completion_signal.dyn_receiver().unwrap(),
                ),
                sequence_number: AtomicU16::new(0),
            },
        )
    }
    fn get_channel_state(&self) -> ChannelState {
        critical_section::with(|cs| *self.channel_state.borrow_ref(cs))
    }
    fn set_channel_state(&self, channel_state: ChannelState) {
        critical_section::with(|cs| *self.channel_state.borrow_ref_mut(cs) = channel_state);
    }
    /// Returns the [OffChannelRequester] for the interface.
    fn get_off_channel_requester_for_interface(
        &self,
        rx_filter_interface: RxFilterInterface,
    ) -> &OffChannelRequester {
        match rx_filter_interface {
            RxFilterInterface::Zero => &self.if_zero_off_channel_requester,
            RxFilterInterface::One => &self.if_one_off_channel_requester,
            _ => unreachable!(),
        }
    }
}

/// An active off channel operation.
///
/// When dropped, this will terminate the off channel operation and reset the channel and scanning
/// mode.
pub struct OffChannelOperation<'a, 'res> {
    transmit_endpoint: &'a LMacTransmitEndpoint<'res>,
    previous_channel_state: ChannelState,
    previous_channel: u8,
    rx_filter_interface: RxFilterInterface,
}
impl OffChannelOperation<'_, '_> {
    /// Set the channel.
    pub fn set_channel(&mut self, channel: u8) -> Result<(), LMacError> {
        self.transmit_endpoint
            .shared_state
            .wifi
            .set_channel(channel)
            .map_err(|_| LMacError::InvalidChannel)
    }
    pub fn set_scanning_mode(&mut self, enabled: bool) {
        self.transmit_endpoint
            .shared_state
            .wifi
            .set_scanning_mode(self.rx_filter_interface, enabled);
    }
}
impl Drop for OffChannelOperation<'_, '_> {
    fn drop(&mut self) {
        let _ = self.set_channel(self.previous_channel);
        self.set_scanning_mode(false);
        critical_section::with(|cs| {
            *self
                .transmit_endpoint
                .shared_state
                .channel_state
                .borrow_ref_mut(cs) = self.previous_channel_state;
        });
        self.transmit_endpoint
            .shared_state
            .off_channel_completion_signal
            .sender()
            .send(());
    }
}

pub(crate) struct LMacReceiveEndpoint<'res> {
    shared_state: &'res SharedLMacState<'res>,
}
impl<'res> LMacReceiveEndpoint<'res> {
    /// Wait for a frame to arrive from the driver.
    pub async fn receive(&mut self) -> BorrowedBuffer<'res, 'res> {
        self.shared_state.wifi.receive().await
    }
}
#[derive(Clone, Copy)]
/// Transmit access to the LMAC.
pub struct LMacTransmitEndpoint<'res> {
    shared_state: &'res SharedLMacState<'res>,
    dyn_tx_buffer_manager: DynTxBufferManager<'res>,
}
impl<'res> LMacTransmitEndpoint<'res> {
    /// Transmit a frame.
    ///
    /// NOTE: This will not check, whether an off channel operation is currently in progress.
    pub async fn transmit(
        &self,
        buffer: &mut [u8],
        tx_parameters: &TxParameters,
    ) -> WiFiResult<()> {
        self.shared_state.wifi.transmit(buffer, tx_parameters).await
    }
    /// Allocate a [TxBuffer] from the buffer manager.
    pub async fn alloc_tx_buf(&self) -> TxBuffer {
        self.dyn_tx_buffer_manager.alloc().await
    }

    // TODO: Implement after congress.
    // To anyone who reads this: I was on a pretty tight schedule.
    /*
    pub async fn begin_off_channel_operation<'a>(&'a self) -> OffChannelOperation<'a, 'res> {
        // We shouldn't hold the lock until permission is granted, so we just copy out the value
        // here.
        let channel_state = self.shared_state.get_channel_state();
        match channel_state {
            ChannelState::NotLocked => {}
            ChannelState::LockedByInterface(interface) => {
                self.shared_state
                    .get_off_channel_requester_for_interface(interface)
                    .request()
                    .await
            }
            ChannelState::LockedByBothInterfaces => {
                join(
                    self.shared_state.if_zero_off_channel_requester.request(),
                    self.shared_state.if_one_off_channel_requester.request(),
                )
                .await;
            }
            ChannelState::OffChannelOperationInProgress => {
                // The idea is, that if an off channel operation is completed, we signal this
                // through the off_channel_completion_signal. If this signal is already signaled,
                // but the channel_state is still set to OffChannelOperationInProgress, we reset
                // the signal. This way, we ensure that everyone who needs to know the end of an
                // off channel operation gets informed, before we reset the signal.
                self.shared_state
                    .off_channel_completion_signal
                    .sender()
                    .send(());
            }
        }
        self.shared_state
            .set_channel_state(ChannelState::OffChannelOperationInProgress);
        OffChannelOperation {
            transmit_endpoint: self,
            previous_channel_state: channel_state,
            previous_channel: self.shared_state.wifi.get_channel(),
        }

    }
    */
}
/// Control over the interface.
pub struct LMacInterfaceControl<'res> {
    shared_state: &'res SharedLMacState<'res>,
    /// The RX filter interface, this logical interface corresponds to.
    rx_filter_interface: RxFilterInterface,
    /// Receiver for off channel completion notifications.
    off_channel_completion_receiver: Mutex<NoopRawMutex, DynReceiver<'res, ()>>,
    /// The current sequence number for this interface.
    sequence_number: AtomicU16,
}
impl LMacInterfaceControl<'_> {
    /// Get and increase the current sequence_number.
    pub fn get_and_increase_sequence_number(&self) -> u16 {
        let sequence_number = self.sequence_number.load(Ordering::Relaxed);
        self.sequence_number
            .store(sequence_number.wrapping_add(1), Ordering::Relaxed);
        sequence_number
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
        self.shared_state.wifi.set_filter(
            bank,
            self.rx_filter_interface,
            mac_address,
            mask.unwrap_or([0xff; 6]),
        );
    }
    /// Set the filter status.
    ///
    /// NOTE: You need to set **and** enable the filter.
    pub fn set_filter_status(&self, bank: RxFilterBank, enabled: bool) {
        self.shared_state
            .wifi
            .set_filter_status(bank, self.rx_filter_interface, enabled);
    }
    /// Set the scanning mode of the RX filter interface.
    ///
    /// Enabling this, tells the hardware to forward any broadcast MPDUs to this interface.
    pub fn set_scanning_mode(&self, enabled: bool) {
        self.shared_state
            .wifi
            .set_scanning_mode(self.rx_filter_interface, enabled);
    }
    // TODO: I will implement this after 38c3, to make the connection bring up cancel safe.
    /*
    pub async fn begin_interface_bringup_operation(&self, channel: u8) {
        let previous_channel_state = self.shared_state.get_channel_state();
        let new_channel_state = ChannelState::InBringUpByInterface {
            rx_filter_interface: self.rx_filter_interface,
            channel,
        };
        if previous_channel_state != new_channel_state {
            match previous_channel_state {
                ChannelState::NotLocked => {
                    self.shared_state.set_channel_state(new_channel_state);
                }
                ChannelState::InBringUpByInterface { .. } => {}
                ChannelState::LockedByInterface(rx_filter_interface) => {}
            }
        }
    } */

    /// Try to set the channel and if possible lock on to it.
    ///
    /// If the channel isn't locked, this will lock it and set channel.
    /// This will fail, if the other interface has already locked the channel or both, and the requested
    /// channel and the locked channel aren't the same. If they're, this function doesn't do
    /// anything.
    /// If the channel is only locked by this interface, it is changed.
    pub async fn set_and_lock_channel(&self, channel: u8) -> Result<(), LMacError> {
        // We'll wait for any off channel operation to complete here. If none are in progress, this
        // returns immediately.
        //  We save the previous state here.
        let previous_channel_state = self.shared_state.get_channel_state();
        // This is the state we'll be in after locking.
        let new_channel_state = ChannelState::LockedByInterface(self.rx_filter_interface);

        // If we are already in the state, we would be in after this operation, we don't need to
        // change anything, so we just set the channel to the requested number.
        if previous_channel_state != new_channel_state {
            match previous_channel_state {
                // This is the simplest case, where we can just set it to the new state.
                ChannelState::NotLocked => {
                    self.shared_state.set_channel_state(new_channel_state);
                }
                // Due to the if statement above, this checks whether the other interface has
                // already locked the channel state.
                ChannelState::LockedByInterface(_) => {
                    // If we're lucky and the locked channel and the requested channel match, we
                    // can just set the channel state to locked by both. Otherwise, we return an
                    // error.
                    if self.shared_state.wifi.get_channel() == channel {
                        self.shared_state
                            .set_channel_state(ChannelState::LockedByBothInterfaces);
                    } else {
                        return Err(LMacError::ChannelLockedByOtherInterface);
                    }
                }
                // This means, that we already locked the channel and both interfaces are locked on
                // to the same channel. Due to this, we can't change the channel until the other
                // interface unlocks it.
                ChannelState::LockedByBothInterfaces => {
                    return Err(LMacError::ChannelLockedByOtherInterface);
                }
                // Due to the call to wait_for_off_channel_completion, this is unreachable.
                _ => unreachable!(),
            }
        }
        // Once we got to this point, we should have handled all invariants and can safely go to a
        // new channel.
        self.shared_state
            .wifi
            .set_channel(channel)
            .map_err(|_| LMacError::InvalidChannel)?;
        Ok(())
    }
    /// Unlock the channel state from this interface.
    pub async fn unlock_channel(&self) {
        // As with any other state changes, we first wait for off channel operations to complete.
        self.wait_for_off_channel_completion().await;
        let channel_state = self.shared_state.get_channel_state();
        if channel_state == ChannelState::LockedByBothInterfaces {
            // Set it to the inverse of our locked channel state.
            self.shared_state
                .set_channel_state(ChannelState::LockedByInterface(get_opposite_interface(
                    self.rx_filter_interface,
                )));
        } else if channel_state == ChannelState::LockedByInterface(self.rx_filter_interface) {
            self.shared_state.set_channel_state(ChannelState::NotLocked);
        }
    }
    /// Begin an interface off channel operation.
    ///
    /// This will only request a grant for the operation from the other interface, not the
    /// requesting one.
    pub async fn begin_interface_off_channel_operation<'a, 'res>(
        &'a self,
        transmit_endpoint: &'a LMacTransmitEndpoint<'res>,
    ) -> Result<OffChannelOperation<'a, 'res>, LMacError> {
        // Before we start any operation, we need to wait for other pending ones to finish.
        self.wait_for_off_channel_completion().await;
        let previous_channel_state = self.shared_state.get_channel_state();

        if previous_channel_state != ChannelState::LockedByInterface(self.rx_filter_interface) {
            match previous_channel_state {
                ChannelState::NotLocked => {}
                // Wait for the other interface to grant our request. We asserted in the if
                // statement, that we don't have exclusive lock on the interface.
                ChannelState::LockedByInterface(interface) => {
                    self.shared_state
                        .get_off_channel_requester_for_interface(interface)
                        .request()
                        .await?
                }
                // We assume our interface automatically grants it's own request, so we again only
                // ask the other interfaces.
                ChannelState::LockedByBothInterfaces => {
                    self.shared_state
                        .get_off_channel_requester_for_interface(get_opposite_interface(
                            self.rx_filter_interface,
                        ))
                        .request()
                        .await?
                }
                ChannelState::OffChannelOperationInProgress => unreachable!(),
            }
        }

        self.shared_state
            .set_channel_state(ChannelState::OffChannelOperationInProgress);

        Ok(OffChannelOperation {
            rx_filter_interface: self.rx_filter_interface,
            transmit_endpoint,
            previous_channel_state,
            previous_channel: self.shared_state.wifi.get_channel(),
        })
    }
    /// Wait for any pending off channel operations to complete.
    pub async fn wait_for_off_channel_completion(&self) {
        if self.shared_state.get_channel_state() != ChannelState::OffChannelOperationInProgress {
            return;
        }
        self.off_channel_completion_receiver
            .lock()
            .await
            .changed()
            .await;
    }
    /// Wait for an off channel operations request to arrive, which can then be granted.
    pub async fn wait_for_off_channel_request(&self) -> OffChannelRequest<'_> {
        self.shared_state
            .get_off_channel_requester_for_interface(self.rx_filter_interface)
            .wait_for_request()
            .await
    }
    pub fn get_default_tx_parameters(&self) -> TxParameters {
        TxParameters {
            rate: WiFiRate::PhyRate1ML,
            duration: 0,
            interface_zero: self.rx_filter_interface == RxFilterInterface::Zero,
            interface_one: self.rx_filter_interface == RxFilterInterface::One,
            wait_for_ack: true,
            tx_error_behaviour: TxErrorBehaviour::RetryUntil(4),
        }
    }
}
