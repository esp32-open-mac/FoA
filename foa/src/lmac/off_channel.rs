use core::{cell::Cell, future::poll_fn, task::Poll};

use embassy_sync::waitqueue::AtomicWaker;
use esp_wifi_hal::prelude::*;

use super::{LMacError, LMacInterfaceControl};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum OffChannelRequestState {
    #[default]
    NotRequested,
    Requested,
    Granted,
    Rejected,
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

/// This implements the mechanism to asynchronously request grants for off channel operations from
/// interfaces.
pub(crate) struct OffChannelRequester {
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
    pub fn request(&self) -> impl Future<Output = Result<(), LMacError>> {
        self.status.set(OffChannelRequestState::Requested);
        self.request_waker.wake();
        poll_fn(|cx| {
            let status = self.status.get();
            match status {
                OffChannelRequestState::Granted => {
                    self.status.set(OffChannelRequestState::NotRequested);
                    Poll::Ready(Ok(()))
                }
                OffChannelRequestState::Rejected => {
                    Poll::Ready(Err(LMacError::OffChannelRequestRejected))
                }
                _ => {
                    self.response_waker.register(cx.waker());
                    Poll::Pending
                }
            }
        })
    }
    pub fn wait_for_request(&self) -> impl Future<Output = OffChannelRequest<'_>> {
        poll_fn(|cx| {
            let status = self.status.get();
            if status == OffChannelRequestState::Requested {
                Poll::Ready(OffChannelRequest { requester: self })
            } else {
                self.request_waker.register(cx.waker());
                Poll::Pending
            }
        })
    }
}

/// An active off channel operation.
///
/// When dropped, this will terminate the off channel operation and reset the channel and scanning
/// mode.
pub struct OffChannelOperation<'a, 'res> {
    pub(crate) interface_control: &'a LMacInterfaceControl<'res>,
    pub(crate) previously_locked_channel: Option<u8>,
    pub(crate) rx_filter_interface: usize,
}
impl OffChannelOperation<'_, '_> {
    /// Set the channel.
    pub fn set_channel(&mut self, channel: u8) -> Result<(), LMacError> {
        self.interface_control
            .shared_state
            .channel_state
            .lock(|ref_cell| {
                ref_cell
                    .borrow_mut()
                    .channel_controller
                    .set_channel(channel)
                    .map_err(|_| LMacError::InvalidChannel)
            })
    }
    /// Set the scanning mode.
    pub fn set_scanning_mode(&mut self, scanning_mode: ScanningMode) {
        self.interface_control.set_scanning_mode(scanning_mode);
    }
}
impl Drop for OffChannelOperation<'_, '_> {
    fn drop(&mut self) {
        // This accounts for the locked channel having changed during the operation.
        if let Some(channel) = self
            .interface_control
            .home_channel()
            .or(self.previously_locked_channel)
        {
            debug!("Switched back to channel {}.", channel);
            let _ = self.set_channel(channel);
        }
        self.set_scanning_mode(ScanningMode::Disabled);
        self.interface_control
            .shared_state
            .channel_state
            .lock(|cs| {
                let _ = cs
                    .try_borrow_mut()
                    .map(|mut channel_state| channel_state.off_channel_operation_interface = None);
            });
        self.interface_control
            .shared_state
            .off_channel_completion_signal
            .sender()
            .send(());
        debug!(
            "Off channel operation for interface {} finished.",
            self.rx_filter_interface
        );
    }
}
