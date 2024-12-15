//! This module defines traits for implementing an interface, which can be used by FoA.
//!
//! An interface in FoA is split into three main parts. The background runner, which handles all
//! background tasks of the interface. The user facing control, which grants the user control over
//! the interface. And the interface input, which is used by the background task, to pass received
//! frames on to the interface.
//! Typically a set of queues would be used, to pass the [BorrowedBuffer]s from the interface input
//! to the background task or user control. For an example see the [STA interface](crate::sta).
use core::future::Future;

use esp32_wifi_hal_rs::BorrowedBuffer;

use crate::lmac::{LMacInterfaceControl, LMacTransmitEndpoint};

/// The background runner for the interface.
pub trait InterfaceRunner {
    /// Run the background task for the interface.
    async fn run(self) -> !;
}
/// The interface input for the input.
pub trait InterfaceInput<'res> {
    /// When a frame is received for this interface, the MAC task will call this function, with the
    /// buffer returned by the driver.
    ///
    /// WARNING: This function should return as quickly as possible, since otherwise the LMAC will
    /// be stalled.
    async fn interface_input(&mut self, borrowed_buffer: BorrowedBuffer<'res, 'res>);
}
#[doc(hidden)]
/// This is used for representing an unused interface.
impl<'res> InterfaceInput<'res> for () {
    async fn interface_input(&mut self, _borrowed_buffer: BorrowedBuffer<'res, 'res>) {}
}
/// The interface itself.
///
/// This is usually a dummy type, which just represents this interface. See
/// [StaInterface](crate::sta::StaInterface).
pub trait Interface {
    /// The name of the interface.
    const NAME: &str;

    /// Resources shared by all parts of the interface.
    type SharedResourcesType<'res>: Default;
    /// The user facing control type for the interface.
    ///
    /// NOTE: This may also be a tuple of a control struct and for example a net driver for
    /// `embassy_net`.
    type ControlType<'res>;
    /// The background for the interface.
    type RunnerType<'res>: InterfaceRunner;
    /// The interface input for the interface.
    type InputType<'res>: InterfaceInput<'res>;
    /// Initialization info for the interface.
    type InitInfo;
    fn new<'res>(
        shared_resources: &'res mut Self::SharedResourcesType<'res>,
        init_info: Self::InitInfo,
        transmit_endpoint: LMacTransmitEndpoint<'res>,
        interface_control: LMacInterfaceControl<'res>,
        mac_address: [u8; 6],
    ) -> impl Future<
        Output = (
            Self::ControlType<'res>,
            Self::RunnerType<'res>,
            Self::InputType<'res>,
        ),
    >;
}
