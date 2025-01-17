#![no_std]

//! This module implements a station (STA) interface for FoA.
//!
//! ## Usage
//! The STA interface, that's provided consists of a few parts.
//!
//! | struct | purpose |
//! | -- | -- |
//! | [StaControl] | Controlling the STA interface. |
//! | [StaRunner] | The background runner for the STA interface. |
//! | [StaInput] | Entry point for frames addressed to this interface. |
//! | [StaSharedResources] | Shared resources for the STA interface. |
//! | [StaNetDevice] | [Device](ch::Device) for `embassy_net`. |
//!
//! For you the user, only the [StaInterface] and [StaControl] are relevant, since all other types
//! are internal.
//!
//! To use this interface, just call either [foa::new_with_single_interface] or [foa::new_with_multiple_interfaces], with [StaInterface] in the turbofish operator.
//! Which for this interface will return you `(StaControl, StaNetDevice)`
//!
//! ## Status
//! Currently only very basic operations are supported and much of this is work in progress. More
//! operations will follow in the future.
//!
//! ### Supported operations.
//! 1. Passive scanning
//! 2. Connecting to a network.
//! 3. Disconnecting from a network.
//! 4. Setting the MAC address.

use control::StaControl;
use embassy_net::driver::HardwareAddress;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::Duration;
use ieee80211::{
    common::{AssociationID, IEEE80211StatusCode},
    mac_parser::MACAddress,
};

use embassy_net_driver_channel::{self as ch};
use foa::{
    esp32_wifi_hal_rs::{BorrowedBuffer, WiFiRate},
    interface::Interface,
    lmac::{LMacError, LMacInterfaceControl, LMacTransmitEndpoint},
};

pub mod control;
mod runner;
pub use runner::StaRunner;
mod input;
pub use input::StaInput;
mod operations;
pub use operations::scan::ScanConfig;
use rx_router::RxRouter;
mod rx_router;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Errors than can occur, during STA operation.
pub enum StaError {
    /// An error occured in the lower MAC.
    LMacError(LMacError),
    /// The scan was unable to find the specified ESS.
    UnableToFindEss,
    /// An operation didn't complete in time.
    Timeout,
    /// Deserializing a received frame failed.
    FrameDeserializationFailed,
    /// Authentication failed with the specified status code.
    AuthenticationFailure(IEEE80211StatusCode),
    /// Association failed with the specified status code.
    AssociationFailure(IEEE80211StatusCode),
    /// An operation couldn't be carried out, due to an active connection.
    StillConnected,
    /// The status of a connection couldn't be changed, because none was established.
    NotConnected,
}

pub(crate) const DEFAULT_TIMEOUT: Duration = Duration::from_millis(200);

pub const MTU: usize = 1514;
pub(crate) const DEFAULT_PHY_RATE: WiFiRate = WiFiRate::PhyRate1ML;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Information about the current connection.
pub(crate) struct ConnectionInfo {
    /// The address of the BSS, we're connected to.
    pub bssid: MACAddress,
    /// Our own address.
    pub own_address: MACAddress,
    /// The association ID assigned by the AP.
    pub aid: AssociationID,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// All possible conneciton states, that the STA can be in.
pub(crate) enum ConnectionState {
    Disconnected,
    Connected(ConnectionInfo),
}
/// Tracks the connection state.
pub(crate) struct ConnectionStateTracker {
    connection_state: Mutex<NoopRawMutex, ConnectionState>,
    connection_state_signal: Signal<NoopRawMutex, ()>,
}
impl ConnectionStateTracker {
    /// Create a new state tracker.
    pub const fn new() -> Self {
        Self {
            connection_state: Mutex::new(ConnectionState::Disconnected),
            connection_state_signal: Signal::new(),
        }
    }
    /// Signal a new state.
    pub async fn signal_state(&self, new_state: ConnectionState) {
        *self.connection_state.lock().await = new_state;
        self.connection_state_signal.signal(());
    }
    /// Get the connection info.
    pub async fn connection_info(&self) -> Option<ConnectionInfo> {
        match *self.connection_state.lock().await {
            ConnectionState::Connected(connection_info) => Some(connection_info),
            ConnectionState::Disconnected => None,
        }
    }
    /// Wait for a connected state to be signaled.
    pub async fn wait_for_connection(&self) -> ConnectionInfo {
        loop {
            let ConnectionState::Connected(connection_info) = *self.connection_state.lock().await
            else {
                self.connection_state_signal.wait().await;
                continue;
            };
            break connection_info;
        }
    }
    /// Wait for a disconnected state to be signaled.
    pub async fn wait_for_disconnection(&self) {
        while matches!(
            *self.connection_state.lock().await,
            ConnectionState::Connected(_)
        ) {
            self.connection_state_signal.wait().await
        }
    }
}

/// Shared resources for the [StaInterface].
pub struct StaSharedResources<'res> {
    // RX routing.
    rx_router: RxRouter,
    bg_queue: Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,
    user_queue: Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,

    // Networking.
    channel_state: ch::State<MTU, 4, 4>,

    // State tracking.
    connection_state: ConnectionStateTracker,

    // Misc.
    interface_control: Option<LMacInterfaceControl<'res>>,
}
impl Default for StaSharedResources<'_> {
    fn default() -> Self {
        Self {
            rx_router: RxRouter::new(),
            bg_queue: Channel::new(),
            user_queue: Channel::new(),
            channel_state: ch::State::new(),
            connection_state: ConnectionStateTracker::new(),
            interface_control: None,
        }
    }
}

pub type StaNetDevice<'a> = embassy_net_driver_channel::Device<'a, MTU>;

/// A dummy type for a station interface.
///
/// This is intended to be used, with
/// [foa::new_with_single_interface].
pub struct StaInterface;
impl Interface for StaInterface {
    const NAME: &str = "STA";
    type SharedResourcesType<'res> = StaSharedResources<'res>;
    type ControlType<'res> = (StaControl<'res>, StaNetDevice<'res>);
    type RunnerType<'res> = StaRunner<'res>;
    type InputType<'res> = StaInput<'res>;
    type InitInfo = ();
    async fn new<'res>(
        sta_shared_state: &'res mut Self::SharedResourcesType<'res>,
        _init_info: Self::InitInfo,
        transmit_endpoint: LMacTransmitEndpoint<'res>,
        interface_control: LMacInterfaceControl<'res>,
        mac_address: [u8; 6],
    ) -> (
        Self::ControlType<'res>,
        Self::RunnerType<'res>,
        Self::InputType<'res>,
    ) {
        // Create a reference to the interface control, with the 'res lifetime.
        sta_shared_state.interface_control = Some(interface_control);
        let interface_control = sta_shared_state.interface_control.as_ref().unwrap();

        // Initialize embassy_net.
        let (net_runner, net_device) = ch::new(
            &mut sta_shared_state.channel_state,
            HardwareAddress::Ethernet(mac_address),
        );
        let (state_runner, rx_runner, tx_runner) = net_runner.split();

        (
            (
                StaControl {
                    transmit_endpoint,
                    interface_control,
                    mac_address: MACAddress::new(mac_address),
                    rx_router: &sta_shared_state.rx_router,
                    rx_queue: &sta_shared_state.user_queue,
                    connection_state: &sta_shared_state.connection_state,
                },
                net_device,
            ),
            StaRunner {
                rx_queue: &sta_shared_state.bg_queue,
                transmit_endpoint,
                interface_control,
                tx_runner,
                state_runner,
                connection_state: &sta_shared_state.connection_state,
            },
            StaInput {
                rx_runner,
                rx_router: &sta_shared_state.rx_router,
                bg_queue_sender: sta_shared_state.bg_queue.dyn_sender(),
                user_queue_sender: sta_shared_state.user_queue.dyn_sender(),
                connection_state: &sta_shared_state.connection_state,
            },
        )
    }
}
