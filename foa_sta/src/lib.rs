#![no_std]

//! This module implements a station (STA) interface for FoA.
//!
//! ## Usage
//! To initialize a STA interface, call [new_sta_interface] with a [VirtualInterface] you got from
//! calling [foa::init], a mutable reference to [StaResources] and optionally a [MACAddress].
//! In return, you'll get a `(StaControl, StaRunner, StaNetDevice)`. For the stack to work, you
//! have to call [StaRunner::run], either in a separate task or by joining the future with another
//! one. Once you have done that, you can use the [StaControl] to control the interface, and pass
//! the [StaNetDevice] to [embassy_net].
//! ## Status
//! Currently only very basic operations are supported and much of this is work in progress. More
//! operations will follow in the future.
//!
//! ### Supported operations.
//! 1. Passive scanning
//! 2. Connecting to a network.
//! 3. Disconnecting from a network.
//! 4. Setting the MAC address.

use core::cell::{Cell, RefCell};

use embassy_net::driver::HardwareAddress;
use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    channel::Channel,
    signal::Signal,
};
use embassy_time::Duration;
use ieee80211::{
    common::{AssociationID, IEEE80211StatusCode},
    mac_parser::MACAddress,
};

use embassy_net_driver_channel::{self as ch};
use foa::{esp_wifi_hal::WiFiRate, LMacError, ReceivedFrame, VirtualInterface};

#[macro_use]
extern crate defmt_or_log;

mod control;
pub use control::*;

mod runner;
pub use runner::StaRunner;
use runner::{ConnectionRunner, RoutingRunner};
mod operations;
pub use operations::scan::ScanConfig;
use rx_router::RxRouter;
mod rx_router;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Errors than can occur, during STA operation.
pub enum StaError {
    /// An error occured in the lower MAC.
    LMacError(LMacError),
    /// The scan was unable to find the specified ESS.
    UnableToFindEss,
    /// No ACK was received in time.
    AckTimeout,
    /// No response from the BSS was received in time, although an ACK was received.
    ResponseTimeout,
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
    /// The network, too which a connection was requested, is the same as the current one.
    SameNetwork,
}

pub(crate) const DEFAULT_TIMEOUT: Duration = Duration::from_millis(200);

pub const MTU: usize = 1514;

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
    connection_state: NoopMutex<RefCell<ConnectionState>>,
    connection_state_signal: Signal<NoopRawMutex, ()>,
}
impl ConnectionStateTracker {
    /// Create a new state tracker.
    pub const fn new() -> Self {
        Self {
            connection_state: NoopMutex::new(RefCell::new(ConnectionState::Disconnected)),
            connection_state_signal: Signal::new(),
        }
    }
    /// Signal a new state.
    pub fn signal_state(&self, new_state: ConnectionState) {
        self.connection_state
            .lock(|rc| *rc.borrow_mut() = new_state);
        self.connection_state_signal.signal(());
    }
    /// Get the connection info.
    pub fn connection_info(&self) -> Option<ConnectionInfo> {
        match self.connection_state.lock(|rc| *rc.borrow()) {
            ConnectionState::Connected(connection_info) => Some(connection_info),
            ConnectionState::Disconnected => None,
        }
    }
    /// Wait for a connected state to be signaled.
    pub async fn wait_for_connection(&self) -> ConnectionInfo {
        loop {
            let ConnectionState::Connected(connection_info) =
                self.connection_state.lock(|rc| *rc.borrow())
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
            self.connection_state.lock(|rc| *rc.borrow()),
            ConnectionState::Connected(_)
        ) {
            self.connection_state_signal.wait().await
        }
    }
}

/// Shared resources for the STA interface.
pub struct StaResources<'foa> {
    // RX routing.
    rx_router: RxRouter,
    bg_queue: Channel<NoopRawMutex, ReceivedFrame<'foa>, 4>,
    user_queue: Channel<NoopRawMutex, ReceivedFrame<'foa>, 4>,

    // Networking.
    channel_state: ch::State<MTU, 4, 4>,

    // State tracking.
    connection_state: ConnectionStateTracker,
    phy_rate: Cell<WiFiRate>,
}
impl StaResources<'_> {
    pub const fn new() -> Self {
        Self {
            rx_router: RxRouter::new(),
            bg_queue: Channel::new(),
            user_queue: Channel::new(),
            channel_state: ch::State::new(),
            connection_state: ConnectionStateTracker::new(),
            phy_rate: Cell::new(WiFiRate::PhyRate1ML),
        }
    }
}
impl Default for StaResources<'_> {
    fn default() -> Self {
        Self::new()
    }
}

pub type StaNetDevice<'a> = embassy_net_driver_channel::Device<'a, MTU>;

pub fn new_sta_interface<'vif, 'foa>(
    virtual_interface: &'vif mut VirtualInterface<'foa>,
    resources: &'vif mut StaResources<'foa>,
    mac_address: Option<[u8; 6]>,
) -> (
    StaControl<'vif, 'foa>,
    StaRunner<'vif, 'foa>,
    StaNetDevice<'vif>,
) {
    let (interface_control, interface_rx_queue) = virtual_interface.split();
    let mac_address = mac_address.unwrap_or(interface_control.get_factory_mac_for_interface());
    // Initialize embassy_net.
    let (net_runner, net_device) = ch::new(
        &mut resources.channel_state,
        HardwareAddress::Ethernet(mac_address),
    );
    let (state_runner, rx_runner, tx_runner) = net_runner.split();
    (
        StaControl {
            interface_control,
            mac_address: MACAddress::new(mac_address),
            connection_state: &resources.connection_state,
            rx_queue: &resources.user_queue,
            rx_router: &resources.rx_router,
            phy_rate: &resources.phy_rate,
        },
        StaRunner {
            connection_runner: ConnectionRunner {
                bg_rx_queue: &resources.bg_queue,
                interface_control,
                tx_runner: Some(tx_runner),
                state_runner,
                connection_state: &resources.connection_state,
                phy_rate: &resources.phy_rate,
            },
            routing_runner: RoutingRunner {
                connection_state: &resources.connection_state,
                bg_queue_sender: resources.bg_queue.dyn_sender(),
                interface_rx_queue,
                rx_router: &resources.rx_router,
                rx_runner,
                user_queue_sender: resources.user_queue.dyn_sender(),
            },
        },
        net_device,
    )
}
