#![no_std]

//! This crate implements a station (STA) interface for FoA.
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

use core::cell::Cell;

use embassy_net::driver::HardwareAddress;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Duration;
use esp_config::esp_config_int;
use ieee80211::{
    common::{AssociationID, IEEE80211StatusCode},
    mac_parser::MACAddress,
};

use embassy_net_driver_channel::{self as ch};
use foa::{
    esp_wifi_hal::WiFiRate,
    rx_router::{RxRouter, RxRouterEndpoint, RxRouterInput},
    LMacError, LMacInterfaceControl, VirtualInterface,
};

#[macro_use]
extern crate defmt_or_log;

mod control;
pub use control::*;

mod runner;
use rand_core::RngCore;
pub use runner::StaRunner;
use runner::{ConnectionRunner, RoutingRunner};
mod operations;
pub use operations::scan::ScanConfig;
mod rx_router;
use rx_router::StaRxRouterOperation;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Errors than can occur, during STA operation.
pub enum StaError {
    /// An error occured in the lower MAC.
    LMacError(LMacError),
    /// The TX buffers provided by the LMAC are too small. This is a config issue with FoA.
    TxBufferTooSmall,
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
    /// The provided BSS was invalid.
    InvalidBss,
    /// Another internal operation is already in progress.
    RouterOperationAlreadyInProgress,
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
    connection_state: Cell<ConnectionState>,
    connection_state_signal: Signal<NoopRawMutex, ()>,
}
impl ConnectionStateTracker {
    /// Create a new state tracker.
    pub const fn new() -> Self {
        Self {
            connection_state: Cell::new(ConnectionState::Disconnected),
            connection_state_signal: Signal::new(),
        }
    }
    /// Signal a new state.
    pub fn signal_state(&self, new_state: ConnectionState) {
        self.connection_state.set(new_state);
        self.connection_state_signal.signal(());
    }
    /// Get the connection info.
    pub fn connection_info(&self) -> Option<ConnectionInfo> {
        match self.connection_state.get() {
            ConnectionState::Connected(connection_info) => Some(connection_info),
            ConnectionState::Disconnected => None,
        }
    }
    /// Wait for a connected state to be signaled.
    pub async fn wait_for_connection(&self) -> ConnectionInfo {
        loop {
            let ConnectionState::Connected(connection_info) = self.connection_state.get() else {
                self.connection_state_signal.wait().await;
                continue;
            };
            break connection_info;
        }
    }
    /// Wait for a disconnected state to be signaled.
    pub async fn wait_for_disconnection(&self) {
        while matches!(self.connection_state.get(), ConnectionState::Connected(_)) {
            self.connection_state_signal.wait().await
        }
    }
}

#[derive(Clone)]
/// TX and RX resources for the interface control and runner.
pub(crate) struct StaTxRx<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) connection_state: &'vif ConnectionStateTracker,
    phy_rate: &'vif Cell<WiFiRate>,
}
impl StaTxRx<'_, '_> {
    /// Reset the PHY rate.
    pub fn reset_phy_rate(&self) {
        self.phy_rate.take();
    }
    /// Get the current PHY rate.
    pub fn phy_rate(&self) -> WiFiRate {
        self.phy_rate.get()
    }
    /// Set the current PHY rate.
    pub fn set_phy_rate(&self, phy_rate: WiFiRate) {
        self.phy_rate.set(phy_rate);
    }
    /// Check if we are currently performing an off channel operation.
    pub fn in_off_channel_operation(&self) -> bool {
        self.interface_control.off_channel_operation_interface()
            == Some(self.interface_control.get_filter_interface())
    }
}

pub(crate) const RX_QUEUE_LEN: usize = esp_config_int!(usize, "FOA_STA_CONFIG_RX_QUEUE_LEN");
pub(crate) const NET_TX_BUFFERS: usize = esp_config_int!(usize, "FOA_STA_CONFIG_NET_TX_BUFFERS");
pub(crate) const NET_RX_BUFFERS: usize = esp_config_int!(usize, "FOA_STA_CONFIG_NET_RX_BUFFERS");

pub(crate) type StaRxRouterEndpoint<'foa, 'router> =
    RxRouterEndpoint<'foa, 'router, RX_QUEUE_LEN, StaRxRouterOperation>;
pub(crate) type StaRxRouterInput<'foa, 'router> =
    RxRouterInput<'foa, 'router, RX_QUEUE_LEN, StaRxRouterOperation>;
pub(crate) type StaRxRouter<'foa> = RxRouter<'foa, RX_QUEUE_LEN, StaRxRouterOperation>;
/// Shared resources for the STA interface.
pub struct StaResources<'foa> {
    // RX routing.
    rx_router: StaRxRouter<'foa>,

    // Networking.
    channel_state: ch::State<MTU, NET_RX_BUFFERS, NET_TX_BUFFERS>,

    // State tracking.
    connection_state: ConnectionStateTracker,
    phy_rate: Cell<WiFiRate>,

    // Misc.
    sta_tx_rx: Option<StaTxRx<'static, 'static>>,
}
impl StaResources<'_> {
    pub const fn new() -> Self {
        Self {
            rx_router: RxRouter::new(),
            channel_state: ch::State::new(),
            connection_state: ConnectionStateTracker::new(),
            phy_rate: Cell::new(WiFiRate::PhyRate1ML),
            sta_tx_rx: None,
        }
    }
}
impl Default for StaResources<'_> {
    fn default() -> Self {
        Self::new()
    }
}

pub type StaNetDevice<'a> = embassy_net_driver_channel::Device<'a, MTU>;

/// Initialize a new STA interface.
pub fn new_sta_interface<'foa: 'vif, 'vif, Rng: RngCore + Clone>(
    virtual_interface: &'vif mut VirtualInterface<'foa>,
    resources: &'vif mut StaResources<'foa>,
    rng: Rng,
) -> (
    StaControl<'foa, 'vif, Rng>,
    StaRunner<'foa, 'vif>,
    StaNetDevice<'vif>,
) {
    let (interface_control, interface_rx_queue) = virtual_interface.split();
    let mac_address = interface_control.get_factory_mac_for_interface();
    // Initialize embassy_net.
    let (net_runner, net_device) = ch::new(
        &mut resources.channel_state,
        HardwareAddress::Ethernet(mac_address),
    );
    // This is done to prevent all sorts of weirdness with self referential structs.
    // SAFETY:
    // Both 'foa and 'vif are shorter lifetimes, than static, and due to StaTxRx only refering to
    // fields that are in StaResources, which has the lifetime 'vif, this is safe.
    let sta_tx_rx = unsafe {
        core::mem::transmute::<
            &'vif mut Option<StaTxRx<'static, 'static>>,
            &'vif mut Option<StaTxRx<'foa, 'vif>>,
        >(&mut resources.sta_tx_rx)
    }
    .insert(StaTxRx {
        interface_control,
        connection_state: &resources.connection_state,
        phy_rate: &resources.phy_rate,
    });
    let (state_runner, rx_runner, tx_runner) = net_runner.split();
    let (rx_router_input, [foreground_endpoint, background_endpoint]) = resources.rx_router.split();
    (
        StaControl {
            sta_tx_rx,
            mac_address: MACAddress::new(mac_address),
            rx_router_endpoint: foreground_endpoint,
            rng,
        },
        StaRunner {
            tx_runner,
            connection_runner: ConnectionRunner {
                sta_tx_rx,
                rx_router_endpoint: background_endpoint,
                state_runner,
            },
            routing_runner: RoutingRunner {
                rx_router_input,
                sta_tx_rx,
                interface_rx_queue,
                rx_runner,
            },
        },
        net_device,
    )
}
