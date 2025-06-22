#![no_std]
#![deny(missing_docs)]
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

use core::cell::{Cell, RefCell};

use connection_state::ConnectionStateTracker;
use embassy_net::driver::HardwareAddress;
use embassy_sync::blocking_mutex::NoopMutex;
use esp_config::esp_config_int;
use ieee80211::{common::IEEE80211StatusCode, mac_parser::MACAddress};

use embassy_net_driver_channel::{self as ch};
use foa::{
    esp_wifi_hal::WiFiRate, util::rx_router::RxRouter, LMacError, LMacInterfaceControl,
    VirtualInterface,
};

#[macro_use]
extern crate defmt_or_log;

mod control;
pub use control::*;
pub use operations::scan::BSS;

mod runner;
use rand_core::RngCore;
use rsn::CryptoState;
pub use runner::StaRunner;
use runner::{ConnectionRunner, RoutingRunner};
mod operations;
mod rx_router;
use rx_router::StaRxRouter;
mod connection_state;
pub use connection_state::ConnectionConfig;
mod rsn;
pub use rsn::{Credentials, SecurityConfig};

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
    /// The provided BSS was invalid.
    InvalidBss,
    /// Another internal operation is already in progress.
    RouterOperationAlreadyInProgress,
    /// No credentials were provided for a network, that requires them.
    NoCredentialsForNetwork,
    /// The 4-Way Handshake failed.
    FourWayHandshakeFailure,
    /// No hardware key slots were unavailable.
    NoKeySlotsAvailable,
    /// The provided PSK length didn't match what was expected.
    InvalidPskLength,
    /// The group key handshake failed.
    GroupKeyHandshakeFailure,
}

/// The MTU used by the STA interface.
pub const MTU: usize = 1514;

/// TX and RX resources for the interface control and runner.
pub(crate) struct StaTxRx<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) connection_state: &'vif ConnectionStateTracker,
    pub(crate) crypto_state: &'vif NoopMutex<RefCell<Option<CryptoState<'foa>>>>,
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
    pub fn map_crypto_state<O, F: FnMut(&mut CryptoState<'_>) -> O>(&self, f: F) -> Option<O> {
        self.crypto_state.lock(|cs| cs.borrow_mut().as_mut().map(f))
    }
    pub fn rsna_activated(&self) -> bool {
        self.map_crypto_state(|_| {}).is_some()
    }
}

pub(crate) const RX_QUEUE_DEPTH: usize = esp_config_int!(usize, "FOA_STA_CONFIG_RX_QUEUE_DEPTH");
pub(crate) const NET_TX_BUFFERS: usize = esp_config_int!(usize, "FOA_STA_CONFIG_NET_TX_BUFFERS");
pub(crate) const NET_RX_BUFFERS: usize = esp_config_int!(usize, "FOA_STA_CONFIG_NET_RX_BUFFERS");
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
    crypto_state: NoopMutex<RefCell<Option<CryptoState<'foa>>>>,
}
impl StaResources<'_> {
    /// Create new resources for the STA interface.
    pub const fn new() -> Self {
        Self {
            rx_router: RxRouter::new(),
            channel_state: ch::State::new(),
            connection_state: ConnectionStateTracker::new(),
            phy_rate: Cell::new(WiFiRate::PhyRate1ML),
            sta_tx_rx: None,
            crypto_state: NoopMutex::new(RefCell::new(None)),
        }
    }
}
impl Default for StaResources<'_> {
    fn default() -> Self {
        Self::new()
    }
}

/// embassy_net device for the STA interface.
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
        crypto_state: &resources.crypto_state,
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
