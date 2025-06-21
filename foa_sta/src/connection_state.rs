use core::{cell::RefCell, ops::Deref};

use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    signal::Signal,
};
use embassy_time::Duration;
use ieee80211::{common::AssociationID, mac_parser::MACAddress};

use crate::BSS;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
/// Configuration parameters for the connection.
pub struct ConnectionConfig {
    /// If we're disconnected from the network and this is set to `true`, we attempt to reconnect.
    pub automatic_reconnect: bool,
    /// How often to retry a step in the connection establishment, if no response is received.
    pub handshake_retries: usize,
    /// Maximum time a response can take during a handshake step.
    pub handshake_timeout: Duration,
}
impl Default for ConnectionConfig {
    fn default() -> Self {
        Self {
            automatic_reconnect: true,
            handshake_retries: 4,
            handshake_timeout: Duration::from_millis(100),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
/// Information about the current connection.
pub(crate) struct ConnectionInfo {
    /// The BSS we're connected to.
    pub bss: BSS,
    /// Our own address.
    pub own_address: MACAddress,
    /// The association ID assigned by the AP.
    pub aid: AssociationID,
    /// Configuration parameters for connection setup and management.
    pub connection_config: ConnectionConfig,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The reason for a disconnection.
pub enum DisconnectionReason {
    User,
    BeaconTimeout,
    Deauthenticated,
}
#[derive(Clone, Debug, PartialEq, Eq)]
/// All possible conneciton states, that the STA can be in.
pub(crate) enum ConnectionState {
    Disconnected(DisconnectionReason),
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
            connection_state: NoopMutex::new(RefCell::new(ConnectionState::Disconnected(
                DisconnectionReason::User,
            ))),
            connection_state_signal: Signal::new(),
        }
    }
    /// Signal a new state.
    pub fn signal_state(&self, new_state: ConnectionState) {
        self.connection_state
            .lock(|state| *state.borrow_mut() = new_state);
        self.connection_state_signal.signal(());
    }
    /// Get the connection info.
    pub fn connection_info(&self) -> Option<ConnectionInfo> {
        self.connection_state.lock(|state| {
            if let ConnectionState::Connected(connection_info) = state.borrow().deref() {
                Some(connection_info.clone())
            } else {
                None
            }
        })
    }
    /// Get the reason for the last disconnection.
    pub fn disconnection_reason(&self) -> Option<DisconnectionReason> {
        self.connection_state.lock(|state| {
            if let ConnectionState::Disconnected(reason) = state.borrow().deref() {
                Some(*reason)
            } else {
                None
            }
        })
    }
    /// Map the connection info.
    ///
    /// Compared to calling [Option::map] on the [Option<ConnectionInfo>] returned by
    /// [Self::connection_info], this avoids copying the entire [ConnectionInfo].
    pub fn map_connection_info<O>(&self, f: impl FnOnce(&ConnectionInfo) -> O) -> Option<O> {
        self.connection_state.lock(|state| {
            if let ConnectionState::Connected(ref connection_info) = state.borrow().deref() {
                Some((f)(connection_info))
            } else {
                None
            }
        })
    }
    /// Check if the current connection state is [ConnectionState::Connected].
    ///
    /// While you could also use [Option::is_some] on [Self::connection_info], that would incur an
    /// unnecessary copy of the [ConnectionState] struct, which this avoids.
    pub fn connected(&self) -> bool {
        self.connection_state
            .lock(|state| matches!(state.borrow().deref(), &ConnectionState::Connected(_)))
    }
    /// Wait for a connected state to be signaled.
    pub async fn wait_for_connection(&self) -> ConnectionInfo {
        loop {
            if !self.connected() {
                self.connection_state_signal.wait().await;
            }
            let Some(connection_info) = self.connection_info() else {
                continue;
            };
            break connection_info;
        }
    }
    /// Wait for a disconnected state to be signaled.
    pub async fn wait_for_disconnection(&self) -> DisconnectionReason {
        loop {
            if let Some(disconnection_reason) = self.disconnection_reason() {
                return disconnection_reason;
            }
            self.connection_state_signal.wait().await;
        }
    }
    #[allow(unused)]
    /// Get the BSSID and our own address for the current connection.
    pub fn bssid_and_own_address(&self) -> Option<(MACAddress, MACAddress)> {
        self.map_connection_info(|connection_info| (connection_info.bss.bssid, connection_info.own_address))
    }
}
