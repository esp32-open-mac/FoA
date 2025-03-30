use core::cell::{Cell, RefCell, RefMut};

use embassy_net_driver_channel::State as NetState;
use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    signal::Signal,
};
use embassy_time::Duration;
use ieee80211::mac_parser::MACAddress;

use crate::{
    peer::{ElectionState, StaticAwdlPeerCache, SynchronizationState},
    AWDL_MTU, NET_RX_BUFFERS, NET_TX_BUFFERS,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum AwdlState {
    Active {
        mac_address: MACAddress,
        channel: u8,
    },
    Inactive,
}
/// Parameters that may change over the course of a session.
pub(crate) struct DynamicSessionParameters {
    /// When the time that has elapsed since the last frame from a peer is greater than this
    /// timeout, the peer is considered stale.
    pub(crate) stale_peer_timeout: Cell<Duration>,
    /// The country code to be broadcast in the data path TLV.
    pub(crate) country_code: Cell<[char; 2]>,
}
impl DynamicSessionParameters {
    pub const fn new() -> Self {
        Self {
            stale_peer_timeout: Cell::new(Duration::from_secs(3)),
            country_code: Cell::new(['X', '0']),
        }
    }
}

/// Resources shared between multiple components of the interface.
pub(crate) struct CommonResources {
    // State signaling
    /// Indicates the current status of the interface to the runner.
    pub(crate) state_signal: Signal<NoopRawMutex, AwdlState>,
    /// Indicates a peer pending for injection into the neighbor cache.
    pub(crate) ndp_inject_signal: Signal<NoopRawMutex, MACAddress>,

    // State
    /// Stores all currently known peers.
    pub(crate) peer_cache: StaticAwdlPeerCache,
    /// Dynamically changing parameters.
    pub(crate) dynamic_session_parameters: DynamicSessionParameters,
    /// Synchronization state for the currently active session if any.
    pub(crate) sync_state: NoopMutex<RefCell<SynchronizationState>>,
    /// Election state for the currently active session if any.
    pub(crate) election_state: NoopMutex<RefCell<ElectionState>>,
}
impl CommonResources {
    pub const fn new() -> Self {
        Self {
            state_signal: Signal::new(),
            ndp_inject_signal: Signal::new(),
            peer_cache: StaticAwdlPeerCache::new(),
            dynamic_session_parameters: DynamicSessionParameters::new(),
            sync_state: NoopMutex::new(RefCell::new(SynchronizationState::UNINIT)),
            election_state: NoopMutex::new(RefCell::new(ElectionState::UNINIT)),
        }
    }
    /// Initialize the parameters for a new session.
    pub fn initialize_session_parameters(&self, channel: u8, address: MACAddress) {
        self.ndp_inject_signal.reset();
        self.lock_sync_state(|mut sync_state| *sync_state = SynchronizationState::new(channel));
        self.lock_election_state(|mut election_state| {
            *election_state = ElectionState::new(address)
        });
    }
    /// Acquire mutable access to the synchronization state in the closure.
    pub fn lock_sync_state<O>(
        &self,
        mut f: impl FnMut(RefMut<'_, SynchronizationState>) -> O,
    ) -> O {
        self.sync_state
            .lock(|sync_state| (f)(sync_state.borrow_mut()))
    }
    /// Acquire mutable access to the election state in the closure.
    pub fn lock_election_state<O>(&self, mut f: impl FnMut(RefMut<'_, ElectionState>) -> O) -> O {
        self.election_state
            .lock(|election_state| (f)(election_state.borrow_mut()))
    }
}

/// Resources for the AWDL interface.
pub struct AwdlResources {
    /// Resources common to all components.
    pub(crate) common_resources: CommonResources,
    /// Resources for embassy_net_driver_channel.
    pub(crate) net_state: NetState<AWDL_MTU, NET_RX_BUFFERS, NET_TX_BUFFERS>,
}
impl AwdlResources {
    pub const fn new() -> Self {
        Self {
            common_resources: CommonResources::new(),
            net_state: NetState::new(),
        }
    }
}
impl Default for AwdlResources {
    fn default() -> Self {
        Self::new()
    }
}
