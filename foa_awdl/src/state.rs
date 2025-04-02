use core::cell::{Cell, RefCell, RefMut};

use defmt_or_log::debug;
use embassy_net_driver_channel::State as NetState;
use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    channel::Channel,
    signal::Signal,
};
use embassy_time::Duration;
use ieee80211::mac_parser::MACAddress;

use crate::{
    event::AwdlEvent,
    peer::{AwdlPeer, ElectionState, OverlapSlotState, SynchronizationState},
    peer_cache::{AwdlPeerCache, StaticAwdlPeerCache},
    AWDL_MTU, EVENT_QUEUE_DEPTH, NET_RX_BUFFERS, NET_TX_BUFFERS,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
/// State of the AWDL interface pushed to the runner.
pub(crate) enum AwdlState {
    Active {
        our_address: MACAddress,
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

pub(crate) type AwdlPeerCacheImplementation = StaticAwdlPeerCache;
/// Resources shared between multiple components of the interface.
pub(crate) struct CommonResources {
    // State signaling
    /// Indicates the current status of the interface to the runner.
    pub(crate) state_signal: Signal<NoopRawMutex, AwdlState>,
    /// Indicates a peer pending for injection into the neighbor cache.
    pub(crate) ndp_inject_signal: Signal<NoopRawMutex, MACAddress>,
    /// User facing event queue.
    ///
    /// NOTE: This is also passed out to the user, but lives here so the runner can reset it at the
    /// start of a session.
    pub(crate) event_queue: Channel<NoopRawMutex, AwdlEvent, EVENT_QUEUE_DEPTH>,

    // State
    /// Stores all currently known peers.
    pub(crate) peer_cache: NoopMutex<RefCell<AwdlPeerCacheImplementation>>,
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
            event_queue: Channel::new(),
            peer_cache: NoopMutex::new(RefCell::new(AwdlPeerCacheImplementation::UNINIT)),
            dynamic_session_parameters: DynamicSessionParameters::new(),
            sync_state: NoopMutex::new(RefCell::new(SynchronizationState::UNINIT)),
            election_state: NoopMutex::new(RefCell::new(ElectionState::UNINIT)),
        }
    }
    /// Initialize the parameters for a new session.
    pub fn initialize_session_parameters(&self, channel: u8, address: MACAddress) {
        // Reset the neighbor injection signal, synchronization state, election state and clear
        // the user event queue.
        self.ndp_inject_signal.reset();
        self.event_queue.clear();
        self.lock_sync_state(|mut sync_state| *sync_state = SynchronizationState::new(channel));
        self.lock_election_state(|mut election_state| {
            *election_state = ElectionState::new(address)
        });
    }
    /// Acquire mutable access to the synchronization state in the closure.
    pub fn lock_sync_state<O>(&self, f: impl FnOnce(RefMut<'_, SynchronizationState>) -> O) -> O {
        self.sync_state
            .lock(|sync_state| (f)(sync_state.borrow_mut()))
    }
    /// Acquire mutable access to the election state in the closure.
    pub fn lock_election_state<O>(&self, f: impl FnOnce(RefMut<'_, ElectionState>) -> O) -> O {
        self.election_state
            .lock(|election_state| (f)(election_state.borrow_mut()))
    }
    /// Acquire mutable access to the peer cache in the closure.
    pub fn lock_peer_cache<O>(
        &self,
        f: impl FnOnce(RefMut<'_, AwdlPeerCacheImplementation>) -> O,
    ) -> O {
        self.peer_cache
            .lock(|peer_cache| (f)(peer_cache.borrow_mut()))
    }
    /// Check if the specified peer is our master.
    pub fn is_peer_master(&self, address: &MACAddress) -> bool {
        self.lock_election_state(|election_state| election_state.is_master(address))
    }
    /// Adjust the election and synchronization state to adopt the specified peer as our master.
    pub fn adopt_peer_as_master(&self, address: &MACAddress, peer: &AwdlPeer) {
        self.lock_sync_state(|mut sync_state| sync_state.adopt_master(&peer.synchronization_state));
        self.lock_election_state(|mut election_state| {
            election_state.adopt_master(address, &peer.election_state)
        });
    }
    /// Raise an event to the user.
    pub fn raise_user_event(&self, event: AwdlEvent) {
        if self.event_queue.try_send(event).is_err() {
            debug!("Failed to raise user event, since queue is full.");
        }
    }
    /// Choose a master for time synchronization.
    pub fn elect_master(&self, our_address: &MACAddress) {
        self.lock_election_state(|mut self_election_state| {
            if let Some((master_address, master_election_state, master_sync_state)) = self
                .lock_peer_cache(|peer_cache| {
                    peer_cache.find_master(&self_election_state, our_address)
                })
            {
                // If the master is new, we log the stats.
                if self_election_state.sync_peer != Some(master_address) {
                    debug!(
                        "Adopting {} as master. Metric: {} Counter: {} Hop Count: {}",
                        master_address,
                        master_election_state.self_metric,
                        master_election_state.self_counter,
                        master_election_state.distance_to_master
                    );
                }
                // We then adopt the master for the sync and election parameters, which will also
                // synchronize us.
                self.lock_sync_state(|mut self_sync_state| {
                    self_sync_state.adopt_master(&master_sync_state)
                });
                self_election_state.adopt_master(&master_address, &master_election_state);
            } else if !self_election_state.is_master(our_address) {
                // We weren't master before, but now are. This happens if we either win the election or
                // the peer cache is empty.
                debug!("We're master.");
                self_election_state.set_master_to_self(*our_address);
            }
        });
    }
    /// Get the nearest overlap slot with the specified peer.
    pub fn get_common_slot_for_peer(&self, peer_address: &MACAddress) -> OverlapSlotState {
        self.lock_sync_state(|our_sync_state| {
            self.lock_peer_cache(|peer_cache| {
                peer_cache.get_common_slot_for_peer(peer_address, &our_sync_state)
            })
        })
    }
    /// Remove all peers from the peer cache.
    pub fn clear_peer_cache(&self) {
        self.lock_peer_cache(|mut peer_cache| {
            <AwdlPeerCacheImplementation as AwdlPeerCache>::clear(&mut peer_cache)
        });
    }
    /// Check if the specified peer is in cache.
    pub fn is_peer_cached(&self, peer_address: &MACAddress) -> bool {
        self.lock_peer_cache(|peer_cache| {
            <AwdlPeerCacheImplementation as AwdlPeerCache>::contains_key(&peer_cache, peer_address)
        })
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
