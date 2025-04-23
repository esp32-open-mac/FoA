use core::ops::Index;

use defmt_or_log::debug;
use embassy_time::Duration;
use heapless::FnvIndexMap;
use ieee80211::mac_parser::MACAddress;

use crate::{
    peer::{AwdlPeer, ElectionState, OverlapSlotState, SynchronizationState},
    PEER_CACHE_SIZE,
};

/// The peer cache is full, so no peer could be added.
pub(crate) struct PeerCacheFullError;

pub(crate) trait AwdlPeerCache: for<'a> Index<&'a MACAddress, Output = AwdlPeer> {
    const UNINIT: Self;

    fn iter(&self) -> impl Iterator<Item = (&MACAddress, &AwdlPeer)> + '_;
    fn retain(&mut self, pred: impl FnMut(&MACAddress, &mut AwdlPeer) -> bool);
    fn get(&self, address: &MACAddress) -> Option<&AwdlPeer>;
    fn get_mut(&mut self, address: &MACAddress) -> Option<&mut AwdlPeer>;
    fn is_full(&self) -> bool;
    fn insert(
        &mut self,
        address: MACAddress,
        peer: AwdlPeer,
    ) -> Result<Option<AwdlPeer>, (MACAddress, AwdlPeer)>;
    fn clear(&mut self);
    fn contains_key(&self, address: &MACAddress) -> bool;

    /// Modify the peer, or attempt to add it if it's not present.
    ///
    /// If the peer cache is full a [PeerCacheFullError].
    fn modify_or_add_peer(
        &mut self,
        peer_address: &MACAddress,
        mut modify: impl FnMut(&mut AwdlPeer),
        add: impl FnOnce() -> Option<AwdlPeer>,
    ) -> Result<bool, PeerCacheFullError> {
        Ok(match self.get_mut(peer_address) {
            Some(peer) => {
                (modify)(peer);
                false
            }
            None => {
                if self.is_full() {
                    return Err(PeerCacheFullError);
                } else {
                    let Some(peer) = (add)() else {
                        return Ok(false);
                    };
                    let _ = self.insert(*peer_address, peer);
                    true
                }
            }
        })
    }
    /// Remove all peers, from which we haven't received a frame within the specified timeout.
    fn purge_stale_peers(
        &mut self,
        mut purge_cb: impl FnMut(&MACAddress, &AwdlPeer),
        timeout: Duration,
    ) {
        self.retain(|address, peer| {
            let retain_peer = peer.last_frame.elapsed() < timeout;
            if !retain_peer {
                debug!("Removing {} from peer cache due to inactivity.", address);
                (purge_cb)(address, peer);
            }
            retain_peer
        })
    }
    /// Find the master from the current peer cache.
    ///
    /// If we're master None will be returned.
    fn find_master(
        &self,
        self_election_state: &ElectionState,
        address: &MACAddress,
    ) -> Option<(MACAddress, ElectionState, SynchronizationState)> {
        let mut most_eligible_peer = (address, self_election_state);

        for current_peer in self
            .iter()
            .map(|(address, peer)| (address, &peer.election_state))
        {
            if most_eligible_peer.1.is_more_eligible(current_peer.1) {
                most_eligible_peer = current_peer;
            }
        }
        if most_eligible_peer.0 == address {
            None
        } else {
            Some((
                *most_eligible_peer.0,
                *most_eligible_peer.1,
                self[most_eligible_peer.0].synchronization_state,
            ))
        }
    }
    /// Get the nearest common slot with the peer.
    fn get_common_slot_for_peer(
        &self,
        peer_address: &MACAddress,
        our_sync_state: &SynchronizationState,
    ) -> OverlapSlotState {
        if peer_address.is_multicast() {
            OverlapSlotState::OverlappingAt(our_sync_state.multicast_slot_start_timestamp())
        } else if let Some(peer) = self.get(peer_address) {
            peer.synchronization_state
                .next_overlap_timestamp(our_sync_state)
        } else {
            OverlapSlotState::PeerNotInCache
        }
    }
    /// Iterate over the cached peers and execute the closure on them.
    fn inspect_peers(&self, f: impl FnMut((&MACAddress, &AwdlPeer))) {
        self.iter().for_each(f);
    }
}
pub type StaticAwdlPeerCache = FnvIndexMap<MACAddress, AwdlPeer, PEER_CACHE_SIZE>;
impl AwdlPeerCache for StaticAwdlPeerCache {
    const UNINIT: Self = Self::new();
    fn get(&self, peer_address: &MACAddress) -> Option<&AwdlPeer> {
        self.get(peer_address)
    }
    fn get_mut(&mut self, peer_address: &MACAddress) -> Option<&mut AwdlPeer> {
        self.get_mut(peer_address)
    }
    fn iter(&self) -> impl Iterator<Item = (&MACAddress, &AwdlPeer)> + '_ {
        self.iter()
    }
    fn clear(&mut self) {
        self.clear();
    }
    fn retain(&mut self, pred: impl FnMut(&MACAddress, &mut AwdlPeer) -> bool) {
        self.retain(pred);
    }
    fn insert(
        &mut self,
        peer_address: MACAddress,
        peer: AwdlPeer,
    ) -> Result<Option<AwdlPeer>, (MACAddress, AwdlPeer)> {
        self.insert(peer_address, peer)
    }
    fn is_full(&self) -> bool {
        self.capacity() == self.len()
    }
    fn contains_key(&self, peer_address: &MACAddress) -> bool {
        self.contains_key(peer_address)
    }
}
