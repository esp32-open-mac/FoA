use core::ops::Index;

use defmt_or_log::derive_format_or_debug;
use heapless::FnvIndexMap;
use ieee80211::mac_parser::MACAddress;

use crate::{MAX_NUM_PEERS, state::MPMFSMState};

#[derive(Clone, PartialEq, Eq, Copy)]
#[derive_format_or_debug]
pub(crate) struct MeshPeerState {
    pub(crate) mpm_state: MPMFSMState,
}

impl Default for MeshPeerState {
    fn default() -> Self {
        Self {
            mpm_state: MPMFSMState::Idle,
        }
    }
}

pub(crate) struct PeerListFullError;

pub(crate) trait MeshPeerList:
    for<'a> Index<&'a MACAddress, Output = MeshPeerState>
{
    const UNINIT: Self;

    fn iter(&self) -> impl Iterator<Item = (&MACAddress, &MeshPeerState)> + '_;
    fn retain(&mut self, pred: impl FnMut(&MACAddress, &mut MeshPeerState) -> bool);
    fn get(&self, address: &MACAddress) -> Option<&MeshPeerState>;
    fn get_mut(&mut self, address: &MACAddress) -> Option<&mut MeshPeerState>;
    fn is_full(&self) -> bool;
    fn insert(
        &mut self,
        address: MACAddress,
        peer: MeshPeerState,
    ) -> Result<Option<MeshPeerState>, (MACAddress, MeshPeerState)>;
    fn clear(&mut self);
    fn contains_key(&self, address: &MACAddress) -> bool;

    /// Modify the peer, or attempt to add it if it's not present.
    ///
    /// If the peer list is full a [PeerListFullError].
    fn modify_or_add_peer(
        &mut self,
        peer_address: &MACAddress,
        mut modify: impl FnMut(&mut MeshPeerState),
        add: impl FnOnce() -> Option<MeshPeerState>,
    ) -> Result<bool, PeerListFullError> {
        Ok(match self.get_mut(peer_address) {
            Some(peer) => {
                (modify)(peer);
                false
            }
            None => {
                if self.is_full() {
                    return Err(PeerListFullError);
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

    /// Iterate over the mesh peers and execute the closure on them.
    fn inspect_peers(&self, f: impl FnMut((&MACAddress, &MeshPeerState))) {
        self.iter().for_each(f);
    }

    fn get_or_create(
        &mut self,
        peer_address: &MACAddress,
    ) -> Result<MeshPeerState, PeerListFullError> {
        Ok(match self.get(peer_address) {
            Some(peer) => *peer,
            None => {
                if self.is_full() {
                    return Err(PeerListFullError);
                } else {
                    let peer: MeshPeerState = Default::default();
                    let _ = self.insert(*peer_address, peer);
                    peer
                }
            }
        })
    }
}
pub(crate) type StaticMeshPeerList = FnvIndexMap<MACAddress, MeshPeerState, MAX_NUM_PEERS>;
impl MeshPeerList for StaticMeshPeerList {
    const UNINIT: Self = Self::new();
    fn get(&self, peer_address: &MACAddress) -> Option<&MeshPeerState> {
        self.get(peer_address)
    }
    fn get_mut(&mut self, peer_address: &MACAddress) -> Option<&mut MeshPeerState> {
        self.get_mut(peer_address)
    }
    fn iter(&self) -> impl Iterator<Item = (&MACAddress, &MeshPeerState)> + '_ {
        self.iter()
    }
    fn clear(&mut self) {
        self.clear();
    }
    fn retain(&mut self, pred: impl FnMut(&MACAddress, &mut MeshPeerState) -> bool) {
        self.retain(pred);
    }
    fn insert(
        &mut self,
        peer_address: MACAddress,
        peer: MeshPeerState,
    ) -> Result<Option<MeshPeerState>, (MACAddress, MeshPeerState)> {
        self.insert(peer_address, peer)
    }
    fn is_full(&self) -> bool {
        self.capacity() == self.len()
    }
    fn contains_key(&self, peer_address: &MACAddress) -> bool {
        self.contains_key(peer_address)
    }
}
