use core::cell::RefCell;

use crate::PEER_CACHE_SIZE;
use awdl_frame_parser::action_frame::DefaultAWDLActionFrame;
use defmt::debug;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Instant};
use heapless::FnvIndexMap;
use ieee80211::mac_parser::MACAddress;

pub(crate) struct AwdlPeer {
    pub last_frame: Instant,
}
impl AwdlPeer {
    pub fn new_with_action_frame(_awdl_action_body: &DefaultAWDLActionFrame<'_>) -> Self {
        Self {
            last_frame: Instant::now(),
        }
    }
    pub fn update(&mut self, _awdl_action_body: &DefaultAWDLActionFrame<'_>) {
        self.last_frame = Instant::now();
    }
}

pub(crate) struct PeerCacheFullError;

pub(crate) struct StaticAwdlPeerCache {
    peers: NoopMutex<RefCell<FnvIndexMap<MACAddress, AwdlPeer, PEER_CACHE_SIZE>>>,
}
impl StaticAwdlPeerCache {
    pub const fn new() -> Self {
        Self {
            peers: NoopMutex::new(RefCell::new(FnvIndexMap::new())),
        }
    }
    pub fn is_full(&self) -> bool {
        self.peers.lock(|peers| {
            // This can not fail, since the only way the RefCell could already be in a borrowed
            // state, is if we locked the mutex again in here.
            peers
                .try_borrow()
                .map(|peers| peers.len() == peers.capacity())
                .unwrap_or(false)
        })
    }
    /// Inspect and modify the peer, or attempt to add it.
    ///
    /// If the peer cache is full a [PeerCacheFullError].
    pub fn inspect_or_add_peer<I: FnOnce(&mut AwdlPeer), A: FnOnce() -> AwdlPeer>(
        &self,
        address: &MACAddress,
        inspect: I,
        add: A,
    ) -> Result<(), PeerCacheFullError> {
        self.peers.lock(|peers| {
            peers
                .try_borrow_mut()
                .map(|mut peers| {
                    match peers.get_mut(address) {
                        Some(peer) => (inspect)(peer),
                        None => {
                            if !self.is_full() {
                                let _ = peers.insert(*address, (add)());
                            } else {
                                return Err(PeerCacheFullError);
                            }
                        }
                    };
                    Ok(())
                })
                .unwrap_or(Ok(()))
        })
    }
    /// Remove all peers, from which we haven't received a frame within the specified timeout.
    pub fn purge_stale_peers(&self, timeout: Duration) {
        self.peers.lock(|peers| {
            let _ = peers.try_borrow_mut().map(|mut peers| {
                peers.retain(|address, peer| {
                    let retain_peer = peer.last_frame.elapsed() < timeout;
                    if !retain_peer {
                        debug!("Removing {} from peer cache due to inactivity.", address);
                    }
                    retain_peer
                })
            });
        })
    }
}
