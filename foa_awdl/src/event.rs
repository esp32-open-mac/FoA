#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// An event from the AWDL interface.
pub enum AwdlEvent {
    /// A peer was discovered.
    ///
    /// The parameter is the address.
    PeerDiscovered([u8; 6]),
    /// A peer went stale and was evicted from the cache.
    ///
    /// The parameter is the address.
    PeerWentStale([u8; 6]),
    /// The services offered by a peer were discovered.
    ///
    /// This is as much service discovery as is currently possible.
    DiscoveredServiceForPeer {
        address: [u8; 6],
        airdrop_port: Option<u16>,
        airplay_port: Option<u16>,
    },
}
