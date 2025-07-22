#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Service parameter for a peer event.
pub struct PeerEventServiceParameters {
    /// Address of the peer.
    pub address: [u8; 6],
    /// Port of the AirDrop service.
    pub airdrop_port: Option<u16>,
    /// Port of the AirPlay service.
    pub airplay_port: Option<u16>,
}
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// An event from the AWDL interface.
pub enum AwdlEvent {
    /// A peer was discovered.
    ///
    /// The parameter is the address.
    PeerDiscovered([u8; 6]),
    /// A peer went stale and was evicted from the cache.
    ///
    /// The ports are the last known values for that peer.
    PeerWentStale(PeerEventServiceParameters),
    /// The services offered by a peer were discovered.
    ///
    /// This is as much service discovery as is currently possible.
    DiscoveredServiceForPeer {
        /// Address of the peer.
        address: [u8; 6],
        /// Port of the AirDrop service.
        airdrop_port: Option<u16>,
        /// Port of the AirPlay service.
        airplay_port: Option<u16>,
    },
}
