pub enum AwdlEvent {
    PeerDiscovered([u8; 6]),
    PeerWentStale([u8; 6]),
    DiscoveredServiceForPeer {
        address: [u8; 6],
        airdrop_port: Option<u16>,
        airplay_port: Option<u16>,
    },
}
