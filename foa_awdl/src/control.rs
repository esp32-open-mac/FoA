use core::net::Ipv6Addr;

use embassy_time::Duration;
use foa::{esp_wifi_hal::RxFilterBank, LMacInterfaceControl};
use rand_core::RngCore;

use crate::{
    hw_address_to_ipv6,
    peer::AwdlPeer,
    peer_cache::AwdlPeerCache,
    state::{AwdlState, CommonResources},
    AwdlError, AWDL_BSSID,
};

/// Control interface for the AWDL interface.
pub struct AwdlControl<'foa, 'vif, Rng: RngCore> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) rng: Rng,
    pub(crate) channel: u8,
    pub(crate) mac_address: [u8; 6],
}
impl<Rng: RngCore> AwdlControl<'_, '_, Rng> {
    /// Set and enable all filters required for the interface.
    fn enable_filters(&self) {
        self.interface_control
            .set_filter_parameters(RxFilterBank::BSSID, AWDL_BSSID, None);
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, true);
        self.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            self.mac_address,
            None,
        );
        self.interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, true);
    }
    /// Disable all filter for the interface.
    fn disable_filters(&self) {
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, false);
        self.interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, false);
    }
    /// Start an AWDL session.
    ///
    /// If a session is already in progress, it will be immediately aborted and the new session
    /// takes over.
    pub async fn start(&mut self) -> Result<(), AwdlError> {
        let bringup_operation = self
            .interface_control
            .begin_interface_bringup_operation(self.channel)
            .map_err(|_| AwdlError::FailedToAcquireChannelLock)?;
        self.interface_control
            .wait_for_off_channel_completion()
            .await;
        self.enable_filters();
        self.common_resources
            .state_signal
            .signal(AwdlState::Active {
                our_address: self.mac_address,
                channel: self.channel,
            });
        bringup_operation.complete();
        Ok(())
    }
    /// Stop the currently active AWDL session.
    ///
    /// If no session is in progress, this won't do anything.
    pub fn stop(&mut self) {
        self.interface_control.unlock_channel();
        self.disable_filters();
        self.common_resources
            .state_signal
            .signal(AwdlState::Inactive);
    }
    /// Set the timeout for a peer to count as stale and be purged from the peer cache.
    ///
    /// The timeout specifies the threshold for when a peer counts as stale. If no frame was
    /// received from a peer within that threshold, the peer will count as stale and be removed.
    /// The change will take effect after the previous timeout duration has passed.
    pub fn set_peer_stale_timeout(&self, timeout: Duration) {
        self.common_resources
            .dynamic_session_parameters
            .stale_peer_timeout
            .set(timeout);
    }
    /// Set the MAC address of the interface.
    ///
    /// This will only take effect after restarting the interface.
    pub fn set_mac_address(&mut self, mac_address: [u8; 6]) {
        self.mac_address = mac_address;
    }
    /// Get the MAC address of the interface.
    pub fn mac_address(&self) -> [u8; 6] {
        self.mac_address
    }
    /// Randomize the MAC address.
    ///
    /// This will also return the MAC address.
    pub fn randomize_mac_address(&mut self) -> [u8; 6] {
        let mut mac_address = [0x00; 6];
        self.rng.fill_bytes(mac_address.as_mut_slice());
        // By clearing the LSB of the first octet, we ensure that the local bit isn't set.
        mac_address[0] &= !(1);
        self.set_mac_address(mac_address);
        mac_address
    }
    /// Get the IPv6 address of this interface.
    pub fn own_ipv6_addr(&self) -> Ipv6Addr {
        hw_address_to_ipv6(&self.mac_address)
    }
    /// Inspect the peers, that are currently in cache.
    ///
    /// The provided closure will be called for every cached peer, with the first parameter being
    /// the peers address and the second a reference to the peer.
    pub fn inspect_peers(&self, f: impl FnMut((&[u8; 6], &AwdlPeer))) {
        self.common_resources
            .lock_peer_cache(move |peer_cache| peer_cache.inspect_peers(f));
    }
}
