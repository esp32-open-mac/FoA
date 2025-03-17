use embassy_time::Duration;
use foa::{esp_wifi_hal::RxFilterBank, LMacInterfaceControl};
use ieee80211::mac_parser::MACAddress;
use rand_core::RngCore;

use crate::{AwdlError, AwdlState, CommonResources, AWDL_BSSID};

/// Control interface for the AWDL interface.
pub struct AwdlControl<'foa, 'vif, Rng: RngCore> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) rng: Rng,
    pub(crate) channel: u8,
    pub(crate) mac_address: MACAddress,
}
impl<Rng: RngCore> AwdlControl<'_, '_, Rng> {
    /// Set and enable all filters required for the interface.
    fn enable_filters(&self) {
        self.interface_control
            .set_filter_parameters(RxFilterBank::BSSID, *AWDL_BSSID, None);
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, true);
        self.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            *self.mac_address,
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
        self.interface_control
            .lock_channel(self.channel)
            .map_err(|_| AwdlError::FailedToAcquireChannelLock)?;
        self.interface_control
            .wait_for_off_channel_completion()
            .await;
        self.enable_filters();
        self.common_resources
            .state_signal
            .signal(AwdlState::Active {
                mac_address: self.mac_address,
                channel: self.channel,
            });
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
}
