use foa::{LMacInterfaceControl, esp_wifi_hal::RxFilterBank};
use ieee80211::mac_parser::MACAddress;

use crate::{CommonResources, MeshError, rx_router::MeshRxRouterEndpoint, state::MeshState};

pub struct MeshControl<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_router_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
    pub(crate) common_resources: &'vif CommonResources,

    pub(crate) channel: u8,
    pub(crate) mac_address: MACAddress,
    pub(crate) mesh_id: heapless::String<32>,
}

impl MeshControl<'_, '_> {
    /// Set and enable all filters required for the interface.
    fn enable_filters(&self) {
        self.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            *self.mac_address,
            None,
        );
        self.interface_control
            .set_filter_parameters(RxFilterBank::BSSID, *self.mac_address, None);
        self.interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, true);
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, true);
        self.interface_control
            .set_scanning_mode(foa::esp_wifi_hal::ScanningMode::ManagementAndData);
    }
    /// Disable all filter for the interface.
    fn disable_filters(&self) {
        self.interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, false);
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, false);
    }
    /// Start a Mesh session.
    ///
    /// If a session is already in progress, it will be immediately aborted and the new session
    /// takes over.
    pub async fn start(&mut self) -> Result<(), MeshError> {
        let bringup_operation = self
            .interface_control
            .begin_interface_bringup_operation(self.channel)
            .map_err(|_| MeshError::FailedToAcquireChannelLock)?;
        self.interface_control
            .wait_for_off_channel_completion()
            .await;
        self.enable_filters();
        self.common_resources
            .state_signal
            .signal(MeshState::Active {
                our_address: self.mac_address,
                channel: self.channel,
                mesh_id: self.mesh_id.clone(),
            });
        bringup_operation.complete();
        Ok(())
    }
    /// Stop the currently active Mesh session.
    ///
    /// If no session is in progress, this won't do anything.
    pub fn stop(&mut self) {
        self.interface_control.unlock_channel();
        self.disable_filters();
        self.common_resources
            .state_signal
            .signal(MeshState::Inactive);
    }

    /// Set the MAC address of the interface.
    ///
    /// This will only take effect after restarting the interface.
    pub fn set_mac_address(&mut self, mac_address: [u8; 6]) {
        self.mac_address = MACAddress::new(mac_address);
    }
}
