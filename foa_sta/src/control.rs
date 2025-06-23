//! This module provides control over the STA interface.

use ieee80211::{common::AssociationID, mac_parser::MACAddress};

use foa::{
    esp_wifi_hal::WiFiRate,
    util::operations::{deauthenticate, ScanConfig},
};
use rand_core::RngCore;

use crate::{
    connection_state::{ConnectionInfo, ConnectionState, DisconnectionReason},
    operations::{
        connect::{self, ConnectionParameters}, scan::{enumerate_bss, search_for_bss, BSS}
    },
    rsn::Credentials,
    rx_router::StaRxRouterEndpoint,
    ConnectionConfig, SecurityConfig, StaTxRx,
};

use super::StaError;

/// This provides control over the STA interface.
pub struct StaControl<'foa, 'vif, Rng: RngCore> {
    // Low level RX/TX.
    pub(crate) rx_router_endpoint: StaRxRouterEndpoint<'foa, 'vif>,
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,

    // Misc.
    pub(crate) mac_address: MACAddress,
    /// Entropy source for the STA implementation.
    pub(crate) rng: Rng,
}
impl<Rng: RngCore + Clone> StaControl<'_, '_, Rng> {
    /// Set the MAC address for the STA interface.
    pub fn set_mac_address(&mut self, mac_address: [u8; 6]) -> Result<(), StaError> {
        if self.sta_tx_rx.connection_state.connection_info().is_some() {
            Err(StaError::StillConnected)
        } else {
            self.mac_address = MACAddress::new(mac_address);
            Ok(())
        }
    }
    /// Randomize the MAC address.
    ///
    /// This will also return the MAC address.
    pub fn randomize_mac_address(&mut self) -> Result<[u8; 6], StaError> {
        let mut mac_address = [0x00; 6];
        self.rng.fill_bytes(mac_address.as_mut_slice());
        // By clearing the LSB of the first octet, we ensure that the local bit isn't set.
        mac_address[0] &= !(1);
        self.set_mac_address(mac_address).map(|_| mac_address)
    }

    /// Scan for networks.
    ///
    /// Invalid channels will cause an error to be returned.
    pub async fn scan<'a, const MAX_ESS: usize>(
        &'a mut self,
        scan_config: Option<ScanConfig<'a>>,
        found_bss: &'a mut heapless::FnvIndexMap<[u8; 6], BSS, MAX_ESS>,
    ) -> Result<(), StaError> {
        enumerate_bss(
            self.sta_tx_rx,
            &mut self.rx_router_endpoint,
            scan_config,
            found_bss,
        )
        .await
    }
    /// Look for a specific ESS and break once the first match is found.
    pub async fn find_ess<'a>(
        &'a mut self,
        scan_config: Option<ScanConfig<'a>>,
        ssid: &str,
    ) -> Result<BSS, StaError> {
        search_for_bss(
            self.sta_tx_rx,
            &mut self.rx_router_endpoint,
            scan_config,
            ssid,
        )
        .await
    }
    /// Connect to a network.
    ///
    /// If we're already connected to a network, this will disconnect from that network, before
    /// establishing a connection to the new network.
    pub async fn connect(
        &mut self,
        bss: BSS,
        connection_config: Option<ConnectionConfig>,
        credentials: Option<Credentials<'_>>,
    ) -> Result<(), StaError> {
        if bss.security_config != SecurityConfig::Open && credentials.is_none() {
            return Err(StaError::NoCredentialsForNetwork);
        }
        if let Some(connection_info) = self.sta_tx_rx.connection_state.connection_info() {
            if connection_info.bss.bssid == bss.bssid {
                return Err(StaError::SameNetwork);
            }
            debug!("Disconnecting from {}.", connection_info.bss.bssid);
            self.disconnect_internal(&connection_info).await;
        }
        self.sta_tx_rx.reset_phy_rate();
        let connection_config = connection_config.unwrap_or_default();
        let aid = connect::connect(
            self.sta_tx_rx,
            &mut self.rx_router_endpoint,
            &bss,
            &ConnectionParameters {
                phy_rate: self.sta_tx_rx.phy_rate(),
                config: connection_config,
                own_address: self.mac_address,
                credentials,
            },
            self.rng.clone(),
        )
        .await?;
        debug!("Successfully connected to {} : \"{}\"", bss.bssid, bss.ssid.as_str());
        self.sta_tx_rx
            .connection_state
            .signal_state(ConnectionState::Connected(ConnectionInfo {
                bss,
                own_address: self.mac_address,
                aid,
                connection_config,
            }));
        Ok(())
    }
    /// Connect to a network based on it's SSID.
    ///
    /// This will search for the network and connect to the first one it finds.
    pub async fn connect_by_ssid(
        &mut self,
        ssid: &str,
        connection_config: Option<ConnectionConfig>,
        credentials: Option<Credentials<'_>>,
    ) -> Result<(), StaError> {
        let bss = self.find_ess(None, ssid).await?;
        self.connect(bss, connection_config, credentials).await
    }
    async fn disconnect_internal(
        &mut self,
        ConnectionInfo {
            bss, own_address, ..
        }: &ConnectionInfo,
    ) {
        // NOTE: The channel is already unlocked here, but since there's no await-point between
        // unlocking the channel and transmitting the deauth, no other interface could attempt to
        // lock it before we're done here.
        self.sta_tx_rx
            .connection_state
            .signal_state(ConnectionState::Disconnected(DisconnectionReason::User));
        self.sta_tx_rx.reset_phy_rate();
        deauthenticate(
            self.sta_tx_rx.interface_control,
            bss.bssid,
            *own_address,
            true,
            self.sta_tx_rx.phy_rate(),
        )
        .await;
    }
    /// Disconnect from the current network.
    pub async fn disconnect(&mut self) -> Result<(), StaError> {
        let Some(connection_info) = self.sta_tx_rx.connection_state.connection_info() else {
            return Err(StaError::NotConnected);
        };
        self.sta_tx_rx.interface_control.unlock_channel();
        self.disconnect_internal(&connection_info).await;
        debug!("Disconnected from {}", connection_info.bss.bssid);
        Ok(())
    }
    /// Check if we're currently connected to a network.
    pub fn connected(&self) -> bool {
        self.sta_tx_rx.connection_state.connected()
    }
    /// Get the [AssociationID] of the current connection.
    pub fn get_aid(&self) -> Option<AssociationID> {
        self.sta_tx_rx
            .connection_state
            .map_connection_info(|connection_info| connection_info.aid)
    }
    /// Get the currently used PHY rate.
    pub fn phy_rate(&self) -> WiFiRate {
        self.sta_tx_rx.phy_rate()
    }
    /// Override the PHY rate.
    pub fn override_phy_rate(&self, phy_rate: WiFiRate) {
        self.sta_tx_rx.set_phy_rate(phy_rate);
    }
}
