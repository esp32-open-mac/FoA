//! This module provides control over the STA interface.

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::Duration;
use ieee80211::{
    common::AssociationID,
    elements::DSSSParameterSetElement,
    mac_parser::MACAddress,
    mgmt_frame::{body::BeaconLikeBody, ManagementFrame},
};

use foa::{esp_wifi_hal::WiFiRate, ReceivedFrame};
use rand_core::RngCore;

use crate::{
    operations::{
        connect::ConnectionOperation,
        deauth::send_deauth,
        scan::{scan, ScanConfig, ScanType},
    },
    rx_router::RouterQueue,
    ConnectionInfo, StaTxRx, RX_QUEUE_LEN,
};

use super::{ConnectionState, StaError, DEFAULT_TIMEOUT};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
/// Information about a BSS.
pub struct BSS {
    pub ssid: heapless::String<32>,
    pub channel: u8,
    pub bssid: MACAddress,
    pub last_rssi: i8,
}
impl BSS {
    /// Create a [BSS] from the information in a beacon or probe response frame.
    pub fn from_beacon_like<Subtype>(
        frame: ManagementFrame<BeaconLikeBody<'_, Subtype>>,
        rssi: i8,
    ) -> Option<Self> {
        let mut ssid = heapless::String::new();
        let _ = ssid.push_str(frame.ssid()?);
        let channel = frame
            .elements
            .get_first_element::<DSSSParameterSetElement>()?
            .current_channel;
        let bssid = frame.header.bssid;
        Some(Self {
            ssid,
            channel,
            bssid,
            last_rssi: rssi,
        })
    }
}

/// This provides control over the STA interface.
pub struct StaControl<'foa, 'vif, Rng: RngCore> {
    // Low level RX/TX.
    pub(crate) rx_queue: &'vif Channel<NoopRawMutex, ReceivedFrame<'foa>, RX_QUEUE_LEN>,
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,

    // Misc.
    pub(crate) mac_address: MACAddress,
    /// Entropy source for the STA implementation.
    pub(crate) rng: Rng,
}
impl<Rng: RngCore> StaControl<'_, '_, Rng> {
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
    pub async fn scan<const MAX_ESS: usize>(
        &mut self,
        scan_config: Option<ScanConfig<'_>>,
        found_bss: &mut heapless::Vec<BSS, MAX_ESS>,
    ) -> Result<(), StaError> {
        scan(
            self.sta_tx_rx,
            self.rx_queue,
            RouterQueue::User,
            scan_config,
            ScanType::Enumerate(found_bss),
        )
        .await
    }
    /// Look for a specific ESS and break once the first match is found.
    pub async fn find_ess(
        &mut self,
        scan_config: Option<ScanConfig<'_>>,
        ssid: &str,
    ) -> Result<BSS, StaError> {
        let mut ess = None;
        scan::<0>(
            self.sta_tx_rx,
            self.rx_queue,
            RouterQueue::User,
            scan_config,
            ScanType::Search(ssid, &mut ess),
        )
        .await?;
        ess.ok_or(StaError::UnableToFindEss)
    }
    /// Check if we're currently connected to a network.
    pub fn is_connected(&self) -> bool {
        self.sta_tx_rx.connection_state.connection_info().is_some()
    }
    /// Connect to a network.
    ///
    /// If we're already connected to a network, this will disconnect from that network, before
    /// establishing a connection to the new network.
    pub async fn connect(&mut self, bss: &BSS, timeout: Option<Duration>) -> Result<(), StaError> {
        if let Some(connection_info) = self.sta_tx_rx.connection_state.connection_info() {
            if connection_info.bssid == bss.bssid {
                return Err(StaError::SameNetwork);
            }
            debug!("Disconnecting from {}.", connection_info.bssid);
            self.disconnect_internal(connection_info).await?;
        }
        self.sta_tx_rx.reset_phy_rate();
        let aid = ConnectionOperation {
            sta_tx_rx: self.sta_tx_rx,
            rx_queue: self.rx_queue,
            router_queue: RouterQueue::User,
        }
        .run(
            bss,
            timeout.unwrap_or(DEFAULT_TIMEOUT),
            self.mac_address,
            self.sta_tx_rx.phy_rate(),
            4,
        )
        .await?;
        self.sta_tx_rx
            .connection_state
            .signal_state(ConnectionState::Connected(ConnectionInfo {
                bssid: bss.bssid,
                own_address: self.mac_address,
                aid,
            }));
        debug!("Successfully connected to {}", bss.bssid);
        Ok(())
    }
    pub async fn connect_by_ssid(
        &mut self,
        ssid: &str,
        timeout: Option<Duration>,
    ) -> Result<(), StaError> {
        let bss = self.find_ess(None, ssid).await?;
        self.connect(&bss, timeout).await
    }
    async fn disconnect_internal(
        &mut self,
        ConnectionInfo {
            bssid, own_address, ..
        }: ConnectionInfo,
    ) -> Result<(), StaError> {
        // NOTE: The channel is already unlocked here, but since there's no await-point between
        // unlocking the channel and transmitting the deauth, no other interface could attempt to
        // lock it before we're done here.
        self.sta_tx_rx
            .connection_state
            .signal_state(ConnectionState::Disconnected);
        self.sta_tx_rx.reset_phy_rate();
        send_deauth(
            self.sta_tx_rx.interface_control,
            bssid,
            own_address,
            self.sta_tx_rx.phy_rate(),
        )
        .await
    }
    /// Disconnect from the current network.
    pub async fn disconnect(&mut self) -> Result<(), StaError> {
        let Some(connection_info) = self.sta_tx_rx.connection_state.connection_info() else {
            return Err(StaError::NotConnected);
        };
        self.sta_tx_rx.interface_control.unlock_channel();
        self.disconnect_internal(connection_info).await?;
        debug!("Disconnected from {}", connection_info.bssid);
        Ok(())
    }
    /// Get the [AssociationID] of the current connection.
    pub fn get_aid(&self) -> Option<AssociationID> {
        self.sta_tx_rx
            .connection_state
            .connection_info()
            .map(|connection_info| connection_info.aid)
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
