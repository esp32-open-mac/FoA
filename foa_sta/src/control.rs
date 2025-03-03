//! This module provides control over the STA interface.

use core::{cell::Cell, str::FromStr};

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::Duration;
use ieee80211::{
    common::AssociationID, elements::DSSSParameterSetElement, mac_parser::MACAddress,
    mgmt_frame::BeaconFrame, scroll::Pread,
};

use foa::{esp_wifi_hal::WiFiRate, LMacInterfaceControl, ReceivedFrame};

use crate::{
    operations::{
        connect::ConnectionOperation,
        deauth::send_deauth,
        scan::{ScanConfig, ScanOperation},
    },
    rx_router::{RxQueue, RxRouter},
    ConnectionInfo, ConnectionStateTracker,
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

/// This provides control over the STA interface.
pub struct StaControl<'vif, 'foa> {
    // Low level RX/TX.
    pub(crate) rx_router: &'vif RxRouter,
    pub(crate) rx_queue: &'vif Channel<NoopRawMutex, ReceivedFrame<'foa>, 4>,
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,

    // Misc.
    pub(crate) mac_address: MACAddress,

    // Connection management.
    pub(crate) connection_state: &'vif ConnectionStateTracker,
    pub(crate) phy_rate: &'vif Cell<WiFiRate>,
}
impl StaControl<'_, '_> {
    /// Set the MAC address for the STA interface.
    pub async fn set_mac_address(&mut self, mac_address: [u8; 6]) -> Result<(), StaError> {
        if self.connection_state.connection_info().is_some() {
            Err(StaError::StillConnected)
        } else {
            self.mac_address = MACAddress::new(mac_address);
            Ok(())
        }
    }

    /// Scan for networks.
    ///
    /// Invalid channels will cause an error to be returned.
    pub async fn scan<const MAX_ESS: usize>(
        &mut self,
        scan_config: Option<ScanConfig<'_>>,
        found_bss: &mut heapless::Vec<BSS, MAX_ESS>,
    ) -> Result<(), StaError> {
        // Putting this into a struct allows us to implement cancel safety more easily.
        ScanOperation {
            rx_router: self.rx_router,
            rx_queue: self.rx_queue,
            router_queue: RxQueue::User,
            interface_control: self.interface_control,
        }
        .run(scan_config, |received| {
            let Ok(beacon_frame) = received.mpdu_buffer().pread::<BeaconFrame>(0) else {
                return true;
            };
            let Some(ssid) = beacon_frame.ssid() else {
                return true;
            };
            if ssid.trim().is_empty() {
                return true;
            }
            if let Some(bss) = found_bss
                .iter_mut()
                .find(|bss| bss.bssid == beacon_frame.header.bssid)
            {
                bss.last_rssi = received.rssi();
            } else {
                let Some(channel) = beacon_frame
                    .elements
                    .get_first_element::<DSSSParameterSetElement>()
                    .map(|dsss_parameter_set| dsss_parameter_set.current_channel)
                else {
                    return true;
                };
                let Ok(ssid) = heapless::String::from_str(ssid) else {
                    return true;
                };
                let _ = found_bss.push(BSS {
                    ssid,
                    channel,
                    bssid: beacon_frame.header.bssid,
                    last_rssi: received.rssi(),
                });
            }
            true
        })
        .await
    }
    /// Look for a specific ESS and break once the first match is found.
    pub async fn find_ess(
        &mut self,
        scan_config: Option<ScanConfig<'_>>,
        ssid: &str,
    ) -> Result<BSS, StaError> {
        let mut ess = None;
        ScanOperation {
            rx_router: self.rx_router,
            rx_queue: self.rx_queue,
            router_queue: RxQueue::User,
            interface_control: self.interface_control,
        }
        .run(scan_config, |received| {
            let Ok(beacon_frame) = received.mpdu_buffer().pread::<BeaconFrame>(0) else {
                return true;
            };
            let Some(received_ssid) = beacon_frame.ssid() else {
                return true;
            };
            if received_ssid.trim().is_empty() {
                return true;
            }
            if ssid == received_ssid {
                let Some(channel) = beacon_frame
                    .elements
                    .get_first_element::<DSSSParameterSetElement>()
                    .map(|dsss_parameter_set| dsss_parameter_set.current_channel)
                else {
                    return true;
                };
                let Ok(ssid) = heapless::String::from_str(ssid) else {
                    return true;
                };
                ess = Some(BSS {
                    ssid,
                    channel,
                    bssid: beacon_frame.header.bssid,
                    last_rssi: received.rssi(),
                });
                return false;
            }
            true
        })
        .await?;
        ess.ok_or(StaError::UnableToFindEss)
    }
    /// Check if we're currently connected to a network.
    pub fn is_connected(&self) -> bool {
        self.connection_state.connection_info().is_some()
    }
    /// Connect to a network.
    ///
    /// If we're already connected to a network, this will disconnect from that network, before
    /// establishing a connection to the new network.
    pub async fn connect(&mut self, bss: &BSS, timeout: Option<Duration>) -> Result<(), StaError> {
        if let Some(connection_info) = self.connection_state.connection_info() {
            if connection_info.bssid == bss.bssid {
                return Err(StaError::SameNetwork);
            }
            debug!("Disconnecting from {}.", connection_info.bssid);
            self.disconnect_internal(connection_info).await;
        }
        let aid = ConnectionOperation {
            rx_router: self.rx_router,
            rx_queue: self.rx_queue,
            router_queue: RxQueue::User,
            interface_control: self.interface_control,
        }
        .run(
            bss,
            timeout.unwrap_or(DEFAULT_TIMEOUT),
            self.mac_address,
            self.phy_rate.get(),
        )
        .await?;
        self.connection_state
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
    async fn disconnect_internal(&mut self, connection_info: ConnectionInfo) {
        // NOTE: The channel is already unlocked here, but since there's no await-point between
        // unlocking the channel and transmitting the deauth, no other interface could attempt to
        // lock it before we're done here.
        self.connection_state
            .signal_state(ConnectionState::Disconnected);
        send_deauth(
            self.interface_control,
            &connection_info,
            self.phy_rate.get(),
        )
        .await;
        self.phy_rate.take();
    }
    /// Disconnect from the current network.
    pub async fn disconnect(&mut self) -> Result<(), StaError> {
        let Some(connection_info) = self.connection_state.connection_info() else {
            return Err(StaError::NotConnected);
        };
        self.interface_control.unlock_channel();
        self.disconnect_internal(connection_info).await;
        debug!("Disconnected from {}", connection_info.bssid);
        Ok(())
    }
    /// Get the [AssociationID] of the current connection.
    pub fn get_aid(&self) -> Option<AssociationID> {
        self.connection_state
            .connection_info()
            .map(|connection_info| connection_info.aid)
    }
    /// Get the currently used PHY rate.
    pub fn phy_rate(&self) -> WiFiRate {
        self.phy_rate.get()
    }
    /// Override the PHY rate.
    pub fn override_phy_rate(&self, phy_rate: WiFiRate) {
        self.phy_rate.set(phy_rate);
    }
}
