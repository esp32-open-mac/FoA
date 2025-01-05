//! This module provides control over the STA interface.

use core::str::FromStr;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::Duration;
use ieee80211::{
    common::AssociationID, elements::DSSSParameterSetElement, mac_parser::MACAddress,
    mgmt_frame::BeaconFrame, scroll::Pread,
};
use log::debug;

use foa::{
    esp32_wifi_hal_rs::BorrowedBuffer,
    lmac::{LMacInterfaceControl, LMacTransmitEndpoint},
};

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
pub struct StaControl<'res> {
    // Low level RX/TX.
    pub(crate) rx_router: &'res RxRouter,
    pub(crate) rx_queue: &'res Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,
    pub(crate) transmit_endpoint: LMacTransmitEndpoint<'res>,
    pub(crate) interface_control: &'res LMacInterfaceControl<'res>,

    // Misc.
    pub(crate) mac_address: MACAddress,

    // Connection management.
    pub(crate) connection_state: &'res ConnectionStateTracker,
}
impl<'res> StaControl<'res> {
    /// Set the MAC address for the STA interface.
    pub async fn set_mac_address(&mut self, mac_address: [u8; 6]) -> Result<(), StaError> {
        if self.connection_state.connection_info().await.is_some() {
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
            transmit_endpoint: &self.transmit_endpoint,
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
            transmit_endpoint: &self.transmit_endpoint,
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
    /// Connect to a network.
    ///
    /// NOTE: Dropping the future returned by this, may leave the channel locked by this interface,
    /// even though no connection was ever established.
    pub async fn connect(&mut self, bss: &BSS, timeout: Option<Duration>) -> Result<(), StaError> {
        let aid = ConnectionOperation {
            rx_router: self.rx_router,
            rx_queue: self.rx_queue,
            router_queue: RxQueue::User,
            interface_control: self.interface_control,
            transmit_endpoint: &self.transmit_endpoint,
        }
        .run(bss, timeout.unwrap_or(DEFAULT_TIMEOUT), self.mac_address)
        .await?;
        self.connection_state
            .signal_state(ConnectionState::Connected(ConnectionInfo {
                bssid: bss.bssid,
                own_address: self.mac_address,
                aid,
            }))
            .await;
        debug!("Successfully connected to {}", bss.bssid);
        Ok(())
    }
    /// Disconnect from the current network.
    pub async fn disconnect(&mut self) -> Result<(), StaError> {
        let Some(connection_info) = self.connection_state.connection_info().await else {
            return Err(StaError::NotConnected);
        };
        send_deauth(
            &self.transmit_endpoint,
            self.interface_control,
            &connection_info,
        )
        .await;
        self.connection_state
            .signal_state(ConnectionState::Disconnected)
            .await;
        debug!("Disconnected from {}", connection_info.bssid);
        Ok(())
    }
    /// Get the [AssociationID] of the current connection.
    pub async fn get_aid(&self) -> Option<AssociationID> {
        self.connection_state
            .connection_info()
            .await
            .map(|connection_info| connection_info.aid)
    }
}
