//! This module provides control over the STA interface.

use core::{marker::PhantomData, str::FromStr};

use embassy_time::{with_timeout, Duration};
use ieee80211::{
    common::{
        AssociationID, CapabilitiesInformation, IEEE80211AuthenticationAlgorithmNumber,
        IEEE80211Reason, IEEE80211StatusCode, SequenceControl,
    },
    element_chain,
    elements::{
        rates::{EncodedRate, ExtendedSupportedRatesElement, SupportedRatesElement},
        DSSSParameterSetElement, SSIDElement,
    },
    extended_supported_rates,
    mac_parser::MACAddress,
    mgmt_frame::{
        body::{AssociationRequestBody, AuthenticationBody, DeauthenticationBody},
        AssociationRequestFrame, AssociationResponseFrame, AuthenticationFrame, BeaconFrame,
        DeauthenticationFrame, ManagementFrameHeader,
    },
    scroll::{Pread, Pwrite},
    supported_rates,
};
use log::debug;

use foa::{
    esp32_wifi_hal_rs::{BorrowedBuffer, RxFilterBank, TxParameters},
    lmac::{LMacInterfaceControl, LMacTransmitEndpoint},
};

use crate::ConnectionStateTracker;

use super::{
    ConnectionInfo, ConnectionState, StaError, StaRxManagement, ASSOCIATING, AUTHENTICATING,
    DEFAULT_PHY_RATE, DEFAULT_TIMEOUT, SCANNING,
};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
/// Information about a BSS.
pub struct BSS {
    pub ssid: heapless::String<32>,
    pub channel: u8,
    pub bssid: MACAddress,
    pub last_rssi: i8,
}
/// Configuration for a scan.
///
/// This allows configuring how the scan is performed and on which channels.
pub struct ScanConfig<'a> {
    /// The time we should remain on one channel, before jumping to the next one.
    pub channel_remain_time: Duration,
    /// The channels to be scanned. Leaving this set to [None], will mean all channels will be
    /// scanned.
    pub channels: Option<&'a [u8]>,
}
impl ScanConfig<'_> {
    fn get_channels(&self) -> &[u8] {
        self.channels
            .unwrap_or(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13])
    }
}
impl Default for ScanConfig<'_> {
    fn default() -> Self {
        Self {
            channel_remain_time: Duration::from_millis(200),
            channels: None,
        }
    }
}
const DEFAULT_SUPPORTED_RATES: SupportedRatesElement<[EncodedRate; 8]> = supported_rates![
    5.5 B,
    11 B,
    1 B,
    2 B,
    6,
    12,
    24,
    48
];
const DEFAULT_XRATES: ExtendedSupportedRatesElement<[EncodedRate; 4]> =
    extended_supported_rates![54, 9, 18, 36];
/// Scan for ESS's.
///
/// This is it's own struct, to have easier cancel safety, since we can implement drop directly on
/// this, which will restore everything to it's original state.
struct ScanOperation<'a, 'res> {
    sta_control: &'a StaControl<'res>,
}
impl ScanOperation<'_, '_> {
    /// Scan for ESS's, with the specified [ScanConfig].
    ///
    /// If a beacon frame is received, the [BorrowedBuffer] is passed to `beacon_rx_cb`. If that
    /// returns `false`, we end the scan. This can be used to implement searching for a specific
    /// ESS and stopping once it's found.
    async fn run(
        self,
        scan_config: Option<ScanConfig<'_>>,
        mut beacon_rx_cb: impl FnMut(BorrowedBuffer<'_, '_>) -> bool,
    ) -> Result<(), StaError> {
        // We begin the off channe operation.
        let mut off_channel_operation = self
            .sta_control
            .interface_control
            .begin_interface_off_channel_operation(&self.sta_control.transmit_endpoint)
            .await
            .map_err(StaError::LMacError)?;

        // We get the scan configuration.
        let scan_config = scan_config.unwrap_or_default();
        debug!(
            "ESS scan started. Scanning channels: {:?}",
            scan_config.get_channels()
        );
        // Setup scanning mode.
        self.sta_control
            .rx_management
            .begin_user_operation(SCANNING);
        off_channel_operation.set_scanning_mode(true);

        // Loop through channels.
        for channel in scan_config.get_channels() {
            off_channel_operation
                .set_channel(*channel)
                .map_err(StaError::LMacError)?;
            // Receive on the channel, until the channel remain time is over.
            if with_timeout(scan_config.channel_remain_time, async {
                loop {
                    // If beacon_rx_cb returns false, we break.
                    if !beacon_rx_cb(self.sta_control.rx_from_user_queue().await) {
                        break;
                    }
                }
            })
            .await
            .is_ok()
            {
                break;
            }
        }

        debug!("ESS scan complete.");

        Ok(())
    }
}
impl Drop for ScanOperation<'_, '_> {
    fn drop(&mut self) {
        self.sta_control.rx_management.clear_user_operation();
    }
}
/// Connecting to an AP.
struct ConnectionOperation<'a, 'res> {
    sta_control: &'a StaControl<'res>,
}
impl ConnectionOperation<'_, '_> {
    /// Authenticate with the BSS.
    async fn do_auth(&self, bss: &BSS, timeout: Duration) -> Result<(), StaError> {
        self.sta_control
            .rx_management
            .begin_user_operation(AUTHENTICATING);
        let mut tx_buffer = self.sta_control.transmit_endpoint.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite_with(
                AuthenticationFrame {
                    header: ManagementFrameHeader {
                        receiver_address: bss.bssid,
                        bssid: bss.bssid,
                        transmitter_address: self.sta_control.mac_address,
                        sequence_control: SequenceControl::new(),
                        ..Default::default()
                    },
                    body: AuthenticationBody {
                        status_code: IEEE80211StatusCode::Success,
                        authentication_algorithm_number:
                            IEEE80211AuthenticationAlgorithmNumber::OpenSystem,
                        authentication_transaction_sequence_number: 1,
                        elements: element_chain! {},
                        _phantom: PhantomData,
                    },
                },
                0,
                false,
            )
            .unwrap();
        let _ = self
            .sta_control
            .transmit_endpoint
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..self
                        .sta_control
                        .interface_control
                        .get_default_tx_parameters()
                },
            )
            .await;
        // Due to the user operation being set to authenticating, we'll only receive authentication
        // frames.
        let Ok(response) = with_timeout(timeout, self.sta_control.rx_from_user_queue()).await
        else {
            debug!("Authentication timed out.");
            return Err(StaError::Timeout);
        };
        let Ok(auth_frame) = response.mpdu_buffer().pread::<AuthenticationFrame>(0) else {
            debug!(
                "Failed to authenticate with {}, frame deserialization failed.",
                bss.bssid
            );
            return Err(StaError::FrameDeserializationFailed);
        };
        if auth_frame.status_code == IEEE80211StatusCode::Success {
            debug!("Successfuly authenticated with {}.", bss.bssid);
            Ok(())
        } else {
            debug!(
                "Failed to authenticate with {}, status: {:?}.",
                bss.bssid, auth_frame.status_code
            );
            Err(StaError::AuthenticationFailure(auth_frame.status_code))
        }
    }
    /// Associate with the BSS.
    async fn do_assoc(&self, bss: &BSS, timeout: Duration) -> Result<AssociationID, StaError> {
        self.sta_control
            .rx_management
            .begin_user_operation(ASSOCIATING);
        let mut tx_buffer = self.sta_control.transmit_endpoint.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite_with(
                AssociationRequestFrame {
                    header: ManagementFrameHeader {
                        receiver_address: bss.bssid,
                        bssid: bss.bssid,
                        transmitter_address: self.sta_control.mac_address,
                        sequence_control: SequenceControl::new(),
                        ..Default::default()
                    },
                    body: AssociationRequestBody {
                        capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                        listen_interval: 0,
                        elements: element_chain! {
                            SSIDElement::new(bss.ssid.as_str()).unwrap(),
                            DEFAULT_SUPPORTED_RATES,
                            DEFAULT_XRATES
                        },
                        _phantom: PhantomData,
                    },
                },
                0,
                false,
            )
            .unwrap();

        let _ = self
            .sta_control
            .transmit_endpoint
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..self
                        .sta_control
                        .interface_control
                        .get_default_tx_parameters()
                },
            )
            .await;
        let Ok(received) = with_timeout(timeout, self.sta_control.rx_from_user_queue()).await
        else {
            debug!("Association timed out.");
            return Err(StaError::Timeout);
        };
        let assoc_response = received
            .mpdu_buffer()
            .pread::<AssociationResponseFrame>(0)
            .unwrap();
        if assoc_response.status_code == IEEE80211StatusCode::Success {
            debug!(
                "Successfuly associated with {}, AID: {:?}.",
                bss.bssid, assoc_response.association_id
            );
            Ok(assoc_response.association_id)
        } else {
            debug!(
                "Failed to associate with {}, status: {:?}.",
                bss.bssid, assoc_response.status_code
            );
            Err(StaError::AssociationFailure(assoc_response.status_code))
        }
    }
    async fn run(self, bss: &BSS, timeout: Duration) -> Result<(), StaError> {
        self.sta_control
            .interface_control
            .set_and_lock_channel(bss.channel)
            .await
            .map_err(StaError::LMacError)?;

        // Just to make sure, these are set for connection.
        self.sta_control.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            *self.sta_control.mac_address,
            None,
        );
        self.sta_control
            .interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, true);
        self.sta_control.interface_control.set_filter_parameters(
            RxFilterBank::BSSID,
            *bss.bssid,
            None,
        );
        self.sta_control
            .interface_control
            .set_filter_status(RxFilterBank::BSSID, true);

        self.do_auth(bss, timeout).await?;
        let aid = self.do_assoc(bss, timeout).await?;

        self.sta_control
            .connection_state
            .signal_state(ConnectionState::Connected(ConnectionInfo {
                bssid: bss.bssid,
                own_address: self.sta_control.mac_address,
                aid,
            }))
            .await;

        Ok(())
    }
}
impl Drop for ConnectionOperation<'_, '_> {
    fn drop(&mut self) {
        self.sta_control.rx_management.clear_user_operation();
    }
}

/// This provides control over the STA interface.
pub struct StaControl<'res> {
    // Low level RX/TX.
    pub(crate) rx_management: &'res StaRxManagement<'res>,
    pub(crate) transmit_endpoint: LMacTransmitEndpoint<'res>,
    pub(crate) interface_control: &'res LMacInterfaceControl<'res>,

    // Misc.
    pub(crate) mac_address: MACAddress,

    // Connection management.
    pub(crate) connection_state: &'res ConnectionStateTracker,
}
impl<'res> StaControl<'res> {
    /// Receive data from the user queue.
    async fn rx_from_user_queue(&self) -> BorrowedBuffer<'res, 'res> {
        self.rx_management.user_rx_queue.receive().await
    }

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
        ScanOperation { sta_control: self }
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
        ScanOperation { sta_control: self }
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
        ConnectionOperation { sta_control: self }
            .run(bss, timeout.unwrap_or(DEFAULT_TIMEOUT))
            .await?;
        debug!("Successfully connected to {}", bss.bssid);
        Ok(())
    }
    /// Disconnect from the current network.
    pub async fn disconnect(&mut self) {
        let Some(connection_info) = self.connection_state.connection_info().await else {
            return;
        };
        let mut tx_buf = self.transmit_endpoint.alloc_tx_buf().await;
        let written = tx_buf
            .pwrite(
                DeauthenticationFrame {
                    header: ManagementFrameHeader {
                        receiver_address: connection_info.bssid,
                        bssid: connection_info.bssid,
                        transmitter_address: connection_info.own_address,
                        sequence_control: SequenceControl::new(),
                        ..Default::default()
                    },
                    body: DeauthenticationBody {
                        reason: IEEE80211Reason::LeavingNetworkDeauth,
                        elements: element_chain! {},
                        _phantom: PhantomData,
                    },
                },
                0,
            )
            .unwrap();
        let _ = self
            .transmit_endpoint
            .transmit(
                &mut tx_buf[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..self.interface_control.get_default_tx_parameters()
                },
            )
            .await;
        self.connection_state
            .signal_state(ConnectionState::Disconnected)
            .await;
        debug!("Disconnected from {}", connection_info.bssid);
    }
}
