use core::{marker::PhantomData, mem};

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{with_timeout, Duration};
use foa::{
    esp_wifi_hal::{RxFilterBank, TxParameters, WiFiError, WiFiRate},
    LMacInterfaceControl, ReceivedFrame,
};
use ieee80211::{
    common::{
        AssociationID, CapabilitiesInformation, IEEE80211AuthenticationAlgorithmNumber,
        IEEE80211StatusCode, SequenceControl,
    },
    element_chain,
    elements::SSIDElement,
    mac_parser::MACAddress,
    mgmt_frame::{
        body::{AssociationRequestBody, AuthenticationBody},
        AssociationRequestFrame, AssociationResponseFrame, AuthenticationFrame,
        ManagementFrameHeader,
    },
    scroll::{Pread, Pwrite},
};

use crate::{
    control::BSS,
    operations::{DEFAULT_SUPPORTED_RATES, DEFAULT_XRATES},
    rx_router::{Operation, RouterQueue},
    StaError, StaTxRx,
};

/// Connecting to an AP.
pub struct ConnectionOperation<'foa, 'vif> {
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,
    pub(crate) rx_queue: &'vif Channel<NoopRawMutex, ReceivedFrame<'foa>, 4>,
    pub(crate) router_queue: RouterQueue,
}
impl ConnectionOperation<'_, '_> {
    fn defuse(self) {
        mem::forget(self);
    }
    /// Authenticate with the BSS.
    async fn do_auth(
        &self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
        phy_rate: WiFiRate,
    ) -> Result<(), StaError> {
        let mut tx_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite_with(
                AuthenticationFrame {
                    header: ManagementFrameHeader {
                        receiver_address: bss.bssid,
                        bssid: bss.bssid,
                        transmitter_address: mac_address,
                        sequence_control: SequenceControl::new(),
                        duration: 60,
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
        let res = self
            .sta_tx_rx
            .interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: phy_rate,
                    ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
                },
                true,
            )
            .await;
        if res == Err(WiFiError::AckTimeout) {
            return Err(StaError::AckTimeout);
        }
        // Due to the user operation being set to authenticating, we'll only receive authentication
        // frames.
        let Ok(response) = with_timeout(timeout, self.rx_queue.receive()).await else {
            debug!("Authentication timed out.");
            return Err(StaError::ResponseTimeout);
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
    async fn do_assoc(
        &self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
        phy_rate: WiFiRate,
    ) -> Result<AssociationID, StaError> {
        let mut tx_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite_with(
                AssociationRequestFrame {
                    header: ManagementFrameHeader {
                        receiver_address: bss.bssid,
                        bssid: bss.bssid,
                        transmitter_address: mac_address,
                        sequence_control: SequenceControl::new(),
                        duration: 60,
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
            .sta_tx_rx
            .interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: phy_rate,
                    ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
                },
                true,
            )
            .await;
        let Ok(received) = with_timeout(timeout, self.rx_queue.receive()).await else {
            debug!("Association timed out.");
            return Err(StaError::ResponseTimeout);
        };
        let Ok(assoc_response) = received.mpdu_buffer().pread::<AssociationResponseFrame>(0) else {
            debug!(
                "Failed to associate with {}, frame deserialization failed.",
                bss.bssid
            );
            return Err(StaError::FrameDeserializationFailed);
        };
        if assoc_response.status_code != IEEE80211StatusCode::Success
            || assoc_response.association_id.is_none()
        {
            debug!(
                "Failed to associate with {}, status: {:?}.",
                bss.bssid, assoc_response.status_code
            );
            Err(StaError::AssociationFailure(assoc_response.status_code))
        } else {
            let aid = assoc_response.association_id.unwrap();
            debug!("Successfuly associated with {}, AID: {:?}.", bss.bssid, aid);
            Ok(aid)
        }
    }
    pub async fn run(
        self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
        phy_rate: WiFiRate,
    ) -> Result<AssociationID, StaError> {
        debug!(
            "Connecting to {} on channel {} with MAC address {}.",
            bss.bssid, bss.channel, mac_address
        );
        let bringup_operation = self
            .sta_tx_rx
            .interface_control
            .begin_interface_bringup_operation(bss.channel)
            .map_err(StaError::LMacError)?;
        let operation = self
            .sta_tx_rx
            .rx_router
            .begin_scoped_operation(self.router_queue, Operation::Connecting(mac_address))
            .await;

        // Just to make sure, these are set for connection.
        self.sta_tx_rx.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            *mac_address,
            None,
        );
        self.sta_tx_rx
            .interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, true);
        self.sta_tx_rx.interface_control.set_filter_parameters(
            RxFilterBank::BSSID,
            *bss.bssid,
            None,
        );
        self.sta_tx_rx
            .interface_control
            .set_filter_status(RxFilterBank::BSSID, true);

        self.do_auth(bss, timeout, mac_address, phy_rate).await?;
        let aid = self.do_assoc(bss, timeout, mac_address, phy_rate).await?;

        bringup_operation.complete();
        operation.complete();

        self.defuse();

        Ok(aid)
    }
}
impl Drop for ConnectionOperation<'_, '_> {
    fn drop(&mut self) {
        self.sta_tx_rx
            .interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, false);
        self.sta_tx_rx
            .interface_control
            .set_filter_status(RxFilterBank::BSSID, false);
    }
}
