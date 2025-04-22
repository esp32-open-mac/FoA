use core::{marker::PhantomData, mem};

use embassy_futures::join::join;
use embassy_time::{Duration, WithTimeout};
use foa::{
    esp_wifi_hal::{RxFilterBank, TxParameters, WiFiRate},
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
    scroll::{ctx::TryIntoCtx, Pread, Pwrite},
};

use crate::{
    control::BSS,
    operations::{DEFAULT_SUPPORTED_RATES, DEFAULT_XRATES},
    rx_router::{StaRxRouterEndpoint, StaRxRouterOperation, StaRxRouterScopedOperation},
    StaError, StaTxRx,
};

pub struct ConnectionParameters {
    pub phy_rate: WiFiRate,
    pub retries: usize,
    pub timeout: Duration,
    pub own_address: MACAddress,
}

/// Connecting to an AP.
struct ConnectionOperation<'foa, 'vif, 'params> {
    sta_tx_rx: &'params StaTxRx<'foa, 'vif>,
    connection_parameters: &'params ConnectionParameters,
}
impl<'foa, 'vif, 'params> ConnectionOperation<'foa, 'vif, 'params> {
    fn complete(self) {
        mem::forget(self);
    }
    /// Send the specified frame and wait for a response.
    ///
    /// If no response is received in the specified timeout duration, or a transmission error
    /// occurs, the step will be retried as many times as specified. This compensates for a weird
    /// behavior of some APs, where they ACK a frame, but don't transmit a response. While this is
    /// rare, it can still occur, so this significantly stabilizes connection establishment.
    async fn do_bidirectional_connection_step(
        &self,
        router_operation: &StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        tx_frame: impl TryIntoCtx<bool, Error = ieee80211::scroll::Error>,
    ) -> Result<ReceivedFrame<'_>, StaError> {
        // Allocate a TX buffer and serialize the frame.
        let mut tx_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;
        let written = tx_buffer.pwrite(tx_frame, 0).unwrap();
        for _ in 0..=self.connection_parameters.retries {
            let res = self
                .sta_tx_rx
                .interface_control
                .transmit(
                    &mut tx_buffer[..written],
                    &TxParameters {
                        rate: self.connection_parameters.phy_rate,
                        ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
                    },
                    true,
                )
                .await;
            if res.is_err() {
                continue;
            }
            // Due to the user operation being set to authenticating, we'll only receive authentication
            // frames.
            if let Ok(frame) = router_operation
                .receive()
                .with_timeout(self.connection_parameters.timeout)
                .await
            {
                return Ok(frame);
            } else {
                trace!("Response to bidirectional connection step timed out.");
                continue;
            };
        }
        Err(StaError::ResponseTimeout)
    }
    /// Authenticate with the BSS.
    ///
    /// This currently only performs open system authentication.
    async fn do_auth(
        &self,
        router_operation: &StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        bss: &BSS,
    ) -> Result<(), StaError> {
        let auth_frame = AuthenticationFrame {
            header: ManagementFrameHeader {
                receiver_address: bss.bssid,
                bssid: bss.bssid,
                transmitter_address: self.connection_parameters.own_address,
                sequence_control: SequenceControl::new(),
                duration: 0,
                ..Default::default()
            },
            body: AuthenticationBody {
                status_code: IEEE80211StatusCode::Success,
                authentication_algorithm_number: IEEE80211AuthenticationAlgorithmNumber::OpenSystem,
                authentication_transaction_sequence_number: 1,
                elements: element_chain! {},
                _phantom: PhantomData,
            },
        };
        // Transmit an authentication frame and wait for the response.
        let response = self
            .do_bidirectional_connection_step(router_operation, auth_frame)
            .await?;
        // Try to parse the frame or return an error.
        let Ok(auth_frame) = response.mpdu_buffer().pread::<AuthenticationFrame>(0) else {
            debug!(
                "Failed to authenticate with {}, frame deserialization failed.",
                bss.bssid
            );
            return Err(StaError::FrameDeserializationFailed);
        };
        // Check if the authentication was successful and return an authentication failure if not.
        if auth_frame.status_code == IEEE80211StatusCode::Success {
            debug!("Successfully authenticated with {}.", bss.bssid);
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
    ///
    /// Like authentication, this only performs the bare minimum with a set of predetermined
    /// supported rates.
    async fn do_assoc(
        &self,
        router_operation: &StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        bss: &BSS,
    ) -> Result<AssociationID, StaError> {
        let assoc_request_frame = AssociationRequestFrame {
            header: ManagementFrameHeader {
                receiver_address: bss.bssid,
                bssid: bss.bssid,
                transmitter_address: self.connection_parameters.own_address,
                sequence_control: SequenceControl::new(),
                duration: 60,
                ..Default::default()
            },
            body: AssociationRequestBody {
                capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                listen_interval: 0,
                elements: element_chain! {
                    SSIDElement::new(bss.ssid.as_str()).ok_or(StaError::InvalidBss)?,
                    DEFAULT_SUPPORTED_RATES,
                    DEFAULT_XRATES
                },
                _phantom: PhantomData,
            },
        };
        // Transmit an association request and wait for the association response.
        let response = self
            .do_bidirectional_connection_step(router_operation, assoc_request_frame)
            .await?;
        // Try to parse the response or return an error.
        let Ok(assoc_response) = response.mpdu_buffer().pread::<AssociationResponseFrame>(0) else {
            debug!(
                "Failed to associate with {}, frame deserialization failed.",
                bss.bssid
            );
            return Err(StaError::FrameDeserializationFailed);
        };
        if let Some(aid) = assoc_response.association_id {
            if assoc_response.status_code == IEEE80211StatusCode::Success {
                debug!(
                    "Successfully associated with {}, AID: {:?}.",
                    bss.bssid, aid
                );
                return Ok(aid);
            }
        }
        debug!(
            "Failed to associate with {}, status: {:?}.",
            bss.bssid, assoc_response.status_code
        );
        Err(StaError::AssociationFailure(assoc_response.status_code))
    }
    fn configure_rx_filters(&self, bss: &BSS) {
        // Here we set and enable the BSSID and RA filters.

        self.sta_tx_rx.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            *self.connection_parameters.own_address,
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
    }
    async fn run(
        self,
        rx_router_endpoint: &'params mut StaRxRouterEndpoint<'foa, 'vif>,
        bss: &BSS,
    ) -> Result<AssociationID, StaError> {
        debug!(
            "Connecting to {} on channel {} with MAC address {}.",
            bss.bssid, bss.channel, self.connection_parameters.own_address
        );
        // Start the bringup operation for LMAC channel lock.
        let bringup_operation = self
            .sta_tx_rx
            .interface_control
            .begin_interface_bringup_operation(bss.channel)
            .map_err(StaError::LMacError)?;

        // Start the RX router operation, so that authentication and association frames are routed
        // to us for the duration of the connection bringup.
        // NOTE: If further protocol negotiations, like RSN, TDLS, FT etc. are added in the future,
        // the match statement in the RX router will have to be expanded, to route those frames
        // too.
        let (router_operation, _) = join(
            rx_router_endpoint.start_operation(StaRxRouterOperation::Connecting(
                self.connection_parameters.own_address,
            )),
            self.sta_tx_rx
                .interface_control
                .wait_for_off_channel_completion(),
        )
        .await;

        // Configure the RX filters to the specified addresses, so that we actually receive frames
        // from the AP.
        self.configure_rx_filters(bss);

        // Try to authenticate with the AP.
        self.do_auth(&router_operation, bss).await?;

        // Try to associate with the AP.
        let aid = self.do_assoc(&router_operation, bss).await?;

        // By marking the connection operation as completed, we forget self and therefore the drop
        // code never gets executed and the filter configuration remains in place.
        self.complete();
        router_operation.complete();
        bringup_operation.complete();

        Ok(aid)
    }
}
impl Drop for ConnectionOperation<'_, '_, '_> {
    fn drop(&mut self) {
        self.sta_tx_rx
            .interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, false);
        self.sta_tx_rx
            .interface_control
            .set_filter_status(RxFilterBank::BSSID, false);
    }
}
pub async fn connect<'foa, 'vif, 'params>(
    sta_tx_rx: &'params StaTxRx<'foa, 'vif>,
    rx_router_endpoint: &'params mut StaRxRouterEndpoint<'foa, 'vif>,
    bss: &'params BSS,
    connection_parameters: &'params ConnectionParameters,
) -> Result<AssociationID, StaError> {
    ConnectionOperation {
        sta_tx_rx,
        connection_parameters,
    }
    .run(rx_router_endpoint, bss)
    .await
}
