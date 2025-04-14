use core::{marker::PhantomData, mem};

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{with_timeout, Duration};
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
    rx_router::{Operation, RouterQueue},
    StaError, StaTxRx, RX_QUEUE_LEN,
};

/// Connecting to an AP.
pub struct ConnectionOperation<'foa, 'vif> {
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,
    pub(crate) rx_queue: &'vif Channel<NoopRawMutex, ReceivedFrame<'foa>, RX_QUEUE_LEN>,
    pub(crate) router_queue: RouterQueue,
}
impl ConnectionOperation<'_, '_> {
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
        tx_frame: impl TryIntoCtx<bool, Error = ieee80211::scroll::Error>,
        timeout: Duration,
        phy_rate: WiFiRate,
        retries: usize,
    ) -> Result<ReceivedFrame<'_>, StaError> {
        // Allocate a TX buffer and serialize the frame.
        let mut tx_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite(tx_frame, 0)
            .map_err(|_| StaError::TxBufferTooSmall)?;
        for _ in 0..=retries {
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
            if res.is_err() {
                continue;
            }
            // Due to the user operation being set to authenticating, we'll only receive authentication
            // frames.
            if let Ok(frame) = with_timeout(timeout, self.rx_queue.receive()).await {
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
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
        phy_rate: WiFiRate,
        retries: usize,
    ) -> Result<(), StaError> {
        let auth_frame = AuthenticationFrame {
            header: ManagementFrameHeader {
                receiver_address: bss.bssid,
                bssid: bss.bssid,
                transmitter_address: mac_address,
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
            .do_bidirectional_connection_step(auth_frame, timeout, phy_rate, retries)
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
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
        phy_rate: WiFiRate,
        retries: usize,
    ) -> Result<AssociationID, StaError> {
        let assoc_request_frame = AssociationRequestFrame {
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
                    SSIDElement::new(bss.ssid.as_str()).ok_or(StaError::InvalidBss)?,
                    DEFAULT_SUPPORTED_RATES,
                    DEFAULT_XRATES
                },
                _phantom: PhantomData,
            },
        };
        // Transmit an association request and wait for the association response.
        let response = self
            .do_bidirectional_connection_step(assoc_request_frame, timeout, phy_rate, retries)
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
    fn configure_rx_filters(&self, bss: &BSS, mac_address: MACAddress) {
        // Here we set and enable the BSSID and RA filters.

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
    }
    pub async fn run(
        self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
        phy_rate: WiFiRate,
        retries: usize,
    ) -> Result<AssociationID, StaError> {
        debug!(
            "Connecting to {} on channel {} with MAC address {}.",
            bss.bssid, bss.channel, mac_address
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
        let rx_router_operation = self
            .sta_tx_rx
            .rx_router
            .begin_scoped_operation(self.router_queue, Operation::Connecting(mac_address))
            .await;

        // Configure the RX filters to the specified addresses, so that we actually receive frames
        // from the AP.
        self.configure_rx_filters(bss, mac_address);

        // Try to authenticate with the AP.
        self.do_auth(bss, timeout, mac_address, phy_rate, retries)
            .await?;

        // Try to associate with the AP.
        let aid = self
            .do_assoc(bss, timeout, mac_address, phy_rate, retries)
            .await?;

        // Because RSN isn't implemented right now, there's nothing more to do here and we've
        // connected Successfully, so we mark the
        // operations as completed.
        // NOTE: This doesn't actually do anything for the RX router, except consuming the
        // operation and therefore invoking the drop code. We mark it as completed anyway, since
        // that's more explicit and easier to read.

        bringup_operation.complete();
        rx_router_operation.complete();
        // By marking the connection operation as completed, we forget self and therefore the drop
        // code never gets executed and the filter configuration remains in place.
        self.complete();

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
