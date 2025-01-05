use core::marker::PhantomData;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{with_timeout, Duration};
use foa::{
    esp32_wifi_hal_rs::{BorrowedBuffer, RxFilterBank, TxParameters},
    lmac::{LMacInterfaceControl, LMacTransmitEndpoint},
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
use log::debug;

use crate::{
    control::BSS,
    operations::{DEFAULT_SUPPORTED_RATES, DEFAULT_XRATES},
    rx_router::{Operation, RxQueue, RxRouter},
    StaError, DEFAULT_PHY_RATE,
};

/// Connecting to an AP.
pub struct ConnectionOperation<'a, 'res> {
    pub(crate) rx_router: &'a RxRouter,
    pub(crate) rx_queue: &'a Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,
    pub(crate) router_queue: RxQueue,
    pub(crate) interface_control: &'a LMacInterfaceControl<'res>,
    pub(crate) transmit_endpoint: &'a LMacTransmitEndpoint<'res>,
}
impl<'a, 'res> ConnectionOperation<'a, 'res> {
    /// Authenticate with the BSS.
    async fn do_auth(
        &self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
    ) -> Result<(), StaError> {
        let mut tx_buffer = self.transmit_endpoint.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite_with(
                AuthenticationFrame {
                    header: ManagementFrameHeader {
                        receiver_address: bss.bssid,
                        bssid: bss.bssid,
                        transmitter_address: mac_address,
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
            .transmit_endpoint
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..self.interface_control.get_default_tx_parameters()
                },
            )
            .await;
        // Due to the user operation being set to authenticating, we'll only receive authentication
        // frames.
        let Ok(response) = with_timeout(timeout, self.rx_queue.receive()).await else {
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
    async fn do_assoc(
        &self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
    ) -> Result<AssociationID, StaError> {
        let mut tx_buffer = self.transmit_endpoint.alloc_tx_buf().await;
        let written = tx_buffer
            .pwrite_with(
                AssociationRequestFrame {
                    header: ManagementFrameHeader {
                        receiver_address: bss.bssid,
                        bssid: bss.bssid,
                        transmitter_address: mac_address,
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
            .transmit_endpoint
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..self.interface_control.get_default_tx_parameters()
                },
            )
            .await;
        let Ok(received) = with_timeout(timeout, self.rx_queue.receive()).await else {
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
    pub async fn run(
        self,
        bss: &BSS,
        timeout: Duration,
        mac_address: MACAddress,
    ) -> Result<AssociationID, StaError> {
        self.interface_control
            .set_and_lock_channel(bss.channel)
            .await
            .map_err(StaError::LMacError)?;
        self.rx_router
            .begin_operation(self.router_queue, Operation::Connecting)
            .await;

        // Just to make sure, these are set for connection.
        self.interface_control.set_filter_parameters(
            RxFilterBank::ReceiverAddress,
            *mac_address,
            None,
        );
        self.interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, true);
        self.interface_control
            .set_filter_parameters(RxFilterBank::BSSID, *bss.bssid, None);
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, true);

        self.do_auth(bss, timeout, mac_address).await?;
        let aid = self.do_assoc(bss, timeout, mac_address).await?;

        Ok(aid)
    }
}
impl Drop for ConnectionOperation<'_, '_> {
    fn drop(&mut self) {
        self.rx_router.end_operation(self.router_queue);
    }
}
