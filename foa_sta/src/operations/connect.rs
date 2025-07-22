use core::{array, marker::PhantomData, mem};

use embassy_futures::join::join;
use embassy_time::WithTimeout;
use foa::{
    esp_wifi_hal::{RxFilterBank, TxErrorBehaviour, TxParameters, WiFiRate},
    LMacInterfaceControl, ReceivedFrame,
};
use ieee80211::{
    common::{
        AssociationID, CapabilitiesInformation, DataFrameSubtype, FCFFlags,
        IEEE80211AuthenticationAlgorithmNumber, IEEE80211StatusCode, SequenceControl,
    },
    crypto::{
        derive_ptk, deserialize_eapol_data_frame,
        eapol::{EapolKeyFrame, KeyDescriptorVersion, KeyInformation},
        partition_ptk, serialize_eapol_data_frame, EapolSerdeError,
    },
    data_frame::{header::DataFrameHeader, DataFrame},
    element_chain,
    elements::{
        kde::GtkKde,
        rsn::{IEEE80211CipherSuiteSelector, RsnElement},
        SSIDElement,
    },
    mac_parser::MACAddress,
    mgmt_frame::{
        body::{AssociationRequestBody, AuthenticationBody},
        AssociationRequestFrame, AssociationResponseFrame, AuthenticationFrame,
        ManagementFrameHeader,
    },
    scroll::{self, ctx::TryIntoCtx, Pread, Pwrite},
};
use llc_rs::{EtherType, SnapLlcFrame};
use rand_core::RngCore;

use crate::{
    operations::{scan::BSS, DEFAULT_SUPPORTED_RATES, DEFAULT_XRATES},
    rsn::{
        Credentials, SecurityAssociations, TransientKeySecurityAssociation, GTK_LENGTH, PMK_LENGTH,
        PTK_LENGTH, WPA2_PSK_AKM,
    },
    rx_router::{StaRxRouterEndpoint, StaRxRouterOperation, StaRxRouterScopedOperation},
    ConnectionConfig, CryptoState, SecurityConfig, StaError, StaTxRx,
};

pub struct ConnectionParameters<'a> {
    pub phy_rate: WiFiRate,
    pub config: ConnectionConfig,
    pub own_address: MACAddress,
    pub credentials: Option<Credentials<'a>>,
}

/// Transmit an EAPOL key frame, with the specified parameters.
pub(crate) async fn send_eapol_key_frame<
    'a,
    KeyMic: AsRef<[u8]>,
    ElementContainer: TryIntoCtx<(), Error = scroll::Error>,
>(
    sta_tx_rx: &StaTxRx<'_, '_>,
    bssid: MACAddress,
    own_address: MACAddress,
    tx_buffer: &mut [u8],
    payload: EapolKeyFrame<'a, KeyMic, ElementContainer>,
    kck: Option<&[u8; 16]>,
    kek: Option<&[u8; 16]>,
) -> Result<(), StaError> {
    let data_frame = DataFrame {
        header: DataFrameHeader {
            subtype: DataFrameSubtype::Data,
            fcf_flags: FCFFlags::new().with_to_ds(true),
            address_1: bssid,
            address_2: own_address,
            address_3: bssid,
            sequence_control: SequenceControl::new(),
            ..Default::default()
        },
        payload: Some(SnapLlcFrame {
            oui: [0u8; 3],
            ether_type: EtherType::Eapol,
            payload,
            _phantom: PhantomData,
        }),
        _phantom: PhantomData,
    };
    let (buffer, temp_buffer) = tx_buffer.split_at_mut(500);
    let written = serialize_eapol_data_frame(kck, kek, data_frame, buffer, temp_buffer).unwrap();
    sta_tx_rx
        .interface_control
        .transmit(
            &mut buffer[..written],
            &TxParameters {
                override_seq_num: true,
                tx_error_behaviour: TxErrorBehaviour::RetryUntil(7),
                ..Default::default()
            },
            true,
        )
        .await
        .map_err(|_| {
            debug!("4WHS step timeout.");
            StaError::AckTimeout
        })?;
    Ok(())
}
/// Connecting to an AP.
struct ConnectionOperation<'foa, 'vif, 'params> {
    sta_tx_rx: &'params StaTxRx<'foa, 'vif>,
    connection_parameters: &'params ConnectionParameters<'params>,
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
        tx_frame: &mut [u8],
    ) -> Result<ReceivedFrame<'_>, StaError> {
        for _ in 0..=self.connection_parameters.config.handshake_retries {
            let res = self
                .sta_tx_rx
                .interface_control
                .transmit(
                    tx_frame,
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
                .with_timeout(self.connection_parameters.config.handshake_timeout)
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
        // Allocate a TX buffer and serialize the frame.
        let mut tx_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;
        let written = tx_buffer.pwrite(auth_frame, 0).unwrap();
        // Transmit an authentication frame and wait for the response.
        let response = self
            .do_bidirectional_connection_step(router_operation, &mut tx_buffer[..written])
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
        router_operation: &mut StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        bss: &BSS,
    ) -> Result<AssociationID, StaError> {
        router_operation.transition(
            StaRxRouterOperation::Associating {
                own_address: self.connection_parameters.own_address
            })
            .expect("This should not fail, since all three connecting operations have the same compatibility and there is no await point in the transition.");

        let rsn_active = bss.security_config != SecurityConfig::Open;
        let mut tx_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;
        let mut assoc_request_frame = AssociationRequestFrame {
            header: ManagementFrameHeader {
                receiver_address: bss.bssid,
                bssid: bss.bssid,
                transmitter_address: self.connection_parameters.own_address,
                sequence_control: SequenceControl::new(),
                duration: 60,
                ..Default::default()
            },
            body: AssociationRequestBody {
                capabilities_info: CapabilitiesInformation::new()
                    .with_is_ess(true)
                    .with_is_confidentiality_required(rsn_active),
                listen_interval: 0,
                elements: element_chain! {
                    SSIDElement::new(bss.ssid.as_str()).ok_or(StaError::InvalidBss)?,
                    DEFAULT_SUPPORTED_RATES,
                    DEFAULT_XRATES
                },
                _phantom: PhantomData,
            },
        }
        .into_dynamic(tx_buffer.as_mut())
        .unwrap();

        if rsn_active {
            assoc_request_frame
                .add_element(RsnElement::WPA2_PERSONAL)
                .unwrap();
        }

        let written = assoc_request_frame.finish(false).unwrap();
        // Transmit an association request and wait for the association response.
        let response = self
            .do_bidirectional_connection_step(router_operation, &mut tx_buffer[..written])
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
        debug!("Assoc frame: {:02x}", response.mpdu_buffer());
        Err(StaError::AssociationFailure(assoc_response.status_code))
    }
    /// Wait for message 1 to arrive and process it accordingly.
    async fn process_message_1(
        router_operation: &StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        key_replay_counter: &mut u64,
    ) -> [u8; 32] {
        loop {
            let mut frame = router_operation.receive().await;
            let eapol_key_frame = match deserialize_eapol_data_frame(
                None,
                None,
                frame.mpdu_buffer_mut(),
                &mut [],
                WPA2_PSK_AKM,
                false,
            ) {
                Ok(eapol_key_frame) => eapol_key_frame,
                Err(EapolSerdeError::InvalidMic) => {
                    debug!("Message 1 MIC failure");
                    continue;
                }
                Err(error) => {
                    debug!(
                        "Another error occured. Frame: {:02x} EAPOL len: {} Error: {}",
                        frame.mpdu_buffer(),
                        frame.mpdu_buffer().len() - 24 - 8,
                        defmt_or_log::Debug2Format(&error)
                    );
                    continue;
                }
            };
            let key_information = eapol_key_frame.key_information;
            if !(key_information.key_descriptor_version() == KeyDescriptorVersion::AesHmacSha1
                && key_information.is_pairwise()
                && key_information.key_ack())
            {
                debug!("Key information didn't match message 1.");
                continue;
            }
            *key_replay_counter = eapol_key_frame.key_replay_counter;
            break eapol_key_frame.key_nonce;
        }
    }
    async fn send_message_2(
        &self,
        bss: &'params BSS,
        tx_buffer: &mut [u8],
        kck: &[u8; 16],
        supplicant_nonce: &[u8; 32],
        key_replay_counter: u64,
    ) -> Result<(), StaError> {
        send_eapol_key_frame(
            self.sta_tx_rx,
            bss.bssid,
            self.connection_parameters.own_address,
            tx_buffer,
            EapolKeyFrame {
                key_information: KeyInformation::new()
                    .with_is_pairwise(true)
                    .with_key_mic(true)
                    .with_key_descriptor_version(KeyDescriptorVersion::AesHmacSha1),
                key_length: 16,
                key_replay_counter,
                key_nonce: *supplicant_nonce,
                key_mic: [0x00u8; WPA2_PSK_AKM.key_mic_len().unwrap()].as_slice(),
                key_data: element_chain! {
                    RsnElement::WPA2_PERSONAL
                },
                ..Default::default()
            },
            Some(kck),
            None,
        )
        .await
    }
    async fn process_message_3(
        router_operation: &StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        shared_buffer: &mut [u8],
        kck: &[u8; 16],
        kek: &[u8; 16],
        key_replay_counter: &mut u64,
    ) -> TransientKeySecurityAssociation<GTK_LENGTH, false> {
        loop {
            let mut frame = router_operation.receive().await;
            let eapol_key_frame = match deserialize_eapol_data_frame(
                Some(kck),
                Some(kek),
                frame.mpdu_buffer_mut(),
                shared_buffer,
                WPA2_PSK_AKM,
                false,
            ) {
                Ok(eapol_key_frame) => eapol_key_frame,
                Err(EapolSerdeError::InvalidMic) => {
                    debug!("Message 3 MIC failure");
                    continue;
                }
                Err(_) => {
                    debug!("Another error occured. Frame: {:02x}", frame.mpdu_buffer());
                    continue;
                }
            };
            let key_information = eapol_key_frame.key_information;
            if !(key_information.key_descriptor_version() == KeyDescriptorVersion::AesHmacSha1
                && key_information.is_pairwise()
                && key_information.key_ack()
                && key_information.secure()
                && key_information.install()
                && key_information.key_mic()
                && key_information.encrypted_key_data())
            {
                debug!("Key information didn't match message 3.");
                continue;
            }
            *key_replay_counter = eapol_key_frame.key_replay_counter;
            let Some(gtk_kde) = eapol_key_frame.key_data.get_first_element::<GtkKde>() else {
                debug!(
                    "No GTK KDE present. Key Data: {}",
                    eapol_key_frame.key_data.bytes
                );
                continue;
            };
            break TransientKeySecurityAssociation::new(
                gtk_kde.gtk.try_into().unwrap(),
                gtk_kde.gtk_info.key_id(),
            );
        }
    }
    async fn send_message_4(
        &self,
        bss: &'params BSS,
        tx_buffer: &mut [u8],
        kck: &[u8; 16],
        supplicant_nonce: &[u8; 32],
        key_replay_counter: u64,
    ) -> Result<(), StaError> {
        send_eapol_key_frame(
            self.sta_tx_rx,
            bss.bssid,
            self.connection_parameters.own_address,
            tx_buffer,
            EapolKeyFrame {
                key_information: KeyInformation::new()
                    .with_is_pairwise(true)
                    .with_key_mic(true)
                    .with_secure(true)
                    .with_key_descriptor_version(KeyDescriptorVersion::AesHmacSha1),
                key_length: 16,
                key_replay_counter,
                key_nonce: *supplicant_nonce,
                key_mic: [0x00u8; WPA2_PSK_AKM.key_mic_len().unwrap()].as_slice(),
                key_data: element_chain! {},
                ..Default::default()
            },
            Some(kck),
            None,
        )
        .await
    }
    async fn do_4whs(
        &self,
        pmk: [u8; PMK_LENGTH],
        router_operation: &mut StaRxRouterScopedOperation<'foa, 'vif, 'params>,
        mut rng: impl RngCore,
        bss: &'params BSS,
    ) -> Result<SecurityAssociations, StaError> {
        router_operation.transition(
            StaRxRouterOperation::CryptoHandshake{
                own_address: self.connection_parameters.own_address
            })
            .expect("This should not fail, since all three connecting operations have the same compatibility and there is no await point in the transition.");

        let mut supplicant_nonce = [0u8; 32];
        rng.fill_bytes(&mut supplicant_nonce);
        debug!(
            "Starting 4WHS. PMK: {:02x}; SNonce: {:02x}",
            pmk, supplicant_nonce
        );
        let mut key_replay_counter = 0;

        let authenticator_nonce =
            Self::process_message_1(router_operation, &mut key_replay_counter).await;
        debug!(
            "Processed 4WHS message 1. ANonce: {:02x}",
            authenticator_nonce
        );

        let mut ptk = TransientKeySecurityAssociation::new([0u8; PTK_LENGTH], 0);
        derive_ptk(
            &pmk,
            &bss.bssid,
            &self.connection_parameters.own_address,
            &authenticator_nonce,
            &supplicant_nonce,
            &mut ptk.key,
        );
        let (kck, kek, tk) = partition_ptk(
            &ptk.key,
            WPA2_PSK_AKM,
            IEEE80211CipherSuiteSelector::Ccmp128,
        )
        .unwrap();
        let (kck, kek): ([u8; 16], [u8; 16]) = (kck.try_into().unwrap(), kek.try_into().unwrap());
        debug!(
            "Derived PTK. KCK: {:02x} KEK: {:02x}, TK: {:02x}",
            kck, kek, tk
        );
        //
        // We use a TX buffer as a general use buffer here.
        let mut shared_buffer = self.sta_tx_rx.interface_control.alloc_tx_buf().await;

        self.send_message_2(
            bss,
            shared_buffer.as_mut_slice(),
            &kck,
            &supplicant_nonce,
            key_replay_counter,
        )
        .await?;
        debug!("Sent 4WHS message 2.");

        let gtk = Self::process_message_3(
            router_operation,
            shared_buffer.as_mut_slice(),
            &kck,
            &kek,
            &mut key_replay_counter,
        )
        .await;
        debug!(
            "Processed 4WHS message 3. GTK: {:02x} GTK Key ID: {}",
            gtk.key, gtk.key_id
        );

        self.send_message_4(
            bss,
            shared_buffer.as_mut_slice(),
            &kck,
            &supplicant_nonce,
            key_replay_counter,
        )
        .await?;
        debug!("Sent 4WHS message 4.");

        Ok(SecurityAssociations {
            ptksa: ptk,
            gtksa: gtk,
            akm_suite: WPA2_PSK_AKM,
            cipher_suite: IEEE80211CipherSuiteSelector::Ccmp128,
        })
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
        rng: impl RngCore,
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
        let (mut router_operation, _) = join(
            rx_router_endpoint.start_operation(StaRxRouterOperation::Authenticating {
                own_address: self.connection_parameters.own_address,
            }),
            self.sta_tx_rx
                .interface_control
                .wait_for_off_channel_completion(),
        )
        .await;
        let mut pmk_and_key_slots = None;
        if bss.security_config != SecurityConfig::Open {
            if let Some(credentials) = self.connection_parameters.credentials {
                let [gtk_key_slot, ptk_key_slot] = array::from_fn(|_| {
                    self.sta_tx_rx
                        .interface_control
                        .acquire_key_slot()
                        .ok_or(StaError::NoKeySlotsAvailable)
                });
                if credentials
                    .pmk(
                        &mut pmk_and_key_slots
                            .insert(([0u8; PMK_LENGTH], gtk_key_slot?, ptk_key_slot?))
                            .0,
                        bss.ssid.as_str(),
                    )
                    .is_err()
                {
                    debug!("Invalid PSK length.");
                    return Err(StaError::InvalidPskLength);
                }
            }
        }

        // Configure the RX filters to the specified addresses, so that we actually receive frames
        // from the AP.
        self.configure_rx_filters(bss);

        // Try to authenticate with the AP.
        self.do_auth(&router_operation, bss).await?;

        // Try to associate with the AP.
        let aid = self.do_assoc(&mut router_operation, bss).await?;

        if let Some((pmk, gtk_key_slot, ptk_key_slot)) = pmk_and_key_slots {
            let crypto_keys = self.do_4whs(pmk, &mut router_operation, rng, bss).await?;
            self.sta_tx_rx.crypto_state.lock(|rc| {
                let _ = rc.borrow_mut().insert(CryptoState::new(
                    gtk_key_slot,
                    ptk_key_slot,
                    *bss.bssid,
                    crypto_keys,
                ));
            })
        }

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
    connection_parameters: &'params ConnectionParameters<'params>,
    rng: impl RngCore,
) -> Result<AssociationID, StaError> {
    ConnectionOperation {
        sta_tx_rx,
        connection_parameters,
    }
    .run(rx_router_endpoint, bss, rng)
    .await
}
