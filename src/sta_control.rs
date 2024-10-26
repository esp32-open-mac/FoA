use core::marker::PhantomData;

use alloc::{
    collections::btree_set::BTreeSet,
    string::{String, ToString},
};
use embassy_futures::select::{select, Either};
use embassy_time::Timer;
use esp32_wifi_hal_rs::{RxFilterInterface, WiFiRate};
use ieee80211::{
    common::{
        AssociationID, CapabilitiesInformation, FCFFlags, IEEE80211AuthenticationAlgorithmNumber,
        IEEE80211StatusCode, SequenceControl,
    },
    control_frame::ControlFrame,
    element_chain,
    elements::SSIDElement,
    mac_parser::{MACAddress, BROADCAST},
    match_frames,
    mgmt_frame::{
        body::{AssociationRequestBody, AuthenticationBody, ProbeRequestBody},
        AssociationRequestFrame, AssociationResponseFrame, AuthenticationFrame, BeaconFrame,
        ManagementFrameHeader, ProbeRequestFrame, ProbeResponseFrame,
    },
};
use log::{debug, trace};

use crate::{
    lower_mac::LowerMACTransaction, timeout, wait_for_frame, ConnectionState, KnownESS, ScanConfig,
    ScanMode, StaError, StaResult, State, DEFAULT_SUPPORTED_RATES, DEFAULT_XRATES,
    RESPONSE_TIMEOUT,
};

pub struct Control<'a> {
    state: &'a State,
}
impl<'a> Control<'a> {
    pub fn new(state: &'a State) -> Self {
        Self { state }
    }
    async fn scan_on_channel(
        lmac_transaction: &LowerMACTransaction<'_>,
        callback: &mut impl FnMut(KnownESS),
        known_ess: &mut BTreeSet<String>,
    ) {
        loop {
            let received = lmac_transaction.receive().await;
            let _ = match_frames! {
                received.mpdu_buffer(),
                beacon_frame = BeaconFrame => {
                    let Some(ssid) = beacon_frame.ssid() else {
                        continue;
                    };
                    if ssid.trim().is_empty() {
                        continue;
                    }
                    if !known_ess.contains(ssid) {
                        known_ess.insert(ssid.to_string());
                        callback(
                            KnownESS::from_elements(
                                beacon_frame.header.bssid,
                                beacon_frame.elements,
                                lmac_transaction.get_wifi().get_channel()
                            )
                        );
                    }
                }
            };
        }
    }
    pub async fn scan(
        &mut self,
        scan_config: ScanConfig<'_>,
        mut callback: impl FnMut(KnownESS),
    ) -> StaResult<()> {
        let mut lmac_transaction = self.state.lower_mac.transaction_begin().await;
        debug!("Initiating ESS scan.");
        let mut known_ess = BTreeSet::new();
        let channel_set = match scan_config.scan_mode {
            ScanMode::Sweep => [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13].as_slice(),
            ScanMode::Custom(channel_set) => channel_set,
        };
        lmac_transaction
            .get_wifi_mut()
            .set_scanning_mode(RxFilterInterface::Zero, true);
        for channel in channel_set {
            lmac_transaction.set_channel(*channel);
            select(
                Self::scan_on_channel(&lmac_transaction, &mut callback, &mut known_ess),
                Timer::after(scan_config.channel_remain_time),
            )
            .await;
        }
        debug!("ESS scan complete.");
        Ok(())
    }
    async fn probe_ess(
        &self,
        lmac_transaction: &LowerMACTransaction<'_>,
        ssid: &str,
        bssid: Option<MACAddress>,
    ) -> StaResult<KnownESS> {
        let mut buf = [0x00u8; 150];
        lmac_transaction
            .transmit(
                ProbeRequestFrame {
                    header: ManagementFrameHeader {
                        receiver_address: BROADCAST,
                        transmitter_address: self.state.lower_mac.get_mac_address(),
                        bssid: BROADCAST,
                        sequence_control: SequenceControl::new().with_sequence_number(
                            self.state.lower_mac.get_and_increase_sequence_number(),
                        ),
                        ..Default::default()
                    },
                    body: ProbeRequestBody {
                        elements: element_chain! {
                            SSIDElement::new(ssid).unwrap(),
                            DEFAULT_SUPPORTED_RATES,
                            DEFAULT_XRATES
                        },
                        ..Default::default()
                    },
                },
                buf.as_mut_slice(),
                WiFiRate::PhyRate6M,
            )
            .await?;

        timeout(
            RESPONSE_TIMEOUT,
            wait_for_frame!(
                lmac_transaction,
                ProbeResponseFrame,
                probe_response => {
                    let Some(ssid_element) = probe_response.ssid() else {
                        continue;
                    };
                    if ssid_element != ssid {
                        continue;
                    }
                    if let Some(bssid) = bssid {
                        if bssid == probe_response.header.bssid {
                            continue;
                        }
                    }
                    Some(
                        KnownESS::from_elements(
                            probe_response.header.bssid,
                            probe_response.elements,
                            lmac_transaction.get_wifi().get_channel()
                        )
                )
            }),
        )
        .await
    }
    async fn find_ess_on_channel(
        &mut self,
        lmac_transaction: &LowerMACTransaction<'_>,
        ssid: &str,
        hidden: bool,
    ) -> Option<KnownESS> {
        let mut probed_channel = false;
        wait_for_frame!(lmac_transaction, BeaconFrame, beacon_frame => {
            match beacon_frame.ssid() {
                Some(received_ssid) if received_ssid == ssid => {
                    break Some(
                        KnownESS::from_elements(
                            beacon_frame.header.bssid,
                            beacon_frame.elements,
                            lmac_transaction.get_wifi().get_channel()
                        )
                    );
                }
                None if hidden && !probed_channel => {
                    probed_channel = true;
                    break self.probe_ess(lmac_transaction, ssid, Some(beacon_frame.header.bssid)).await.ok()
                }
                _ => None
            }
        })
        .await
    }
    pub async fn find_ess(
        &mut self,
        ssid: &str,
        scan_config: ScanConfig<'_>,
        hidden: bool,
    ) -> StaResult<KnownESS> {
        let mut lmac_transaction = self.state.lower_mac.transaction_begin().await;
        debug!("Searching for ESS: {ssid}");
        let channel_set = match scan_config.scan_mode {
            ScanMode::Sweep => [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13].as_slice(),
            ScanMode::Custom(channel_set) => channel_set,
        };
        lmac_transaction
            .get_wifi_mut()
            .set_scanning_mode(RxFilterInterface::Zero, true);
        for channel in channel_set {
            lmac_transaction.set_channel(*channel);
            if let Either::First(Some(known_ess)) = select(
                self.find_ess_on_channel(&lmac_transaction, ssid, hidden),
                Timer::after(scan_config.channel_remain_time),
            )
            .await
            {
                return Ok(known_ess);
            }
        }
        Err(StaError::UnableToFindESS)
    }
    async fn authenticate(
        &mut self,
        lmac_transaction: &LowerMACTransaction<'_>,
        ess: &KnownESS,
    ) -> StaResult<()> {
        let mut buf = [0x00u8; 100];
        lmac_transaction
            .transmit(
                AuthenticationFrame {
                    header: ManagementFrameHeader {
                        receiver_address: ess.bssid,
                        bssid: ess.bssid,
                        transmitter_address: self.state.lower_mac.get_mac_address(),
                        sequence_control: SequenceControl::new().with_sequence_number(
                            self.state.lower_mac.get_and_increase_sequence_number(),
                        ),
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
                buf.as_mut_slice(),
                WiFiRate::PhyRate6M,
            )
            .await?;
        debug!("Transmitted authentication frame to {}", ess.bssid);
        timeout(
            RESPONSE_TIMEOUT,
            wait_for_frame!(lmac_transaction, AuthenticationFrame, auth_frame => {
                debug!("Received authentication frame from {}, with status code: {:?}.", ess.bssid, auth_frame.status_code);
                if auth_frame.status_code == IEEE80211StatusCode::Success {
                    Some(Ok(()))
                } else {
                    Some(Err(StaError::AuthFailure(auth_frame.status_code)))
                }
            }),
        )
        .await?
    }
    async fn associate(
        &mut self,
        lmac_transaction: &LowerMACTransaction<'_>,
        ess: &KnownESS,
    ) -> StaResult<AssociationID> {
        let mut buf = [0x00u8; 300];
        lmac_transaction
            .transmit(
                AssociationRequestFrame {
                    header: ManagementFrameHeader {
                        receiver_address: ess.bssid,
                        bssid: ess.bssid,
                        transmitter_address: self.state.lower_mac.get_mac_address(),
                        sequence_control: SequenceControl::new().with_sequence_number(
                            self.state.lower_mac.get_and_increase_sequence_number(),
                        ),
                        ..Default::default()
                    },
                    body: AssociationRequestBody {
                        capabilities_info: CapabilitiesInformation::new().with_is_ess(true),
                        listen_interval: 0,
                        elements: element_chain! {
                            SSIDElement::new(ess.ssid.as_str()).unwrap(),
                            DEFAULT_SUPPORTED_RATES,
                            DEFAULT_XRATES
                        },
                        _phantom: PhantomData,
                    },
                },
                buf.as_mut_slice(),
                WiFiRate::PhyRate6M,
            )
            .await?;
        debug!("Transmitted association frame to {}", ess.bssid);

        timeout(
            RESPONSE_TIMEOUT,
            wait_for_frame!(
                lmac_transaction,
                AssociationResponseFrame,
                assoc_response => {
                    debug!("Received association response from {}, with status code: {:?}.", ess.bssid, assoc_response.status_code);
                    if assoc_response.status_code == IEEE80211StatusCode::Success {
                        Some(Ok(assoc_response.association_id))
                    } else {
                        Some(Err(StaError::AssocFailure(assoc_response.status_code)))
                    }
                }
            ),
        )
        .await?
    }
    pub async fn connect(&mut self, ess: KnownESS) -> StaResult<()> {
        let mut lmac_transaction = self.state.lower_mac.transaction_begin().await;
        lmac_transaction.disable_rollback();
        debug!("Initiating connection to {}", ess.bssid);
        let _ = lmac_transaction.get_wifi_mut().set_channel(ess.channel);
        self.authenticate(&lmac_transaction, &ess).await?;
        debug!("Successfully authenticated.");
        let assoc_id = self.associate(&lmac_transaction, &ess).await?;
        debug!("Successfully associated. Got AID: {}", assoc_id.aid());
        self.state
            .state_signal
            .signal(ConnectionState::Associated(ess.bssid));
        Ok(())
    }
}
