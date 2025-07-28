use core::{marker::PhantomData, num};

use defmt::println;
use defmt_or_log::debug;
use embassy_futures::select::{Either3, select3};
use embassy_net_driver_channel::StateRunner as NetStateRunner;

use foa::{
    LMacInterfaceControl,
    esp_wifi_hal::{TxErrorBehaviour, TxParameters, WiFiRate},
};
use rand_core::RngCore;

use crate::{
    peer_state::MeshPeerList,
    rx_router::MeshRxRouterEndpoint,
    state::{CommonResources, MPMFSMState, MPMFSMSubState},
};

use embassy_time::{Duration, Ticker};
use ieee80211::{
    common::{AssociationID, CapabilitiesInformation, IEEE80211Reason, TU},
    element_chain,
    elements::{
        self, DSSSParameterSetElement, MeshIDElement, ReadElements,
        mesh::{
            MeshCapability, MeshConfigurationActivePathSelectionMetricIdentifier,
            MeshConfigurationActivePathSelectionProtocolIdentifier,
            MeshConfigurationAuthenticationProtocolIdentifier,
            MeshConfigurationCongestionControlModeIdentifier, MeshConfigurationElement,
            MeshConfigurationSynchronizationMethodIdentifier, MeshFormationInfo,
            MeshPeeringManagement, MeshPeeringProtocolIdentifier,
        },
        rates::{EncodedRate, ExtendedSupportedRatesElement, SupportedRatesElement},
        tim::{TIMBitmap, TIMElement},
    },
    extended_supported_rates,
    mac_parser::{BROADCAST, MACAddress},
    match_frames, mesh_id,
    mgmt_frame::{
        BeaconFrame, ManagementFrameHeader,
        body::{
            BeaconBody,
            action::{
                MeshPeeringCloseBody, MeshPeeringCloseFrame, MeshPeeringConfirmBody,
                MeshPeeringConfirmFrame, MeshPeeringOpenBody, MeshPeeringOpenFrame,
            },
        },
    },
    scroll::Pwrite,
    ssid, supported_rates,
};

const BEACON_INTERVAL_TU: u64 = 100;

// TODO deduplicate this from foa_sta
const DEFAULT_SUPPORTED_RATES: SupportedRatesElement<[EncodedRate; 8]> = supported_rates![
                        1 B,
                        2,
                        5.5,
                        11,
                        6,
                        9,
                        12,
                        18
];

const DEFAULT_XRATES: ExtendedSupportedRatesElement<[EncodedRate; 4]> =
    extended_supported_rates![24, 36, 48, 54];

// TODO make this settable from the consumer of this library
const MESH_ID: &str = "meshtest";

pub struct MeshManagementRunner<'foa, 'vif, Rng: RngCore + Copy> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_router_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
    pub(crate) net_state_runner: NetStateRunner<'vif>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) rng: Rng,
}

impl<Rng: RngCore + Copy> MeshManagementRunner<'_, '_, Rng> {
    fn generate_own_mesh_configuration_element(&self) -> MeshConfigurationElement {
        let num_peerings = self.common_resources.lock_peer_list(|peer_list| {
            peer_list
                .iter()
                .map(|(_, state)| match state.mpm_state {
                    MPMFSMState::Estab { .. } => 1,
                    _ => 0,
                })
                .sum()
        });
        let accept_additional_peerings = self
            .common_resources
            .lock_peer_list(|peer_list| peer_list.capacity() - peer_list.len() > 0);

        MeshConfigurationElement {
            active_path_selection_protocol_identifier:
                MeshConfigurationActivePathSelectionProtocolIdentifier::HWMP,
            active_path_selection_metric_identifier:
                MeshConfigurationActivePathSelectionMetricIdentifier::AirtimeLinkMetric,
            congestion_control_mode_identifier:
                MeshConfigurationCongestionControlModeIdentifier::NotActivated,
            syncronization_method_identifier:
                MeshConfigurationSynchronizationMethodIdentifier::NeighborOffsetSynchronization,
            authentication_protocol_identifier:
                MeshConfigurationAuthenticationProtocolIdentifier::NoAuthentication,
            mesh_formation_info: MeshFormationInfo::new()
                .with_connected_to_mesh_gate(false) // TODO fill this in once we can actually connect to mesh gate
                .with_num_peerings(num_peerings)
                .with_connected_to_as(false), // 'Connected to authentication system' is always false in open / SAE mesh
            mesh_capability: MeshCapability::new()
                .with_accept_additional_mesh_peerings(accept_additional_peerings)
                .with_forwarding(true),
        }
    }

    fn does_mesh_sta_configuration_match(&self, elements: ReadElements) -> bool {
        // Check if mesh profile is equal
        if (elements
            .get_first_element::<MeshIDElement>()
            .map(|a| (a.ssid() != MESH_ID)))
        .unwrap_or(true)
        {
            return false;
        }

        let Some(peer_config_element) = elements.get_first_element::<MeshConfigurationElement>()
        else {
            debug!("no mesh configuration element");
            return false;
        };
        let own_config_element = self.generate_own_mesh_configuration_element();
        if peer_config_element.active_path_selection_metric_identifier
            != own_config_element.active_path_selection_metric_identifier
            || peer_config_element.active_path_selection_protocol_identifier
                != own_config_element.active_path_selection_protocol_identifier
            || peer_config_element.authentication_protocol_identifier
                != own_config_element.authentication_protocol_identifier
            || peer_config_element.congestion_control_mode_identifier
                != own_config_element.congestion_control_mode_identifier
            || peer_config_element.syncronization_method_identifier
                != own_config_element.syncronization_method_identifier
        {
            debug!("mesh configuration element mismatch");
            return false;
        }
        // TODO we should also check the peers EPD capability; but this is very uncommon on 2.4GHz or 5GHz

        // Check if all other fields of the mesh STA configuration matches
        if (elements
            .get_first_element::<SupportedRatesElement>()
            .map(|a| (a != DEFAULT_SUPPORTED_RATES)))
        .unwrap_or(true)
        {
            debug!("default rates mismatch");
            return false;
        }
        if elements
            .get_first_element::<ExtendedSupportedRatesElement>()
            .map(|a| (a != DEFAULT_XRATES))
            .unwrap_or(true)
        {
            debug!("extended rates mismatch");
            return false;
        }

        true
    }

    pub async fn send_beacon_frame(&mut self, address: &MACAddress) {
        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;

        // TODO this is currently only for 802.11bg, but not N
        // we could automatically adapt the contents of our beacon frames to other mesh stations
        // so that we can peer with them (only permitted if mesh id, rates and MeshConfigurationElement match)
        let beacon_frame = BeaconFrame {
            header: ManagementFrameHeader {
                receiver_address: BROADCAST,
                transmitter_address: *address,
                bssid: *address,
                ..Default::default()
            },
            body: BeaconBody {
                timestamp: 0, // TODO let the hardware fill this in automatically
                beacon_interval: BEACON_INTERVAL_TU as u16,
                capabilities_info: CapabilitiesInformation::new(),
                elements: element_chain! {
                    ssid!(""), // wildcard SSID
                    DEFAULT_SUPPORTED_RATES,
                    DSSSParameterSetElement {
                        current_channel: self.interface_control.home_channel().unwrap_or(1)
                    },
                    TIMElement {
                        dtim_count: 1, // TODO oscillate this
                        dtim_period: 2,
                        bitmap: None::<TIMBitmap<&[u8]>>, // TODO fill this in once we implement data traffic
                        _phantom: PhantomData
                    },
                    DEFAULT_XRATES,
                    mesh_id!(MESH_ID),
                    self.generate_own_mesh_configuration_element()

                },
                _phantom: PhantomData,
            },
        };

        let written = tx_buffer.pwrite(beacon_frame, 0).unwrap();
        let _ = self
            .interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate12M,
                    override_seq_num: true,
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    ..Default::default()
                },
                false,
            )
            .await;
    }

    pub async fn send_mesh_peering_confirm(
        &mut self,
        our_address: &MACAddress,
        dst_address: &MACAddress,
        aid: AssociationID,
        local_link_id: u16,
        peer_link_id: u16,
    ) {
        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;
        let mesh_peering_confirm_frame = MeshPeeringConfirmFrame {
            header: ManagementFrameHeader {
                receiver_address: *dst_address,
                transmitter_address: *our_address,
                bssid: *our_address,
                ..Default::default()
            },
            body: MeshPeeringConfirmBody {
                capabilities_info: CapabilitiesInformation::new(),
                association_id: aid,
                elements: element_chain! {
                    DEFAULT_SUPPORTED_RATES,
                    DEFAULT_XRATES,
                    mesh_id!(MESH_ID),
                    self.generate_own_mesh_configuration_element(),
                    MeshPeeringManagement::new_confirm(
                        MeshPeeringProtocolIdentifier::MeshPeeringManagementProtocol,
                        local_link_id, peer_link_id,
                        None)
                },
                _phantom: PhantomData,
            },
        };

        let written = tx_buffer.pwrite(mesh_peering_confirm_frame, 0).unwrap();
        let _ = self
            .interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate12M,
                    override_seq_num: true,
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    ..Default::default()
                },
                false,
            )
            .await;
    }

    pub async fn send_mesh_peering_open(
        &mut self,
        our_address: &MACAddress,
        dst_address: &MACAddress,
        local_link_id: u16,
    ) {
        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;
        let mesh_peering_confirm_frame = MeshPeeringOpenFrame {
            header: ManagementFrameHeader {
                receiver_address: *dst_address,
                transmitter_address: *our_address,
                bssid: *our_address,
                ..Default::default()
            },
            body: MeshPeeringOpenBody {
                capabilities_info: CapabilitiesInformation::new(),
                elements: element_chain! {
                    DEFAULT_SUPPORTED_RATES,
                    DEFAULT_XRATES,
                    mesh_id!(MESH_ID),
                    self.generate_own_mesh_configuration_element(),
                    MeshPeeringManagement::new_open(
                        MeshPeeringProtocolIdentifier::MeshPeeringManagementProtocol,
                        local_link_id,
                        None)
                },
                _phantom: PhantomData,
            },
        };

        let written = tx_buffer.pwrite(mesh_peering_confirm_frame, 0).unwrap();
        let _ = self
            .interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate12M,
                    override_seq_num: true,
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    ..Default::default()
                },
                false,
            )
            .await;
    }

    pub async fn send_mesh_peering_close(
        &mut self,
        our_address: &MACAddress,
        dst_address: &MACAddress,
        local_link_id: u16,
        peer_link_id: Option<u16>,
        reason: IEEE80211Reason,
    ) {
        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;
        let mesh_peering_close_frame = MeshPeeringCloseFrame {
            header: ManagementFrameHeader {
                receiver_address: *dst_address,
                transmitter_address: *our_address,
                bssid: *our_address,
                ..Default::default()
            },
            body: MeshPeeringCloseBody {
                elements: element_chain! {
                    mesh_id!(MESH_ID),
                    MeshPeeringManagement::new_close(
                        MeshPeeringProtocolIdentifier::MeshPeeringManagementProtocol,
                        local_link_id, peer_link_id,
                        reason,
                        None)
                },
                _phantom: PhantomData,
            },
        };

        let written = tx_buffer.pwrite(mesh_peering_close_frame, 0).unwrap();
        let _ = self
            .interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate12M,
                    override_seq_num: true,
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    ..Default::default()
                },
                false,
            )
            .await;
    }

    pub fn generate_new_link_id(&self) -> u16 {
        loop {
            let candidate = u16::try_from(self.rng.clone().next_u32() & 0xFFFF).unwrap();
            // Check if there is a link in our peer list that already has that ID
            if !self
                .common_resources
                .peer_list
                .borrow()
                .borrow()
                .iter()
                .map(|peer| match peer.1.mpm_state {
                    MPMFSMState::Idle => false,
                    MPMFSMState::Setup {
                        local_link_id,
                        peer_link_id,
                        ..
                    } => local_link_id == candidate || peer_link_id == candidate,
                    MPMFSMState::Estab {
                        local_link_id,
                        peer_link_id,
                        ..
                    } => local_link_id == candidate || peer_link_id == candidate,
                    MPMFSMState::Holding {
                        local_link_id,
                        peer_link_id,
                        ..
                    } => {
                        local_link_id == candidate
                            || peer_link_id.map(|id| id == candidate).unwrap_or(false)
                    }
                })
                .fold(false, |acc, mk| acc || mk)
            {
                // This generated ID was not yet in use
                return candidate;
            }
        }
    }

    pub async fn process_mesh_peering_open(
        &mut self,
        mesh_peering_open_frame: &MeshPeeringOpenFrame<'_>,
        our_address: &MACAddress,
    ) -> Option<()> {
        let addr = mesh_peering_open_frame.header.transmitter_address;
        let peer_link_id = mesh_peering_open_frame
            .body
            .elements
            .get_first_element::<MeshPeeringManagement>()?
            .parse_as_open()?
            .local_link_id;
        if self.does_mesh_sta_configuration_match(mesh_peering_open_frame.elements) {
            // check that we still have space left for an extra association
            let peer = {
                self.common_resources
                    .lock_peer_list(|mut peer_list| peer_list.get_or_create(&addr))
            };
            let Ok(peer) = peer else {
                self.send_mesh_peering_close(
                    our_address,
                    &addr,
                    peer_link_id,
                    None,
                    IEEE80211Reason::Unspecified, // TODO correct error code
                )
                .await;
                return None;
            };

            match peer.mpm_state {
                MPMFSMState::Idle => {
                    let local_link_id = self.generate_new_link_id();
                    let local_aid = self.common_resources.new_association_id();
                    self.common_resources.lock_peer_list(|mut peer_list| {
                        let _ = peer_list.modify_or_add_peer(
                            &addr,
                            |peer| {
                                peer.mpm_state = MPMFSMState::Setup {
                                    mac_addr: addr,
                                    local_link_id: local_link_id,
                                    peer_link_id: peer_link_id,
                                    substate: MPMFSMSubState::OpnRcvd,
                                    local_aid: local_aid,
                                    remote_aid: None,
                                }
                            },
                            || None,
                        );
                    });

                    self.send_mesh_peering_confirm(
                        our_address,
                        &addr,
                        local_aid,
                        local_link_id,
                        peer_link_id,
                    )
                    .await;
                    self.send_mesh_peering_open(our_address, &addr, local_link_id)
                        .await;
                }
                MPMFSMState::Setup {
                    mac_addr,
                    local_link_id,
                    peer_link_id,
                    substate,
                    local_aid,
                    remote_aid,
                } => match substate {
                    MPMFSMSubState::OpnSnt => {
                        self.common_resources.lock_peer_list(|mut peer_list| {
                            let _ = peer_list.modify_or_add_peer(
                                &addr,
                                |peer| {
                                    peer.mpm_state = MPMFSMState::Setup {
                                        mac_addr,
                                        local_link_id,
                                        peer_link_id,
                                        substate: MPMFSMSubState::OpnRcvd,
                                        local_aid,
                                        remote_aid,
                                    }
                                },
                                || None,
                            );
                        });
                        self.send_mesh_peering_confirm(
                            our_address,
                            &addr,
                            local_aid,
                            local_link_id,
                            peer_link_id,
                        )
                        .await
                    }
                    MPMFSMSubState::CnfRcvd => {
                        self.common_resources.lock_peer_list(|mut peer_list| {
                            let _ = peer_list.modify_or_add_peer(
                                &addr,
                                |peer| {
                                    peer.mpm_state = MPMFSMState::Estab {
                                        mac_addr,
                                        local_link_id,
                                        peer_link_id,
                                        local_aid,
                                        remote_aid: remote_aid.unwrap(),
                                    }
                                },
                                || None,
                            );
                        });
                        self.send_mesh_peering_confirm(
                            our_address,
                            &addr,
                            local_aid,
                            local_link_id,
                            peer_link_id,
                        )
                        .await;
                    }
                    MPMFSMSubState::OpnRcvd => {
                        self.send_mesh_peering_confirm(
                            our_address,
                            &addr,
                            local_aid,
                            local_link_id,
                            peer_link_id,
                        )
                        .await;
                    }
                },
                MPMFSMState::Estab {
                    local_link_id,
                    peer_link_id,
                    local_aid,
                    ..
                } => {
                    self.send_mesh_peering_confirm(
                        our_address,
                        &addr,
                        local_aid,
                        local_link_id,
                        peer_link_id,
                    )
                    .await;
                }
                MPMFSMState::Holding {
                    local_link_id,
                    peer_link_id,
                    ..
                } => {
                    self.send_mesh_peering_close(
                        our_address,
                        &addr,
                        local_link_id,
                        peer_link_id,
                        IEEE80211Reason::Unspecified, // TODO correct error code
                    )
                    .await;
                }
            };
        } else {
            self.send_mesh_peering_close(
                our_address,
                &addr,
                peer_link_id,
                None,
                IEEE80211Reason::Unspecified, // TODO correct error code
            )
            .await;
        }

        None
    }

    pub async fn process_mesh_peering_confirm(
        &mut self,
        mesh_peering_confirm_frame: &MeshPeeringConfirmFrame<'_>,
        our_address: &MACAddress,
    ) -> Option<()> {
        let addr = mesh_peering_confirm_frame.header.transmitter_address;
        let mpm = mesh_peering_confirm_frame
            .body
            .elements
            .get_first_element::<MeshPeeringManagement>()?
            .parse_as_confirm()?;
        let remote_aid = mesh_peering_confirm_frame.association_id;
        if !self.does_mesh_sta_configuration_match(mesh_peering_confirm_frame.elements) {
            self.send_mesh_peering_close(
                our_address,
                &addr,
                mpm.peer_link_id.unwrap_or(0),
                Some(mpm.local_link_id),
                IEEE80211Reason::MeshInconsistentParameters,
            )
            .await;
            return None;
        }
        // mesh configuration matches
        let peer = {
            self.common_resources
                .lock_peer_list(|mut peer_list| peer_list.get_or_create(&addr))
        };
        let Ok(peer) = peer else {
            // Normally, we should be in a state where we know about the peer, so reject
            self.send_mesh_peering_close(
                our_address,
                &addr,
                mpm.peer_link_id.unwrap_or(0),
                Some(mpm.local_link_id),
                IEEE80211Reason::Unspecified, // TODO correct error code
            )
            .await;
            return None;
        };

        match peer.mpm_state {
            MPMFSMState::Idle | MPMFSMState::Holding { .. } => {
                self.send_mesh_peering_close(
                    our_address,
                    &addr,
                    mpm.peer_link_id.unwrap_or(0),
                    Some(mpm.local_link_id),
                    IEEE80211Reason::Unspecified, // TODO correct error code
                )
                .await;
            }
            MPMFSMState::Estab { .. } => {
                // Ignore
            }
            MPMFSMState::Setup {
                mac_addr,
                local_link_id: local_link_id_cached,
                substate,
                local_aid,
                ..
            } => match substate {
                MPMFSMSubState::OpnSnt => {
                    self.common_resources.lock_peer_list(|mut peer_list| {
                        let _ = peer_list.modify_or_add_peer(
                            &addr,
                            |peer| {
                                peer.mpm_state = MPMFSMState::Setup {
                                    mac_addr: mac_addr,
                                    local_link_id: mpm.peer_link_id.unwrap_or(local_link_id_cached),
                                    peer_link_id: mpm.local_link_id,
                                    substate: MPMFSMSubState::CnfRcvd,
                                    local_aid: local_aid,
                                    remote_aid: Some(remote_aid),
                                };
                            },
                            || None,
                        );
                    });
                }
                MPMFSMSubState::OpnRcvd => {
                    self.common_resources.lock_peer_list(|mut peer_list| {
                        let _ = peer_list.modify_or_add_peer(
                            &addr,
                            |peer| {
                                peer.mpm_state = MPMFSMState::Estab {
                                    mac_addr: mac_addr,
                                    local_link_id: mpm.peer_link_id.unwrap_or(local_link_id_cached),
                                    peer_link_id: mpm.local_link_id,
                                    local_aid: local_aid,
                                    remote_aid: remote_aid,
                                };
                            },
                            || None,
                        );
                    });
                }
                _ => {
                    // Ignored
                }
            },
        }

        None
    }

    async fn process_mesh_peering_close(
        &mut self,
        mesh_peering_close_frame: &MeshPeeringCloseFrame<'_>,
        our_address: &MACAddress,
    ) -> Option<()> {
        let addr = mesh_peering_close_frame.header.transmitter_address;
        let mpm = mesh_peering_close_frame
            .body
            .elements
            .get_first_element::<MeshPeeringManagement>()?
            .parse_as_close()?;
        if !self.does_mesh_sta_configuration_match(mesh_peering_close_frame.elements) {
            // Let's not send a close frame, to avoid getting in an infinite loop
            return None;
        }
        let peer = {
            self.common_resources
                .lock_peer_list(|peer_list| (peer_list.get(&addr).cloned()))
        };
        let Some(peer) = peer else {
            // We don't know about this peer; let's not send a close frame (to avoid getting into infinite loop)
            return None;
        };
        match peer.mpm_state {
            MPMFSMState::Holding { mac_addr, .. } => {
                // delete from peer list
                self.common_resources.lock_peer_list(|mut peer_list| {
                    peer_list.remove(&mac_addr);
                });
            }
            MPMFSMState::Setup {
                mac_addr,
                local_link_id,
                ..
            }
            | MPMFSMState::Estab {
                mac_addr,
                local_link_id,
                ..
            } => {
                self.common_resources.lock_peer_list(|mut peer_list| {
                    let _ = peer_list.modify_or_add_peer(
                        &addr,
                        |peer| {
                            peer.mpm_state = MPMFSMState::Holding {
                                mac_addr: mac_addr,
                                local_link_id: mpm.peer_link_id.unwrap_or(local_link_id),
                                peer_link_id: Some(mpm.local_link_id),
                            };
                        },
                        || None,
                    );
                });
                self.send_mesh_peering_close(
                    our_address,
                    &addr,
                    local_link_id,
                    Some(mpm.local_link_id),
                    IEEE80211Reason::Unspecified, // TODO correct error code
                )
                .await;
            }
            MPMFSMState::Idle => {
                // Should not happen normally
                return None;
            }
        }
        None
    }

    pub async fn run(&mut self, our_address: &MACAddress) -> ! {
        let mut beacon_ticker = Ticker::every(Duration::from_micros(
            BEACON_INTERVAL_TU * TU.as_micros() as u64,
        ));

        loop {
            match select3(
                self.interface_control.wait_for_off_channel_request(),
                self.rx_router_endpoint.receive(),
                beacon_ticker.next(),
            )
            .await
            {
                Either3::First(_off_channel_request) => {}
                Either3::Second(buffer) => {
                    let _ = match_frames! {
                        buffer.mpdu_buffer(),
                        _beacon_frame = BeaconFrame => {
                            // TODO process beacon frames
                        }
                        mesh_peering_open_frame = MeshPeeringOpenFrame => {
                            self.process_mesh_peering_open(&mesh_peering_open_frame, our_address).await;
                        }
                        mesh_peering_confirm_frame = MeshPeeringConfirmFrame => {
                            self.process_mesh_peering_confirm(&mesh_peering_confirm_frame, our_address).await;
                        }
                        mesh_peering_close_frame = MeshPeeringCloseFrame => {
                            self.process_mesh_peering_close(&mesh_peering_close_frame, our_address).await;
                        }
                    };
                }
                Either3::Third(_) => {
                    // Time to send a beacon frame
                    self.send_beacon_frame(our_address).await;
                }
            }
        }
    }
}
