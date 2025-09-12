use core::marker::PhantomData;

use defmt_or_log::debug;
use defmt_or_log::warn;
use embassy_futures::select::{Either4, select4};
use embassy_net_driver_channel::StateRunner as NetStateRunner;

use foa::{
    LMacInterfaceControl,
    esp_wifi_hal::{TxErrorBehaviour, TxParameters, WiFiRate},
};
use rand_core::RngCore;

use crate::{
    peer_state::{MeshPeerList, MeshPeerState},
    rx_router::MeshRxRouterEndpoint,
    state::{CommonResources, MPMFSMState, MPMFSMSubState},
};

use embassy_time::{Duration, Instant, Ticker, Timer};
use ieee80211::{
    common::{AssociationID, CapabilitiesInformation, IEEE80211Reason, TU},
    element_chain,
    elements::{
        DSSSParameterSetElement, MeshIDElement, ReadElements, SSIDElement,
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
        BeaconFrame, ManagementFrameHeader, ProbeRequestFrame, ProbeResponseFrame,
        body::{
            BeaconBody, ProbeResponseBody,
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

type MeshIdStr = heapless::String<32>;

const DOT11_MESH_RETRY_TIMEOUT_MS: u32 = 40;
const DOT11_MESH_CONFIRM_TIMEOUT_MS: u32 = 40;
const DOT11_MESH_HOLDING_TIMEOUT_MS: u32 = 40;

const DOT11_MESH_MAX_RETRIES: u32 = 2;

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

    fn does_mesh_sta_configuration_match(
        &self,
        mesh_id: &MeshIdStr,
        elements: ReadElements,
    ) -> bool {
        // Check if mesh profile is equal
        if (elements
            .get_first_element::<MeshIDElement>()
            .map(|a| (a.ssid() != mesh_id)))
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

    pub async fn send_beacon_frame(&mut self, address: &MACAddress, mesh_id: &MeshIdStr) {
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
                    MeshIDElement::new(mesh_id).unwrap(),
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
        mesh_id: &MeshIdStr,
        dst_address: &MACAddress,
        aid: AssociationID,
        local_link_id: u16,
        peer_link_id: u16,
    ) {
        debug!("mesh peering confirm tx");
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
                    MeshIDElement::new(mesh_id).unwrap(),
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
        mesh_id: &MeshIdStr,
        dst_address: &MACAddress,
        local_link_id: u16,
    ) {
        debug!("mesh peering open tx");
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
                    MeshIDElement::new(mesh_id).unwrap(),
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
        mesh_id: &MeshIdStr,
        dst_address: &MACAddress,
        local_link_id: u16,
        peer_link_id: Option<u16>,
        reason: IEEE80211Reason,
    ) {
        debug!("tx mesh peering close");
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
                    MeshIDElement::new(mesh_id).unwrap(),
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
        mesh_id: &MeshIdStr,
    ) -> Option<()> {
        debug!("mesh peering open rxd");
        let addr = mesh_peering_open_frame.header.transmitter_address;
        let packet_peer_link_id = mesh_peering_open_frame
            .body
            .elements
            .get_first_element::<MeshPeeringManagement>()?
            .parse_as_open()?
            .local_link_id;
        if self.does_mesh_sta_configuration_match(mesh_id, mesh_peering_open_frame.elements) {
            // check that we still have space left for an extra association
            let peer = {
                self.common_resources
                    .lock_peer_list(|mut peer_list| peer_list.get_or_create(&addr))
            };
            let Ok(peer) = peer else {
                self.send_mesh_peering_close(
                    our_address,
                    mesh_id,
                    &addr,
                    packet_peer_link_id,
                    None,
                    IEEE80211Reason::MeshMaxPeers,
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
                                    peer_link_id: packet_peer_link_id,
                                    substate: MPMFSMSubState::OpnRcvd,
                                    local_aid: local_aid,
                                    remote_aid: None,
                                    retry_timer_expiration: Instant::now()
                                        + Duration::from_millis(DOT11_MESH_RETRY_TIMEOUT_MS.into()),
                                    retry_counter: DOT11_MESH_MAX_RETRIES as u8,
                                    confirm_timer_expiration: Instant::MAX,
                                }
                            },
                            || None,
                        );
                    });

                    self.send_mesh_peering_confirm(
                        our_address,
                        mesh_id,
                        &addr,
                        local_aid,
                        local_link_id,
                        packet_peer_link_id,
                    )
                    .await;
                    self.send_mesh_peering_open(our_address, mesh_id, &addr, local_link_id)
                        .await;
                }
                MPMFSMState::Setup {
                    mac_addr,
                    local_link_id,
                    substate,
                    local_aid,
                    remote_aid,
                    retry_timer_expiration,
                    retry_counter,
                    confirm_timer_expiration,
                    ..
                } => match substate {
                    MPMFSMSubState::OpnSnt => {
                        self.common_resources.lock_peer_list(|mut peer_list| {
                            let _ = peer_list.modify_or_add_peer(
                                &addr,
                                |peer| {
                                    peer.mpm_state = MPMFSMState::Setup {
                                        mac_addr,
                                        local_link_id,
                                        peer_link_id: packet_peer_link_id,
                                        substate: MPMFSMSubState::OpnRcvd,
                                        local_aid,
                                        remote_aid,
                                        retry_timer_expiration,
                                        retry_counter,
                                        confirm_timer_expiration,
                                    }
                                },
                                || None,
                            );
                        });
                        self.send_mesh_peering_confirm(
                            our_address,
                            mesh_id,
                            &addr,
                            local_aid,
                            local_link_id,
                            packet_peer_link_id,
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
                                        peer_link_id: packet_peer_link_id,
                                        local_aid,
                                        remote_aid: remote_aid.unwrap(),
                                    }
                                },
                                || None,
                            );
                        });
                        self.send_mesh_peering_confirm(
                            our_address,
                            mesh_id,
                            &addr,
                            local_aid,
                            local_link_id,
                            packet_peer_link_id,
                        )
                        .await;
                    }
                    MPMFSMSubState::OpnRcvd => {
                        self.send_mesh_peering_confirm(
                            our_address,
                            mesh_id,
                            &addr,
                            local_aid,
                            local_link_id,
                            packet_peer_link_id,
                        )
                        .await;
                    }
                },
                MPMFSMState::Estab {
                    local_link_id,
                    local_aid,
                    ..
                } => {
                    self.send_mesh_peering_confirm(
                        our_address,
                        mesh_id,
                        &addr,
                        local_aid,
                        local_link_id,
                        packet_peer_link_id,
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
                        mesh_id,
                        &addr,
                        local_link_id,
                        peer_link_id,
                        IEEE80211Reason::Unspecified, // TODO what error code should we use? standard is unclear
                    )
                    .await;
                }
            };
        } else {
            self.send_mesh_peering_close(
                our_address,
                mesh_id,
                &addr,
                packet_peer_link_id,
                None,
                IEEE80211Reason::MeshInconsistentParameters,
            )
            .await;
        }

        None
    }

    pub async fn process_mesh_peering_confirm(
        &mut self,
        mesh_peering_confirm_frame: &MeshPeeringConfirmFrame<'_>,
        our_address: &MACAddress,
        mesh_id: &MeshIdStr,
    ) -> Option<()> {
        debug!("mesh peering confirm");
        let addr = mesh_peering_confirm_frame.header.transmitter_address;
        let mpm = mesh_peering_confirm_frame
            .body
            .elements
            .get_first_element::<MeshPeeringManagement>()?
            .parse_as_confirm()?;
        let remote_aid = mesh_peering_confirm_frame.association_id;
        if !self.does_mesh_sta_configuration_match(mesh_id, mesh_peering_confirm_frame.elements) {
            self.send_mesh_peering_close(
                our_address,
                mesh_id,
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
            // '''If the incoming frame is a Mesh Peering Confirm or Mesh Peering Close frame
            // and no matching mesh peering instance is found, it shall be silently discarded.'''
            return None;
        };

        match peer.mpm_state {
            MPMFSMState::Holding { .. } => {
                self.send_mesh_peering_close(
                    our_address,
                    mesh_id,
                    &addr,
                    mpm.peer_link_id.unwrap_or(0),
                    Some(mpm.local_link_id),
                    IEEE80211Reason::Unspecified, // TODO what error code should we use? standard is unclear
                )
                .await;
            }
            MPMFSMState::Idle | MPMFSMState::Estab { .. } => {
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
                                    retry_timer_expiration: Instant::MAX,
                                    retry_counter: 0,
                                    confirm_timer_expiration: Instant::now()
                                        + Duration::from_millis(
                                            DOT11_MESH_CONFIRM_TIMEOUT_MS.into(),
                                        ),
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
        mesh_id: &MeshIdStr,
    ) -> Option<()> {
        debug!("mesh peering close rxd");
        let addr = mesh_peering_close_frame.header.transmitter_address;
        let mpm = mesh_peering_close_frame
            .body
            .elements
            .get_first_element::<MeshPeeringManagement>()?
            .parse_as_close()?;
        if !self.does_mesh_sta_configuration_match(mesh_id, mesh_peering_close_frame.elements) {
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
                                holding_timer_expiration: Instant::now()
                                    + Duration::from_millis(DOT11_MESH_HOLDING_TIMEOUT_MS.into()),
                            };
                        },
                        || None,
                    );
                });
                self.send_mesh_peering_close(
                    our_address,
                    mesh_id,
                    &addr,
                    local_link_id,
                    Some(mpm.local_link_id),
                    IEEE80211Reason::MeshCloseRcvd,
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

    async fn process_beacon_frame(
        &mut self,
        beacon_frame: &BeaconFrame<'_>,
        our_address: &MACAddress,
        mesh_id: &MeshIdStr,
    ) -> Option<()> {
        let peer_addr = beacon_frame.header.transmitter_address;
        if !self.does_mesh_sta_configuration_match(mesh_id, beacon_frame.elements) {
            return None;
        }
        debug!("rxd beacon");
        let (known, free_space) = self.common_resources.lock_peer_list(|peer_list| {
            (
                peer_list.contains_key(&peer_addr),
                peer_list.capacity() - peer_list.len() > 0,
            )
        });
        if known || !free_space {
            // We already know about this peer, currently no further actions need to be taken
            return None;
        }
        let mesh_config_element = beacon_frame
            .elements
            .get_first_element::<MeshConfigurationElement>()?;
        if !mesh_config_element
            .mesh_capability
            .accept_additional_mesh_peerings()
        {
            // Does not accept extra peerings, so also not ours
            return None;
        }
        let local_link_id = self.generate_new_link_id();
        // We now know about a peer, that we can pair with, so let's try to
        // We checked earlier that the peer list should have free space, so we can ignore the result
        let _ = self.common_resources.lock_peer_list(|mut peer_list| {
            peer_list.insert(
                peer_addr,
                MeshPeerState {
                    mpm_state: MPMFSMState::Setup {
                        mac_addr: peer_addr,
                        local_link_id: local_link_id,
                        peer_link_id: 0,
                        substate: MPMFSMSubState::OpnSnt,
                        local_aid: self.common_resources.new_association_id(),
                        remote_aid: None,
                        retry_timer_expiration: Instant::now()
                            + Duration::from_millis(DOT11_MESH_RETRY_TIMEOUT_MS.into()),
                        retry_counter: DOT11_MESH_MAX_RETRIES as u8,
                        confirm_timer_expiration: Instant::MAX,
                    },
                },
            )
        });
        self.send_mesh_peering_open(our_address, mesh_id, &peer_addr, local_link_id)
            .await;

        return None;
    }

    async fn process_probe_request(
        &mut self,
        probe_request: &ProbeRequestFrame<'_>,
        our_address: &MACAddress,
        mesh_id: &MeshIdStr,
    ) -> Option<()> {
        debug!("processing probe request");
        // Only process wildcard requests or mesh probe requests
        if probe_request
            .elements
            .get_first_element::<SSIDElement>()?
            .ssid()
            != ""
        {
            return None;
        }

        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;
        let probe_response = ProbeResponseFrame {
            header: ManagementFrameHeader {
                receiver_address: probe_request.header.transmitter_address,
                transmitter_address: *our_address,
                bssid: *our_address,
                ..Default::default()
            },
            body: ProbeResponseBody {
                timestamp: 0, // TODO let the hardware fill this in automatically
                beacon_interval: BEACON_INTERVAL_TU as u16,
                capabilities_info: CapabilitiesInformation::new(),
                elements: element_chain! {
                    ssid!(""), // wildcard SSID
                    DEFAULT_SUPPORTED_RATES,
                    DSSSParameterSetElement {
                        current_channel: self.interface_control.home_channel().unwrap_or(1)
                    },
                    DEFAULT_XRATES,
                    MeshIDElement::new(mesh_id).unwrap(),
                    self.generate_own_mesh_configuration_element()
                },
                _phantom: PhantomData,
            },
        };

        let written = tx_buffer.pwrite(probe_response, 0).unwrap();
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
        None
    }

    async fn handle_timer_event(&mut self, our_address: &MACAddress, mesh_id: &MeshIdStr) {
        debug!("timer event!");
        enum TimerEvent {
            TOR1 {
                destination: MACAddress,
                local_link_id: u16,
            },
            TOR2 {
                destination: MACAddress,
                local_link_id: u16,
                peer_link_id: Option<u16>,
            },
            TOC {
                destination: MACAddress,
                local_link_id: u16,
                peer_link_id: Option<u16>,
            },
            TOH {
                destination: MACAddress,
            },
        }
        // A timer expired, so let's figure out which timers did in fact expire, and then set the next timer
        let now = Instant::now();
        let event = self.common_resources.lock_peer_list(|mut peer_list| {
            for (mac, peer) in peer_list.iter_mut() {
                debug!("peer = {}", peer);
                match peer.mpm_state {
                    MPMFSMState::Idle | MPMFSMState::Estab { .. } => (),
                    MPMFSMState::Setup {
                        ref mut retry_timer_expiration,
                        ref mut retry_counter,
                        ref mut confirm_timer_expiration,
                        local_link_id,
                        peer_link_id,
                        ..
                    } => {
                        if *retry_timer_expiration <= now {
                            debug!("retry {}", *retry_counter);
                            if *retry_counter == 0 {
                                peer.mpm_state = MPMFSMState::Holding {
                                    mac_addr: *mac,
                                    local_link_id: local_link_id,
                                    peer_link_id: Some(peer_link_id),
                                    holding_timer_expiration: now
                                        + Duration::from_millis(
                                            DOT11_MESH_HOLDING_TIMEOUT_MS.into(),
                                        ),
                                };
                                return Some(TimerEvent::TOR2 {
                                    destination: *mac,
                                    local_link_id,
                                    peer_link_id: Some(peer_link_id),
                                });
                            } else {
                                *retry_counter -= 1;
                                *retry_timer_expiration =
                                    now + Duration::from_millis(DOT11_MESH_RETRY_TIMEOUT_MS.into());
                                return Some(TimerEvent::TOR1 {
                                    destination: *mac,
                                    local_link_id,
                                });
                            }
                        }
                        if *confirm_timer_expiration <= now {
                            debug!("confirm timer expiration");
                            peer.mpm_state = MPMFSMState::Holding {
                                mac_addr: *mac,
                                local_link_id: local_link_id,
                                peer_link_id: Some(peer_link_id),
                                holding_timer_expiration: now
                                    + Duration::from_millis(DOT11_MESH_HOLDING_TIMEOUT_MS.into()),
                            };
                            return Some(TimerEvent::TOC {
                                destination: *mac,
                                local_link_id,
                                peer_link_id: Some(peer_link_id),
                            });
                        }
                    }
                    MPMFSMState::Holding {
                        ref mut holding_timer_expiration,
                        ..
                    } => {
                        if *holding_timer_expiration <= now {
                            debug!("holding timer expiration");
                            peer.mpm_state = MPMFSMState::Idle;
                            return Some(TimerEvent::TOH { destination: *mac });
                        }
                    }
                }
            }
            return None;
        });
        let Some(event) = event else {
            let timer = self
                .common_resources
                .next_peer_timer_event
                .lock(|cell| cell.borrow().clone());
            warn!(
                "timer expired, without timer event? {}ms (now={})",
                timer.as_millis(),
                Instant::now().as_millis()
            );
            return;
        };
        match event {
            TimerEvent::TOR1 {
                destination,
                local_link_id,
            } => {
                debug!("TOR1");
                self.send_mesh_peering_open(our_address, mesh_id, &destination, local_link_id)
                    .await;
            }
            TimerEvent::TOR2 {
                destination,
                local_link_id,
                peer_link_id,
            } => {
                debug!("TOR2");
                self.send_mesh_peering_close(
                    our_address,
                    mesh_id,
                    &destination,
                    local_link_id,
                    peer_link_id,
                    IEEE80211Reason::MeshMaxRetries,
                )
                .await;
            }
            TimerEvent::TOC {
                destination,
                local_link_id,
                peer_link_id,
            } => {
                debug!("TOC");
                self.send_mesh_peering_close(
                    our_address,
                    mesh_id,
                    &destination,
                    local_link_id,
                    peer_link_id,
                    IEEE80211Reason::MeshConfirmTimeout,
                )
                .await;
            }
            TimerEvent::TOH { destination } => {
                // Delete Idle from peer list to save space
                debug!("TOH");
                self.common_resources
                    .lock_peer_list(|mut peer_list| peer_list.remove(&destination));
            }
        }
    }

    pub async fn run(&mut self, our_address: &MACAddress, mesh_id: &heapless::String<32>) -> ! {
        let mut beacon_ticker = Ticker::every(Duration::from_micros(
            BEACON_INTERVAL_TU * TU.as_micros() as u64,
        ));

        loop {
            match select4(
                self.interface_control.wait_for_off_channel_request(),
                self.rx_router_endpoint.receive(),
                beacon_ticker.next(),
                Timer::at(
                    self.common_resources
                        .next_peer_timer_event
                        .lock(|cell| cell.borrow().clone()),
                ),
            )
            .await
            {
                Either4::First(_off_channel_request) => {}
                Either4::Second(buffer) => {
                    let _ = match_frames! {
                        buffer.mpdu_buffer(),
                        beacon_frame = BeaconFrame => {
                            self.process_beacon_frame(&beacon_frame, our_address, mesh_id).await;
                        }
                        mesh_peering_open_frame = MeshPeeringOpenFrame => {
                            self.process_mesh_peering_open(&mesh_peering_open_frame, our_address, mesh_id).await;
                        }
                        mesh_peering_confirm_frame = MeshPeeringConfirmFrame => {
                            self.process_mesh_peering_confirm(&mesh_peering_confirm_frame, our_address, mesh_id).await;
                        }
                        mesh_peering_close_frame = MeshPeeringCloseFrame => {
                            self.process_mesh_peering_close(&mesh_peering_close_frame, our_address, mesh_id).await;
                        }
                        probe_request = ProbeRequestFrame => {
                            self.process_probe_request(&probe_request, our_address, mesh_id).await;
                        }
                    };
                }
                Either4::Third(_) => {
                    // Time to send a beacon frame
                    self.send_beacon_frame(our_address, mesh_id).await;
                }
                Either4::Fourth(_) => {
                    self.handle_timer_event(our_address, mesh_id).await;
                }
            }
        }
    }
}
