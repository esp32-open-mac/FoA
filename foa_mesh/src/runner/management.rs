use core::marker::PhantomData;

use defmt_or_log::debug;
use embassy_futures::select::{Either3, select3};
use embassy_net_driver_channel::StateRunner as NetStateRunner;

use foa::{
    LMacInterfaceControl,
    esp_wifi_hal::{TxErrorBehaviour, TxParameters, WiFiRate},
};
use rand_core::RngCore;

use crate::{rx_router::MeshRxRouterEndpoint, state::CommonResources, state::MPMFSMState};

use embassy_time::{Duration, Ticker};
use ieee80211::{
    common::{AssociationID, CapabilitiesInformation, TU},
    element_chain,
    elements::{
        self, DSSSParameterSetElement, MeshIDElement, ReadElements,
        mesh::{MeshConfigurationElement, MeshPeeringManagement, MeshPeeringProtocolIdentifier},
        tim::{TIMBitmap, TIMElement},
    },
    mac_parser::{BROADCAST, MACAddress},
    match_frames, mesh_id,
    mgmt_frame::{
        BeaconFrame, ManagementFrameHeader,
        body::{
            BeaconBody,
            action::{
                MeshPeeringConfirmBody, MeshPeeringConfirmFrame, MeshPeeringOpenBody,
                MeshPeeringOpenFrame,
            },
        },
    },
    scroll::Pwrite,
    ssid,
};

pub struct MeshManagementRunner<'foa, 'vif, Rng: RngCore + Copy> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_router_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
    pub(crate) net_state_runner: NetStateRunner<'vif>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) rng: Rng,
}

const BEACON_INTERVAL_TU: u64 = 100;

// TODO deduplicate this from foa_sta
use ieee80211::{
    elements::rates::{EncodedRate, ExtendedSupportedRatesElement, SupportedRatesElement},
    extended_supported_rates, supported_rates,
};

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

impl<Rng: RngCore + Copy> MeshManagementRunner<'_, '_, Rng> {
    fn generate_own_mesh_configuration_element() -> MeshConfigurationElement {
        MeshConfigurationElement {
            active_path_selection_protocol_identifier: ieee80211::elements::mesh::MeshConfigurationActivePathSelectionProtocolIdentifier::HWMP,
            active_path_selection_metric_identifier: ieee80211::elements::mesh::MeshConfigurationActivePathSelectionMetricIdentifier::AirtimeLinkMetric,
            congestion_control_mode_identifier: ieee80211::elements::mesh::MeshConfigurationCongestionControlModeIdentifier::NotActivated,
            syncronization_method_identifier: ieee80211::elements::mesh::MeshConfigurationSynchronizationMethodIdentifier::NeighborOffsetSynchronization,
            authentication_protocol_identifier: ieee80211::elements::mesh::MeshConfigurationAuthenticationProtocolIdentifier::NoAuthentication,
            mesh_formation_info: ieee80211::elements::mesh::MeshFormationInfo::new()
                .with_connected_to_mesh_gate(false) // TODO fill this in once we have it
                .with_num_peerings(0) // TODO fill this in
                .with_connected_to_as(false), // 'Connected to authentication system' is always false in open / SAE mesh
            mesh_capability: ieee80211::elements::mesh::MeshCapability::new()
                .with_accept_additional_mesh_peerings(true) // TODO fill this in
                .with_forwarding(true)
        }
    }

    fn does_mesh_sta_configuration_match(elements: ReadElements) -> bool {
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
        let own_config_element = Self::generate_own_mesh_configuration_element();
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
                        bitmap: None::<TIMBitmap<&[u8]>>, // TODO fill this in
                        _phantom: PhantomData
                    },
                    DEFAULT_XRATES,
                    mesh_id!(MESH_ID),
                    Self::generate_own_mesh_configuration_element()

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
                    Self::generate_own_mesh_configuration_element(),
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
                    Self::generate_own_mesh_configuration_element(),
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

    pub fn generate_new_link_id(&self) -> u16 {
        u16::try_from(self.rng.clone().next_u32() & 0xFFFF).unwrap()
    }

    pub async fn process_mesh_peering_open(
        &mut self,
        mesh_peering_open_frame: &MeshPeeringOpenFrame<'_>,
        our_address: &MACAddress,
    ) -> Option<()> {
        if Self::does_mesh_sta_configuration_match(mesh_peering_open_frame.elements) {
            // check that we still have space left for an extra association
            let addr = mesh_peering_open_frame.header.transmitter_address;
            let peer_link_id = mesh_peering_open_frame
                .body
                .elements
                .get_first_element::<MeshPeeringManagement>()?
                .parse_as_open()?
                .local_link_id;
            if let Some(cell) = self
                .common_resources
                .dynamic_session_parameters
                .find_relevant_peer(addr)
            {
                let old = cell.get();
                match old {
                    MPMFSMState::Idle => {
                        let local_link_id = self.generate_new_link_id();
                        cell.set(MPMFSMState::Active {
                            mac_addr: addr,
                            local_link_id: local_link_id,
                            peer_link_id: peer_link_id,
                            substate: crate::state::MPMFSMSubState::OpnRcvd,
                        });
                        self.send_mesh_peering_confirm(
                            our_address,
                            &addr,
                            AssociationID::new_checked(1).unwrap(), // TODO increment
                            local_link_id,
                            peer_link_id,
                        )
                        .await;
                        self.send_mesh_peering_open(our_address, &addr, local_link_id)
                            .await;
                    }
                    _ => {}
                };
            }
        } else {
            // TODO configuration does not match, send rejection
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
                    // qrp
                    let _ = match_frames! {
                        buffer.mpdu_buffer(),
                        beacon_frame = BeaconFrame => {
                            debug!("beacon frame");
                            if beacon_frame.body.ssid() == Some("") {
                                debug!("empty beacon frame");
                            }
                        }
                        mesh_peering_open_frame = MeshPeeringOpenFrame => {
                            self.process_mesh_peering_open(&mesh_peering_open_frame, our_address).await;
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
