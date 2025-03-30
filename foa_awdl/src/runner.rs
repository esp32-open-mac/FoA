use core::{iter::Empty, marker::PhantomData, num::NonZero, ops::Deref};

use crate::{
    hw_address_to_ipv6, ipv6_to_hw_address,
    peer::{AwdlPeer, OverlapSlotState},
    state::{AwdlState, CommonResources},
    APPLE_OUI, AWDL_BSSID, AWDL_MTU,
};
use awdl_frame_parser::{
    action_frame::{AWDLActionFrame, AWDLActionFrameSubType, DefaultAWDLActionFrame},
    common::{AWDLStr, AWDLVersion},
    data_frame::AWDLDataFrame,
    tlvs::{
        data_path::{
            ampdu_parameters::AMpduParameters, ht_capabilities_info::HTCapabilitiesInfo,
            ChannelMap, DataPathChannel, DataPathExtendedFlags, DataPathFlags, DataPathStateTLV,
            HTCapabilitiesTLV, UnicastOptions,
        },
        sync_elect::{channel_sequence::ChannelSequence, ChannelSequenceTLV},
        version::{AWDLDeviceClass, VersionTLV},
        RawAWDLTLV, AWDLTLV,
    },
};
use defmt_or_log::{debug, error, trace};
use embassy_futures::select::{select, select3, select4, Either, Either4};
use embassy_net_driver_channel::{
    driver::{HardwareAddress, LinkState},
    RxRunner, StateRunner, TxRunner,
};
use embassy_sync::channel::DynamicReceiver;
use embassy_time::{Duration, Instant, Ticker, Timer, WithTimeout};
use ether_type::EtherType;
use foa::{
    esp_wifi_hal::{TxErrorBehaviour, TxParameters, WiFiRate},
    LMacInterfaceControl, ReceivedFrame,
};
use ieee80211::{
    common::TU,
    data_frame::{builder::DataFrameBuilder, DataFrame, DataFrameReadPayload},
    mac_parser::{MACAddress, BROADCAST, ZERO},
    match_frames,
    mgmt_frame::{
        body::action::{RawVendorSpecificActionBody, RawVendorSpecificActionFrame},
        ManagementFrameHeader,
    },
    scroll::{Pread, Pwrite},
};
use llc_rs::SnapLlcFrame;
use smoltcp::{
    phy::ChecksumCapabilities,
    wire::{
        EthernetAddress, EthernetFrame, EthernetProtocol, Icmpv6Message, Icmpv6Packet, Icmpv6Repr,
        IpProtocol, Ipv6Packet, NdiscNeighborFlags, NdiscRepr, RawHardwareAddress,
    },
};

/// An event related to RX handling.
enum RxEvent<'foa> {
    /// A Frame was received.
    Frame(ReceivedFrame<'foa>),
    /// A request for injecting a peer into the IPv6 neighbor cache was received.
    NeighborInjectionRequest(MACAddress),
}

/// Runner for the AWDL interface.
pub(crate) struct AwdlManagementRunner<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_queue: &'vif DynamicReceiver<'foa, ReceivedFrame<'foa>>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) rx_runner: RxRunner<'vif, AWDL_MTU>,
}
impl<'foa> AwdlManagementRunner<'foa, '_> {
    /// Check if the specified peer is our master.
    fn is_peer_master(&self, address: &MACAddress) -> bool {
        self.common_resources
            .lock_election_state(|election_state| election_state.is_master(address))
    }
    fn adopt_peer_as_master(&self, address: &MACAddress, peer: &AwdlPeer) {
        self.common_resources
            .lock_sync_state(|mut sync_state| sync_state.adopt_master(&peer.synchronization_state));
        self.common_resources
            .lock_election_state(|mut election_state| {
                election_state.adopt_master(address, &peer.election_state)
            });
    }
    fn process_action_frame(
        &mut self,
        transmitter: MACAddress,
        awdl_action_body: DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) {
        let _ = self.common_resources.peer_cache.modify_or_add_peer(
            &transmitter,
            |peer| {
                peer.update(transmitter, &awdl_action_body, rx_timestamp);
                if self.is_peer_master(&transmitter) {
                    self.adopt_peer_as_master(&transmitter, peer);
                }
            },
            || {
                AwdlPeer::new_with_action_frame(&awdl_action_body, rx_timestamp).inspect(|peer| {
                    debug!(
                        "Adding peer {} to cache. Channel Sequence: {}, TSF Epoch: {}s ago.",
                        transmitter,
                        peer.synchronization_state.channel_sequence,
                        peer.synchronization_state
                            .elapsed_since_tsf_epoch()
                            .as_secs()
                    );
                })
            },
        );
    }
    fn serialize_ethernet_frame(
        buf: &mut [u8],
        src: [u8; 6],
        dst: [u8; 6],
        ethernet_protocol: EthernetProtocol,
        payload: &[u8],
    ) -> Option<usize> {
        let mut ethernet_frame = EthernetFrame::new_unchecked(buf);
        ethernet_frame.set_src_addr(EthernetAddress(src));
        ethernet_frame.set_dst_addr(EthernetAddress(dst));
        ethernet_frame.set_ethertype(ethernet_protocol);
        let payload_slice = ethernet_frame.payload_mut().get_mut(..payload.len())?;
        payload_slice.copy_from_slice(payload);
        Some(EthernetFrame::<&[u8]>::header_len() + payload.len())
    }
    async fn process_data_frame(&mut self, data_frame: &DataFrame<'_>) {
        trace!("AWDL data frame RX.");
        let Some(destination_address) = data_frame.header.destination_address() else {
            return;
        };
        let Some(source_address) = data_frame.header.source_address() else {
            return;
        };
        let Some(DataFrameReadPayload::Single(payload)) = data_frame.payload else {
            return;
        };
        let Ok(llc_frame) = payload.pread::<SnapLlcFrame>(0) else {
            return;
        };
        if llc_frame.oui != APPLE_OUI {
            return;
        }
        let Ok(awdl_data_frame) = llc_frame.payload.pread::<AWDLDataFrame<&[u8]>>(0) else {
            return;
        };
        // 20 µs seems like a good compromise, between not throwing away a ton of packets and not
        // stalling the background runner.
        let Ok(rx_buffer) = self
            .rx_runner
            .rx_buf()
            .with_timeout(Duration::from_micros(20))
            .await
        else {
            return;
        };

        let Some(written) = Self::serialize_ethernet_frame(
            rx_buffer,
            source_address.0,
            destination_address.0,
            EthernetProtocol::from(awdl_data_frame.ether_type.into_bits()),
            awdl_data_frame.payload,
        ) else {
            return;
        };

        self.rx_runner.rx_done(written);
    }
    /// Process a received frame.
    async fn process_frame(&mut self, received: ReceivedFrame<'_>) {
        let _ = match_frames! {
            received.mpdu_buffer(),
            action_frame = RawVendorSpecificActionFrame => {
                if action_frame.body.oui != APPLE_OUI {
                    return;
                }
                let Ok(awdl_action_body) = action_frame.body.payload.pread::<DefaultAWDLActionFrame>(0) else {
                    return;
                };
                self.process_action_frame(action_frame.header.transmitter_address, awdl_action_body, received.corrected_timestamp());
            }
            data_frame = DataFrame => {
                self.process_data_frame(&data_frame).await;
            }
        };
    }
    /// Choose a master for time synchronization.
    fn elect_master(&mut self, address: &MACAddress) {
        self.common_resources
            .lock_election_state(|mut self_election_state| {
                if let Some((master_address, master_election_state, master_sync_state)) = self
                    .common_resources
                    .peer_cache
                    .find_master(&self_election_state, address)
                {
                    // If the master is new, we log the stats.
                    if self_election_state.sync_peer != Some(master_address) {
                        debug!(
                            "Adopting {} as master. Metric: {} Counter: {} Hop Count: {}",
                            master_address,
                            master_election_state.self_metric,
                            master_election_state.self_counter,
                            master_election_state.distance_to_master
                        );
                    }
                    // We then adopt the master for the sync and election parameters, which will also
                    // synchronize us.
                    self.common_resources
                        .lock_sync_state(|mut self_sync_state| {
                            self_sync_state.adopt_master(&master_sync_state)
                        });
                    self_election_state.adopt_master(&master_address, &master_election_state);
                } else if !self_election_state.is_master(address) {
                    // We weren't master before, but now are. This happens if we either win the election or
                    // the peer cache is empty.
                    debug!("We're master.");
                    self_election_state.set_master_to_self(*address);
                }
            });
    }
    async fn serialize_and_send_action_frame(
        &self,
        address: MACAddress,
        af_type: AWDLActionFrameSubType,
    ) {
        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;
        let awdl_frame = RawVendorSpecificActionFrame {
            header: ManagementFrameHeader {
                receiver_address: BROADCAST,
                transmitter_address: address,
                bssid: AWDL_BSSID,
                ..Default::default()
            },
            body: RawVendorSpecificActionBody {
                oui: APPLE_OUI,
                payload: AWDLActionFrame::<[AWDLTLV<'_, Empty<MACAddress>, Empty<AWDLStr<'_>>>; 8]> {
                    target_tx_time: core::time::Duration::from_micros(
                        self.interface_control.mac_time() as u64,
                    ),
                    phy_tx_time: core::time::Duration::from_micros(0),
                    subtype: af_type,
                    tagged_data: [
                        AWDLTLV::SynchronizationParameters(self.common_resources.sync_state.lock(
                            |sync_state| {
                                sync_state.borrow().generate_tlv(
                                    self.common_resources.lock_election_state(|election_state| {
                                        election_state.root_peer
                                    }),
                                )
                            },
                        )),
                        AWDLTLV::ElectionParameters(self.common_resources.lock_election_state(
                            |election_state| election_state.generate_tlv_v1(),
                        )),
                        AWDLTLV::ChannelSequence(ChannelSequenceTLV {
                            step_count: NonZero::new(4).unwrap(),
                            channel_sequence: ChannelSequence::OpClass(
                                self.common_resources
                                    .lock_sync_state(|sync_state| sync_state.channel_sequence)
                                    .map(|channel| (channel, 0x51)),
                            ),
                        }),
                        AWDLTLV::ElectionParametersV2(
                            self.common_resources
                                .lock_election_state(|election_state| {
                                    election_state.generate_tlv_v2()
                                })
                                .unwrap(),
                        ),
                        AWDLTLV::DataPathState(DataPathStateTLV {
                            flags: DataPathFlags {
                                dualband_support: true,
                                airplay_solo_mode_support: true,
                                umi_support: true,
                                ..Default::default()
                            },
                            infra_bssid_channel: Some((ZERO, 0)),
                            infra_address: Some(address),
                            awdl_address: Some(address),
                            country_code: Some(
                                self.common_resources
                                    .dynamic_session_parameters
                                    .country_code
                                    .get(),
                            ),
                            unicast_options: Some(UnicastOptions::default()),
                            channel_map: Some(DataPathChannel::ChannelMap(ChannelMap {
                                channel_6: true,
                                ..Default::default()
                            })),
                            extended_flags: Some(DataPathExtendedFlags::default()),
                            ..Default::default()
                        }),
                        AWDLTLV::Unknown(RawAWDLTLV {
                            tlv_type: 6,
                            slice: [0x00u8; 9].as_slice(),
                            _phantom: PhantomData,
                        }),
                        AWDLTLV::HTCapabilities(HTCapabilitiesTLV {
                            ht_capabilities_info: HTCapabilitiesInfo::from_bits(0x11ce),
                            a_mpdu_parameters: AMpduParameters::from_bits(0x1b),
                            rx_spatial_stream_count: 1,
                        }),
                        AWDLTLV::Version(VersionTLV {
                            version: AWDLVersion { major: 3, minor: 4 },
                            device_class: AWDLDeviceClass::MacOS,
                        }),
                    ],
                },
                _phantom: PhantomData,
            },
        };
        let written = tx_buffer.pwrite(awdl_frame, 0).unwrap();
        let _ = self
            .interface_control
            .transmit_with_hook(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate12M,
                    override_seq_num: true,
                    tx_error_behaviour: TxErrorBehaviour::Drop,
                    ..Default::default()
                },
                false,
                |buffer| {
                    buffer[32..36].copy_from_slice(
                        self.interface_control.mac_time().to_le_bytes().as_slice(),
                    );
                },
            )
            .await;
        trace!(
            "Transmitted {} to {}.",
            if af_type == AWDLActionFrameSubType::MIF {
                "MIF"
            } else {
                "PSF"
            },
            address
        );
    }
    fn mif_target_tx_time(&self) -> Instant {
        self.common_resources.sync_state.lock(|sync_state| {
            sync_state
                .try_borrow_mut()
                .map(|sync_state| sync_state.mif_transmition_time())
                .unwrap_or(Instant::now())
        })
    }
    async fn wait_for_rx_event(&self) -> RxEvent<'foa> {
        match select(
            self.rx_queue.receive(),
            self.common_resources.ndp_inject_signal.wait(),
        )
        .await
        {
            Either::First(frame) => RxEvent::Frame(frame),
            Either::Second(peer) => RxEvent::NeighborInjectionRequest(peer),
        }
    }
    fn inject_neighbor(&mut self, own_address: MACAddress, peer_address: MACAddress) {
        if !self
            .common_resources
            .peer_cache
            .is_peer_in_cache(&peer_address)
        {
            debug!(
                "Peer {} was requested for neighbor injection, but is not in peer cache.",
                peer_address
            );
            return;
        }
        // It's ok to not wait here, since if the neighbor solicitation, that caused this
        // request, times out, another one will be sent.
        let Some(rx_buf) = self.rx_runner.try_rx_buf() else {
            debug!("Failed to inject peer, since RX queue is full.");
            return;
        };

        let mut ethernet_frame = EthernetFrame::new_unchecked(rx_buf);
        ethernet_frame.set_dst_addr(EthernetAddress(own_address.0));
        ethernet_frame.set_src_addr(EthernetAddress(peer_address.0));
        ethernet_frame.set_ethertype(EthernetProtocol::Ipv6);
        let src_addr = hw_address_to_ipv6(peer_address);
        let dst_addr = hw_address_to_ipv6(own_address);
        let neighbor_advert = Icmpv6Repr::Ndisc(NdiscRepr::NeighborAdvert {
            flags: NdiscNeighborFlags::SOLICITED,
            target_addr: src_addr,
            lladdr: Some(RawHardwareAddress::from_bytes(peer_address.as_slice())),
        });

        let mut ipv6_packet = Ipv6Packet::new_unchecked(ethernet_frame.payload_mut());
        ipv6_packet.set_version(6);
        ipv6_packet.set_next_header(IpProtocol::Icmpv6);
        ipv6_packet.set_src_addr(src_addr);
        ipv6_packet.set_dst_addr(dst_addr);
        ipv6_packet.set_hop_limit(255);
        ipv6_packet.set_payload_len(neighbor_advert.buffer_len() as u16);
        let ipv6_len = ipv6_packet.total_len();

        let mut icmpv6_packet = Icmpv6Packet::new_unchecked(ipv6_packet.payload_mut());
        neighbor_advert.emit(
            &src_addr,
            &dst_addr,
            &mut icmpv6_packet,
            &ChecksumCapabilities::default(),
        );

        let written = EthernetFrame::<&[u8]>::header_len() + ipv6_len;
        self.rx_runner.rx_done(written);

        debug!(
            "Successfully injected peer {} into neighbor cache.",
            peer_address
        );
    }
    /// Run an AWDL session.
    async fn run_session(&mut self, address: MACAddress) -> ! {
        // We purge stale peers and resynchronize every second.
        let mut stale_peer_ticker = Ticker::every(Duration::from_secs(1));
        let mut psf_ticker = Ticker::every(Duration::from_micros(110 * TU.as_micros() as u64));
        loop {
            match select4(
                self.wait_for_rx_event(),
                stale_peer_ticker.next(),
                psf_ticker.next(),
                Timer::at(self.mif_target_tx_time()),
            )
            .await
            {
                Either4::First(rx_event) => match rx_event {
                    RxEvent::Frame(frame) => self.process_frame(frame).await,
                    RxEvent::NeighborInjectionRequest(peer_address) => {
                        self.inject_neighbor(address, peer_address)
                    }
                },
                Either4::Second(_) => {
                    // Purge the stale peers with the current timeout and set the ticker to use
                    // that timeout.
                    let timeout = self
                        .common_resources
                        .dynamic_session_parameters
                        .stale_peer_timeout
                        .get();
                    self.common_resources.peer_cache.purge_stale_peers(timeout);
                    stale_peer_ticker = Ticker::every(timeout);
                    self.elect_master(&address);
                }
                Either4::Third(_) => {
                    self.serialize_and_send_action_frame(address, AWDLActionFrameSubType::PSF)
                        .await
                }
                Either4::Fourth(_) => {
                    self.serialize_and_send_action_frame(address, AWDLActionFrameSubType::MIF)
                        .await;
                }
            }
        }
    }
}
pub(crate) struct AwdlMsduTxRunner<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) tx_runner: TxRunner<'vif, AWDL_MTU>,
    pub(crate) common_resources: &'vif CommonResources,
}
impl AwdlMsduTxRunner<'_, '_> {
    fn find_ndp_solicit(ipv6_payload: &[u8]) -> Option<MACAddress> {
        let ipv6_packet = Ipv6Packet::new_checked(ipv6_payload).ok()?;
        if ipv6_packet.next_header() != IpProtocol::Icmpv6 {
            return None;
        }
        let icmpv6_packet = Icmpv6Packet::new_checked(ipv6_packet.payload()).ok()?;
        if icmpv6_packet.msg_type() != Icmpv6Message::NeighborSolicit {
            return None;
        }
        let target_addr = icmpv6_packet.target_addr();
        Some(ipv6_to_hw_address(&target_addr))
    }
    async fn transmit_frame(
        msdu_buffer: &mut [u8],
        interface_control: &LMacInterfaceControl<'_>,
        common_resources: &CommonResources,
        data_frame_sequence_number: &mut u16,
    ) {
        let Ok(mut ethernet_frame) = EthernetFrame::new_checked(msdu_buffer) else {
            return;
        };
        if ethernet_frame.ethertype() == EthernetProtocol::Ipv6 {
            if let Some(peer_address) = Self::find_ndp_solicit(ethernet_frame.payload_mut()) {
                common_resources.ndp_inject_signal.signal(peer_address);
                return;
            }
        }
        let source_address = MACAddress::new(ethernet_frame.src_addr().0);
        let destination_address = MACAddress::new(ethernet_frame.dst_addr().0);

        if !destination_address.is_multicast()
            && !common_resources
                .peer_cache
                .is_peer_in_cache(&destination_address)
        {
            debug!(
                "Can't transmit frame to peer {}, since it's not in peer cache.",
                destination_address
            );
            return;
        }

        let mut tx_buffer = interface_control.alloc_tx_buf().await;
        match common_resources.sync_state.lock(|synchronization_state| {
            common_resources.peer_cache.get_common_slot_for_peer(
                &destination_address,
                synchronization_state.borrow().deref(),
            )
        }) {
            OverlapSlotState::OverlappingAt(timestamp) => {
                Timer::at(timestamp + Duration::from_micros(110)).await
            }
            OverlapSlotState::PeerNotInCache => {
                debug!(
                    "Can't transmit frame to peer {}, since it was dropped from the cache, while we waited for a TX buffer.",
                    destination_address
                );
                return;
            }
            OverlapSlotState::NoOverlappingSlots => {
                debug!(
                    "Can't transmit frame to peer {}, due to lack of overlapping slots.",
                    destination_address
                );
                return;
            }
            OverlapSlotState::CurrentlyOverlapping => {}
        };
        let Ok(written) = tx_buffer.pwrite(
            DataFrameBuilder::new()
                .neither_to_nor_from_ds()
                .category_data()
                .payload(SnapLlcFrame {
                    oui: APPLE_OUI,
                    ether_type: EtherType::Unknown(0x0800),
                    payload: AWDLDataFrame {
                        ether_type: EtherType::from_bits(ethernet_frame.ethertype().into()),
                        payload: ethernet_frame.payload_mut() as &_,
                        sequence_number: *data_frame_sequence_number,
                    },
                    _phantom: PhantomData,
                })
                .bssid(AWDL_BSSID)
                .source_address(source_address)
                .destination_address(destination_address)
                .build(),
            0,
        ) else {
            error!("Data frame serialization failed.");
            return;
        };
        let res = interface_control
            .transmit(
                &mut tx_buffer[..written],
                &TxParameters {
                    rate: WiFiRate::PhyRate24M,
                    ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
                },
                true,
            )
            .await;
        if let Err(err) = res {
            trace!(
                "Failed to send MSDU to {}, with TX error: {:?}.",
                ethernet_frame.dst_addr(),
                err
            );
        }
        trace!(
            "AWDL MSDU TX. Peer: {} Length: {} bytes",
            destination_address,
            ethernet_frame.payload_mut().len()
        );
    }
    async fn run_session(&mut self) -> ! {
        let mut data_frame_sequence_number = 0;
        debug!("AWDL MSDU TX runner active.");
        loop {
            let msdu_buffer = self.tx_runner.tx_buf().await;

            Self::transmit_frame(
                msdu_buffer,
                self.interface_control,
                self.common_resources,
                &mut data_frame_sequence_number,
            )
            .await;

            self.tx_runner.tx_done();
            data_frame_sequence_number += 1;
        }
    }
}
pub struct AwdlRunner<'foa, 'vif> {
    pub(crate) management_runner: AwdlManagementRunner<'foa, 'vif>,
    pub(crate) msdu_tx_runner: AwdlMsduTxRunner<'foa, 'vif>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) state_runner: StateRunner<'vif>,
}
impl AwdlRunner<'_, '_> {
    /// Run the AWDL interface background task.
    pub async fn run(&mut self) -> ! {
        let mut state = AwdlState::Inactive;
        loop {
            // We wait until we reach an active state, by looping until such a state is reached.
            // This also compensates for the user being stupid and repeatedly setting an inactive
            // state.
            let AwdlState::Active {
                mac_address,
                channel,
            } = state
            else {
                state = self.common_resources.state_signal.wait().await;
                continue;
            };

            // Setup session parameters
            self.common_resources
                .initialize_session_parameters(channel, mac_address);

            // We configure embassy_net here.
            self.state_runner
                .set_hardware_address(HardwareAddress::Ethernet(*mac_address));
            self.state_runner.set_link_state(LinkState::Up);

            let _ = select3(
                self.management_runner.run_session(mac_address),
                self.msdu_tx_runner.run_session(),
                async {
                    state = self.common_resources.state_signal.wait().await;
                },
            )
            .await;
            // We clear the peer cache here and set the link down, so the next session starts of fresh.
            self.common_resources.peer_cache.clear();
            self.state_runner.set_link_state(LinkState::Down);
        }
    }
}
