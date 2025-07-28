use awdl_frame_parser::{action_frame::DefaultAWDLActionFrame, data_frame::AWDLDataFrame};
use defmt_or_log::{debug, trace};
#[cfg(feature = "ndp-inject")]
use embassy_futures::select::{select, Either};
use embassy_net_driver_channel::RxRunner;
use embassy_sync::channel::DynamicReceiver;
use embassy_time::{Duration, Instant, WithTimeout};
use foa::ReceivedFrame;
use ieee80211::{
    data_frame::{DataFrame, DataFrameReadPayload},
    mac_parser::MACAddress,
    match_frames,
    mgmt_frame::body::action::RawVendorSpecificActionFrame,
    scroll::Pread,
};
use llc_rs::SnapLlcFrame;
use smoltcp::wire::{EthernetAddress, EthernetFrame, EthernetProtocol};

use crate::{
    peer::AwdlPeer, peer_cache::AwdlPeerCache, state::CommonResources, AwdlEvent, APPLE_OUI,
    AWDL_MTU,
};

/// Handles reception of all MPDUs.
pub struct AwdlMpduRxRunner<'foa, 'vif> {
    pub rx_queue: &'vif DynamicReceiver<'foa, ReceivedFrame<'foa>>,
    pub common_resources: &'vif CommonResources,
    pub rx_runner: RxRunner<'vif, AWDL_MTU>,
}
impl AwdlMpduRxRunner<'_, '_> {
    fn process_action_frame(
        &self,
        our_address: &[u8; 6],
        transmitter: &[u8; 6],
        awdl_action_body: DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) {
        let peer_added =
            self.common_resources
                .lock_peer_cache(|mut peer_cache| {
                    peer_cache.modify_or_add_peer(
                        transmitter,
                        |peer| {
                            let service_discovered =
                                peer.update(transmitter, &awdl_action_body, rx_timestamp);
                            if service_discovered {
                                self.common_resources.raise_user_event(
                                    AwdlEvent::DiscoveredServiceForPeer {
                                        address: *transmitter,
                                        airdrop_port: peer.airdrop_port,
                                        airplay_port: peer.airplay_port,
                                    },
                                );
                            }
                            // If the peer is our master, we resynchronize to it, so timing error remains
                            // small.
                            if self.common_resources.is_peer_master(transmitter) {
                                self.common_resources
                                    .adopt_peer_as_master(transmitter, peer);
                            }
                        },
                        || {
                            AwdlPeer::new_with_action_frame(&awdl_action_body, rx_timestamp)
                                .inspect(|peer| {
                                    debug!(
                        "Adding peer {} to cache. Channel Sequence: {}, TSF Epoch: {}s ago.",
                        MACAddress(*transmitter),
                        peer.synchronization_state.channel_sequence,
                        peer.synchronization_state
                            .elapsed_since_tsf_epoch()
                            .as_secs()
                    );
                                    self.common_resources
                                        .raise_user_event(AwdlEvent::PeerDiscovered(*transmitter));
                                })
                        },
                    )
                })
                .unwrap_or(false);
        if peer_added {
            self.common_resources.elect_master(our_address);
        }
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
    async fn process_frame(&mut self, our_address: &[u8; 6], received: ReceivedFrame<'_>) {
        let _ = match_frames! {
            received.mpdu_buffer(),
            action_frame = RawVendorSpecificActionFrame => {
                if action_frame.body.oui != APPLE_OUI {
                    return;
                }
                let Ok(awdl_action_body) = action_frame.body.payload.pread::<DefaultAWDLActionFrame>(0) else {
                    return;
                };
                self.process_action_frame(our_address, &action_frame.header.transmitter_address, awdl_action_body, received.corrected_timestamp());
            }
            data_frame = DataFrame => {
                self.process_data_frame(&data_frame).await;
            }
        };
    }
    #[cfg(feature = "ndp-inject")]
    fn inject_neighbor(&mut self, own_address: &[u8; 6], peer_address: &[u8; 6]) {
        use crate::hw_address_to_ipv6;
        use smoltcp::{
            phy::ChecksumCapabilities,
            wire::{
                Icmpv6Packet, Icmpv6Repr, IpProtocol, Ipv6Packet, NdiscNeighborFlags, NdiscRepr,
                RawHardwareAddress,
            },
        };

        if !self.common_resources.is_peer_cached(peer_address) {
            debug!(
                "Peer {} was requested for neighbor injection, but is not in peer cache.",
                MACAddress(*peer_address)
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
        ethernet_frame.set_dst_addr(EthernetAddress(*own_address));
        ethernet_frame.set_src_addr(EthernetAddress(*peer_address));
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
            MACAddress(*peer_address)
        );
    }
    pub async fn run_session(&mut self, our_address: &[u8; 6]) -> ! {
        loop {
            #[cfg(feature = "ndp-inject")]
            match select(
                self.rx_queue.receive(),
                self.common_resources.ndp_inject_signal.wait(),
            )
            .await
            {
                Either::First(received) => self.process_frame(our_address, received).await,
                Either::Second(peer_address) => self.inject_neighbor(our_address, &peer_address),
            }
            #[cfg(not(feature = "ndp-inject"))]
            self.process_frame(our_address, self.rx_queue.receive().await)
                .await;
        }
    }
}
