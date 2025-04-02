use core::marker::PhantomData;

use awdl_frame_parser::data_frame::AWDLDataFrame;
use defmt_or_log::{debug, error, trace};
use embassy_net_driver_channel::TxRunner;
use embassy_time::{Duration, Timer};
use ether_type::EtherType;
use foa::{
    esp_wifi_hal::{TxParameters, WiFiRate},
    LMacInterfaceControl,
};
use ieee80211::{data_frame::builder::DataFrameBuilder, mac_parser::MACAddress, scroll::Pwrite};
use llc_rs::SnapLlcFrame;
use smoltcp::wire::{
    EthernetFrame, EthernetProtocol, Icmpv6Message, Icmpv6Packet, IpProtocol, Ipv6Packet,
};

use crate::{peer::OverlapSlotState, state::CommonResources, APPLE_OUI, AWDL_BSSID, AWDL_MTU};

/// Handles transmission of MSDUs.
pub struct AwdlMsduTxRunner<'foa, 'vif> {
    pub interface_control: &'vif LMacInterfaceControl<'foa>,
    pub tx_runner: TxRunner<'vif, AWDL_MTU>,
    pub common_resources: &'vif CommonResources,
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
        Some(crate::ipv6_to_hw_address(&target_addr))
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
            && !common_resources.is_peer_cached(&destination_address)
        {
            debug!(
                "Can't transmit frame to peer {}, since it's not in peer cache.",
                destination_address
            );
            return;
        }

        let mut tx_buffer = interface_control.alloc_tx_buf().await;
        match common_resources.get_common_slot_for_peer(&destination_address) {
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
    pub async fn run_session(&mut self) -> ! {
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
