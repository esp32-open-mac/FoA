use core::{iter::Empty, marker::PhantomData, num::NonZero};

use awdl_frame_parser::{
    action_frame::{AWDLActionFrame, AWDLActionFrameSubType},
    common::{AWDLStr, AWDLVersion},
    tlvs::{
        data_path::{
            ChannelMap, DataPathChannel, DataPathExtendedFlags, DataPathFlags, DataPathStateTLV,
            UnicastOptions,
        },
        sync_elect::{channel_sequence::ChannelSequence, ChannelSequenceTLV},
        version::{AWDLDeviceClass, VersionTLV},
        RawAWDLTLV, AWDLTLV,
    },
};
use defmt_or_log::trace;
use embassy_futures::select::{select4, Either4};
use embassy_time::{Duration, Instant, Ticker, Timer};
use foa::{
    esp_wifi_hal::{TxErrorBehaviour, TxParameters, WiFiRate},
    LMacInterfaceControl,
};
use ieee80211::{
    common::TU,
    mac_parser::{MACAddress, BROADCAST, ZERO},
    mgmt_frame::{
        body::action::{RawVendorSpecificActionBody, RawVendorSpecificActionFrame},
        ManagementFrameHeader,
    },
    scroll::Pwrite,
};

use crate::{peer_cache::AwdlPeerCache, state::CommonResources, AwdlEvent, APPLE_OUI, AWDL_BSSID};

/// Runner for the AWDL interface.
pub struct AwdlManagementRunner<'foa, 'vif> {
    pub interface_control: &'vif LMacInterfaceControl<'foa>,
    pub common_resources: &'vif CommonResources,
}
impl AwdlManagementRunner<'_, '_> {
    /// Serialize an action frame with the specified address and subtype.
    ///
    /// Currently PSF and MIF are exactly the same.
    async fn serialize_and_send_action_frame(
        &self,
        address: [u8; 6],
        af_type: AWDLActionFrameSubType,
    ) {
        let mut tx_buffer = self.interface_control.alloc_tx_buf().await;
        let address = MACAddress(address);
        let awdl_frame = RawVendorSpecificActionFrame {
            header: ManagementFrameHeader {
                receiver_address: BROADCAST,
                transmitter_address: address,
                bssid: AWDL_BSSID.into(),
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
                        // We override the default HT parameters here, to prevent peers from
                        // transmitting frames to us at MCS indices above 3, since the ESP32 for
                        // some reason has issues picking those up.
                        AWDLTLV::Unknown(RawAWDLTLV {
                            tlv_type: 7,
                            slice: [0x00, 0x00, 0xce, 0x11, 0x1b, 0x0f, 0x00, 0x00].as_slice(),
                            _phantom: PhantomData,
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
        // We transmit using the 12 Mbit/s OFDM PHY, driver sequence number override and drop the
        // transmittion if it fails.
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
    /// Get the timestamp for the next MIF transmission.
    fn mif_target_tx_time(&self) -> Instant {
        self.common_resources
            .lock_sync_state(|sync_state| sync_state.mif_transmition_time())
    }
    /// Run an AWDL session.
    pub async fn run_session(&mut self, our_address: &[u8; 6]) -> ! {
        // We purge stale peers and resynchronize every second.
        let mut stale_peer_ticker = Ticker::every(Duration::from_secs(1));
        let mut psf_ticker = Ticker::every(Duration::from_micros(110 * TU.as_micros() as u64));
        loop {
            match select4(
                self.interface_control.wait_for_off_channel_request(),
                stale_peer_ticker.next(),
                psf_ticker.next(),
                Timer::at(self.mif_target_tx_time()),
            )
            .await
            {
                Either4::First(off_channel_request) => off_channel_request.reject(),
                Either4::Second(_) => {
                    // Purge the stale peers with the current timeout and set the ticker to use
                    // that timeout.
                    let timeout = self
                        .common_resources
                        .dynamic_session_parameters
                        .stale_peer_timeout
                        .get();
                    self.common_resources.lock_peer_cache(|mut peer_cache| {
                        peer_cache.purge_stale_peers(
                            |address, peer| {
                                self.common_resources
                                    .raise_user_event(AwdlEvent::PeerWentStale {
                                        address: *address,
                                        airdrop_port: peer.airdrop_port,
                                        airplay_port: peer.airplay_port,
                                    });
                            },
                            timeout,
                        )
                    });
                    stale_peer_ticker = Ticker::every(timeout);
                    self.common_resources.elect_master(our_address);
                }
                Either4::Third(_) => {
                    self.serialize_and_send_action_frame(*our_address, AWDLActionFrameSubType::PSF)
                        .await
                }
                Either4::Fourth(_) => {
                    self.serialize_and_send_action_frame(*our_address, AWDLActionFrameSubType::MIF)
                        .await;
                }
            }
        }
    }
}
