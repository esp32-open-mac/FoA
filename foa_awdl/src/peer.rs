use core::num::NonZero;

use awdl_frame_parser::{
    action_frame::DefaultAWDLActionFrame,
    common::{AWDLDnsCompression, ReadLabelIterator},
    tlvs::{
        dns_sd::{dns_record::AWDLDnsRecord, ServiceResponseTLV},
        sync_elect::{
            channel::{Band, ChannelBandwidth, LegacyFlags, SupportChannel},
            channel_sequence::ChannelSequence,
            ChannelSequenceTLV, ElectionParametersTLV, ElectionParametersV2TLV,
            SynchronizationParametersTLV,
        },
    },
};
use defmt_or_log::debug;
use embassy_time::{Duration, Instant};
use foa::esp_wifi_hal::WiFiRate;
use ieee80211::{common::TU, mac_parser::MACAddress};

/// The state of overlapping slots between us and a peer.
pub enum OverlapSlotState {
    /// No slots overlap.
    NoOverlappingSlots,
    /// The peer isn't in cache
    PeerNotInCache,
    /// The slots currently overlap.
    CurrentlyOverlapping,
    /// The slots will overlap at the specified timestamp.
    OverlappingAt(Instant),
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// The time sync state of a peer.
pub struct SynchronizationState {
    pub channel_sequence: [u8; 16],
    /// The epoch of the TSF of the peer, relative to our system time. Unit: µs since boot.
    pub tsf_epoch_in_us: i64,
}
impl SynchronizationState {
    /// The amount of availability windows per slot.
    pub(crate) const AW_PER_SLOT: usize = 4;
    /// The amount of time units per availability window.
    pub(crate) const TU_PER_AW: usize = 16;
    /// The amount of slots per channel sequence.
    pub(crate) const SLOTS_PER_CHANNEL_SEQUENCE: usize = 16;
    /// The duration of a time unit as an [embassy_time::Duration].
    pub(crate) const TU_IN_EMBASSY_TIME: Duration = Duration::from_micros(TU.as_micros() as u64);
    /// The duration of an availability window in microseconds.
    pub(crate) const AW_DURATION_IN_US: u64 =
        Self::TU_IN_EMBASSY_TIME.as_micros() * Self::TU_PER_AW as u64;
    /// The duration of one slot, consisting of one AW and three EAWs.
    pub(crate) const SLOT_DURATION: Duration =
        Duration::from_micros((Self::AW_PER_SLOT * Self::TU_PER_AW) as u64 * TU.as_micros() as u64);
    /// The duration of an entire channel sequence.
    pub(crate) const CHANNEL_SEQUENCE_DURATION: Duration = Duration::from_micros(
        Self::SLOT_DURATION.as_micros() * Self::SLOTS_PER_CHANNEL_SEQUENCE as u64,
    );

    /// Uninitialized synchronization state.
    ///
    /// This is intended for const init.
    pub(crate) const UNINIT: SynchronizationState = SynchronizationState {
        channel_sequence: [6; 16],
        tsf_epoch_in_us: 0,
    };
    /// Create a new sync state with specified channel.
    pub fn new(channel: u8) -> Self {
        Self {
            channel_sequence: [channel; 16],
            tsf_epoch_in_us: Instant::now().as_micros() as i64,
        }
    }
    /// Adopts the other synchronization state as the master state.
    pub fn adopt_master(&mut self, master_sync_state: &Self) {
        self.tsf_epoch_in_us = master_sync_state.tsf_epoch_in_us;
    }
    /// Recover the TSF epoch from the synchronization parameters.
    fn recover_tsf_epoch(
        sync_params_tlv: &SynchronizationParametersTLV,
        tx_delta: Duration,
        rx_timestamp: Instant,
    ) -> i64 {
        // The duration of an availability window (AW).
        let aw_period_in_tu = sync_params_tlv.aw_period as u32;
        // The remaining length of the AW.
        let remaining_aw_length_in_tu = sync_params_tlv.remaining_aw_length as u32;
        // The time since the first AW (so TSF epoch) is the AW duration times the AW sequence
        // number plus the remaining length of the AW.
        let tu_since_first_aw =
            aw_period_in_tu * sync_params_tlv.aw_seq_number as u32 + remaining_aw_length_in_tu;
        // We multiply by the length of a TU and convert from core::time::Duration to micros.
        // We do use an i64 of µs here, since the TSF epoch of peers maybe in the past relative to
        // our system clock.
        let elapsed_since_tsf_epoch =
            TU.as_micros() as i64 * tu_since_first_aw as i64 - tx_delta.as_micros() as i64;
        // By subtracting the time that has elapsed since the peers TSF epoch from the RX
        // timestamp, we get the timestamp of the TSF epoch.
        rx_timestamp.as_micros() as i64 - elapsed_since_tsf_epoch
    }
    /// Derive the synchronization state of the peer, from the TLV.
    fn from_af(
        awdl_action_body: &DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) -> Option<Self> {
        let channel_sequence = awdl_action_body
            .tagged_data
            .get_first_tlv::<ChannelSequenceTLV>()
            .map(|tlv| match tlv.channel_sequence {
                ChannelSequence::Simple(chan_seq) => chan_seq,
                ChannelSequence::Legacy(chan_seq) => chan_seq.map(|(_flags, channel)| channel),
                ChannelSequence::OpClass(chan_seq) => chan_seq.map(|(channel, _opclass)| channel),
            })?;
        let sync_params_tlv = awdl_action_body
            .tagged_data
            .get_first_tlv::<SynchronizationParametersTLV>()?;
        let tsf_epoch_in_us = Self::recover_tsf_epoch(
            &sync_params_tlv,
            awdl_action_body.tx_delta().try_into().unwrap(),
            rx_timestamp,
        );
        Some(Self {
            tsf_epoch_in_us,
            channel_sequence,
        })
    }
    /// The time that has elapsed, since the TSF epoch of the peer.
    pub fn elapsed_since_tsf_epoch(&self) -> Duration {
        Duration::from_micros((Instant::now().as_micros() as i64 - self.tsf_epoch_in_us) as u64)
    }
    /// The sequence number of the current AW.
    pub fn aw_seq_number(duration_since_epoch: Duration) -> u16 {
        // We get the sequence number by dividing the time that has passed since the TSF epoch, by
        // the duration of an AW.
        (duration_since_epoch.as_micros() / Self::AW_DURATION_IN_US) as u16
    }
    /// The remaining length of the current AW in TU.
    pub fn remaining_aw_length(duration_since_epoch: Duration) -> u16 {
        // Calculating the remaining length of the AW can be done, by calculating the remainder of
        // the time that has passed since the TSF epoch and the duration of an AW in µs. Dividing
        // this by the length of a TU in µs, yields the remaining AW length in TU.
        (duration_since_epoch.as_micros() % Self::AW_DURATION_IN_US / TU.as_micros() as u64) as u16
    }
    /// The remaining slot length in TU.
    pub fn remaining_slot_length(duration_since_epoch: Duration) -> u16 {
        (duration_since_epoch.as_micros() % (Self::AW_DURATION_IN_US * Self::AW_PER_SLOT as u64)
            / TU.as_micros() as u64) as u16
    }
    /// The index of the current slot.
    pub fn current_slot_index(duration_since_epoch: Duration) -> usize {
        // Dividing the AW sequence number by the amount of AW per slot yields the amount of slots,
        // that have passed since the TSF epoch. Calculating the remainder with the amount of slots
        // per channel sequence brings this into a range of 0-15.
        Self::aw_seq_number(duration_since_epoch) as usize / Self::AW_PER_SLOT
            % Self::SLOTS_PER_CHANNEL_SEQUENCE
    }
    /// The index of the next slot.
    pub fn next_slot_index(duration_since_epoch: Duration) -> usize {
        // This is the same as curren_slot_index, but we artificially increase the slot number by
        // one.
        (Self::aw_seq_number(duration_since_epoch) as usize / Self::AW_PER_SLOT + 1)
            % Self::SLOTS_PER_CHANNEL_SEQUENCE
    }
    /*
    /// Calculate the distance from the current slot to the specified slot.
    pub fn distance_to_slot(duration_since_epoch: Duration, slot: usize) -> usize {
        let current_slot = Self::current_slot_index(duration_since_epoch);
        if current_slot > slot {
            slot - current_slot
        } else {
            16 - slot + current_slot
        }
    }
    */
    /*
    /// The current channel.
    pub fn current_channel(&self, duration_since_epoch: Duration) -> u8 {
        self.channel_sequence[Self::current_slot_index(duration_since_epoch)]
    }
    */
    /// The next channel.
    pub fn next_channel(&self, duration_since_epoch: Duration) -> u8 {
        self.channel_sequence[Self::next_slot_index(duration_since_epoch)]
    }
    /// Calculate the tiemstamp at which the specified slot starts, plus the specified offset.
    pub fn slot_start_timestamp(&self, slot: usize, offset: Duration) -> Instant {
        // We calculate the elapsed time since the TSF epoch manually here, since we don't want a
        // time delta between the calculation and the timestamp from which we subtract the final
        // duration.
        let now = Instant::now();
        let duration_since_epoch =
            Duration::from_micros((now.as_micros() as i64 - self.tsf_epoch_in_us) as u64);
        let elapsed_since_chan_seq_start = Duration::from_micros(
            duration_since_epoch.as_micros() % Self::CHANNEL_SEQUENCE_DURATION.as_micros(),
        );
        // We emulate saturating subtraction here.
        let channel_sequence_start_timestamp =
            now.checked_sub(elapsed_since_chan_seq_start).unwrap_or(now);
        let time_to_slot_from_zero = Self::SLOT_DURATION * slot as u32 + offset;
        channel_sequence_start_timestamp
            + if elapsed_since_chan_seq_start > time_to_slot_from_zero {
                time_to_slot_from_zero + Self::CHANNEL_SEQUENCE_DURATION
            } else {
                time_to_slot_from_zero
            }
    }
    /// Calculate the timestamp, at which the next mutlicast slot starts.
    pub fn multicast_slot_start_timestamp(&self) -> Instant {
        self.slot_start_timestamp(9, Duration::from_secs(0))
    }
    /// Get the timestamp, at which the channel sequences will overlap next.
    pub fn next_overlap_timestamp(&self, sync_state: &Self) -> OverlapSlotState {
        let current_slot = Self::current_slot_index(self.elapsed_since_tsf_epoch());
        if self.channel_sequence[current_slot]
            == sync_state.channel_sequence
                [Self::current_slot_index(sync_state.elapsed_since_tsf_epoch())]
        {
            return OverlapSlotState::CurrentlyOverlapping;
        }
        // This orders the slot in ascending order of distance.
        (current_slot..16)
            .chain(0..current_slot)
            .find_map(|slot| {
                (self.channel_sequence[slot] == sync_state.channel_sequence[slot])
                    .then(|| self.slot_start_timestamp(slot, Duration::from_micros(0)))
            })
            .map(OverlapSlotState::OverlappingAt)
            .unwrap_or(OverlapSlotState::NoOverlappingSlots)
    }
    /// The timetamp at which the next MIF should be transmitted.
    pub fn mif_transmition_time(&self) -> Instant {
        let duration_since_epoch = self.elapsed_since_tsf_epoch();
        let slot = if Self::remaining_slot_length(duration_since_epoch) < 32 {
            Self::next_slot_index(duration_since_epoch)
        } else {
            Self::current_slot_index(duration_since_epoch)
        };
        self.slot_start_timestamp(slot, Self::TU_IN_EMBASSY_TIME * 16)
    }
    /// Generate the [SynchronizationParametersTLV] from this synchronization state.
    pub fn generate_tlv(&self, master_address: [u8; 6]) -> SynchronizationParametersTLV {
        let duration_since_epoch = self.elapsed_since_tsf_epoch();
        let aw_seq_number = Self::aw_seq_number(duration_since_epoch);
        SynchronizationParametersTLV {
            next_channel: self.next_channel(duration_since_epoch),
            tx_counter: Self::remaining_slot_length(duration_since_epoch),
            master_channel: 6,
            aw_period: 16,
            af_period: 110,
            awdl_flags: 0x1800,
            aw_ext_length: 16,
            aw_common_length: 16,
            remaining_aw_length: Self::remaining_aw_length(duration_since_epoch),
            min_ext_count: 3,
            max_multicast_ext_count: 3,
            max_unicast_ext_count: 3,
            max_af_ext_count: 3,
            master_address: master_address.into(),
            ap_beacon_alignment_delta: 0,
            presence_mode: 4,
            aw_seq_number,
            channel_sequence: ChannelSequenceTLV {
                step_count: NonZero::new(4).unwrap(),
                channel_sequence: ChannelSequence::Legacy(self.channel_sequence.map(|channel| {
                    (
                        LegacyFlags {
                            band: Band::TwoPointFourGHz,
                            support_channel: SupportChannel::Primary,
                            channel_bandwidth: ChannelBandwidth::Unknown(2),
                        },
                        channel,
                    )
                })),
            },
            ..Default::default()
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
/// The state of the master election for a peer.
pub struct ElectionState {
    pub self_metric: u32,
    pub self_counter: Option<u32>,
    pub master_metric: u32,
    pub master_counter: Option<u32>,
    pub sync_peer: Option<[u8; 6]>,
    pub root_peer: [u8; 6],
    pub distance_to_master: u32,
}
impl ElectionState {
    /// Uninitialized election state.
    ///
    /// This is intended for const init.
    pub(crate) const UNINIT: ElectionState = ElectionState::new([0u8; 6]);

    /// Create a new election state with default parameters.
    pub const fn new(address: [u8; 6]) -> Self {
        let metric = 60;
        Self {
            self_metric: metric,
            master_metric: metric,
            self_counter: Some(0),
            master_counter: Some(0),
            distance_to_master: 0,
            sync_peer: Some(address),
            root_peer: address,
        }
    }
    /// Sets the election state, as if the peer were it's own master.
    pub fn set_master_to_self(&mut self, our_address: [u8; 6]) {
        *self = Self::new(our_address);
    }
    /// Is the specified address the master.
    pub fn is_master(&self, address: &[u8; 6]) -> bool {
        self.sync_peer.unwrap_or(self.root_peer) == *address
    }
    /// Adopt the election parameters as the master parameters.
    pub fn adopt_master(
        &mut self,
        master_address: &[u8; 6],
        master_election_state: &ElectionState,
    ) {
        self.master_metric = master_election_state.master_metric;
        self.master_counter = Some(master_election_state.master_counter.unwrap_or(0));
        self.distance_to_master = master_election_state.distance_to_master + 1;
        self.root_peer = master_election_state.root_peer;
        self.sync_peer = Some(*master_address);
    }
    fn from_af(awdl_action_body: &DefaultAWDLActionFrame) -> Option<Self> {
        awdl_action_body
            .tagged_data
            .get_first_tlv::<ElectionParametersV2TLV>()
            .map(|tlv| Self {
                self_metric: tlv.self_metric,
                self_counter: Some(tlv.self_counter),
                master_metric: tlv.master_metric,
                master_counter: Some(tlv.master_counter),
                sync_peer: Some(*tlv.sync_address),
                root_peer: *tlv.master_address,
                distance_to_master: tlv.distance_to_master,
            })
            .or_else(|| {
                awdl_action_body
                    .tagged_data
                    .get_first_tlv::<ElectionParametersTLV>()
                    .map(|tlv| Self {
                        self_metric: tlv.self_metric,
                        self_counter: None,
                        master_metric: tlv.master_metric,
                        master_counter: None,
                        root_peer: *tlv.master_address,
                        distance_to_master: tlv.distance_to_master as u32,
                        sync_peer: None,
                    })
            })
    }
    /// Check if the other election state is more eligibile to be master, than this state.
    pub fn is_more_eligible(&self, rhs: &Self) -> bool {
        if let Some(lhs_counter) = self.self_counter {
            if let Some(rhs_counter) = self.self_counter {
                lhs_counter < rhs_counter
                    || (lhs_counter == rhs_counter && self.self_metric < rhs.self_metric)
            } else {
                false
            }
        } else if rhs.self_counter.is_some() {
            true
        } else {
            self.self_metric <= rhs.self_metric
        }
    }
    pub fn generate_tlv_v1(&self) -> ElectionParametersTLV {
        ElectionParametersTLV {
            self_metric: self.self_metric,
            distance_to_master: self.distance_to_master as u8,
            master_address: self.root_peer.into(),
            master_metric: self.master_metric,
            ..Default::default()
        }
    }
    pub fn generate_tlv_v2(&self) -> Option<ElectionParametersV2TLV> {
        Some(ElectionParametersV2TLV {
            self_metric: self.self_metric,
            master_metric: self.master_metric,
            self_counter: self.self_counter?,
            master_counter: self.master_counter?,
            distance_to_master: self.distance_to_master,
            master_address: self.root_peer.into(),
            sync_address: self.sync_peer?.into(),
            ..Default::default()
        })
    }
}

/// An AWDL peer.
pub struct AwdlPeer {
    pub synchronization_state: SynchronizationState,
    pub election_state: ElectionState,
    /// The timestamp when the last frame was received from this peer.
    pub last_frame: Instant,
    pub data_rate: WiFiRate,
    pub airdrop_port: Option<u16>,
    pub airplay_port: Option<u16>,
}
impl AwdlPeer {
    /// Create a peer with the data from an action frame.
    pub(crate) fn new_with_action_frame(
        awdl_action_body: &DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) -> Option<Self> {
        Some(Self {
            synchronization_state: SynchronizationState::from_af(awdl_action_body, rx_timestamp)?,
            election_state: ElectionState::from_af(awdl_action_body)?,
            data_rate: WiFiRate::PhyRate12M,
            last_frame: Instant::now(),
            airdrop_port: None,
            airplay_port: None,
        })
    }
    /// Update the peer with the information from the provided action frame.
    ///
    /// The returned bool indicates, whether services were discovered.
    pub(crate) fn update(
        &mut self,
        transmitter: &[u8; 6],
        awdl_action_body: &DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) -> bool {
        self.last_frame = Instant::now();
        if let Some(synchronization_state) =
            SynchronizationState::from_af(awdl_action_body, rx_timestamp)
        {
            if self.synchronization_state.channel_sequence != synchronization_state.channel_sequence
            {
                debug!(
                    "Peer {} changed it's channel sequence. {} -> {}.",
                    MACAddress(*transmitter),
                    self.synchronization_state.channel_sequence,
                    synchronization_state.channel_sequence
                );
            }
            self.synchronization_state = synchronization_state;
        }
        if let Some(election_state) = ElectionState::from_af(awdl_action_body) {
            self.election_state = election_state;
        }
        let mut service_discovered = false;
        for service_response in awdl_action_body
            .tagged_data
            .get_tlvs::<ServiceResponseTLV<'_, ReadLabelIterator<'_>>>()
        {
            if let AWDLDnsRecord::SRV { port, .. } = service_response.record {
                let protocol_port_ref = match service_response.name.domain {
                    AWDLDnsCompression::AirPlayTcpLocal => &mut self.airplay_port,
                    AWDLDnsCompression::AirDropTcpLocal => &mut self.airdrop_port,
                    _ => continue,
                };
                if protocol_port_ref.is_none() {
                    *protocol_port_ref = Some(port);
                    service_discovered = true;
                }
            }
        }
        service_discovered
    }
}
