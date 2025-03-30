use core::{cell::RefCell, num::NonZero};

use crate::PEER_CACHE_SIZE;
use awdl_frame_parser::{
    action_frame::DefaultAWDLActionFrame,
    common::{AWDLDnsCompression, ReadLabelIterator},
    tlvs::{
        dns_sd::ServiceResponseTLV,
        sync_elect::{
            channel::{Band, ChannelBandwidth, LegacyFlags, SupportChannel},
            channel_sequence::ChannelSequence,
            ChannelSequenceTLV, ElectionParametersTLV, ElectionParametersV2TLV,
            SynchronizationParametersTLV,
        },
    },
};
use defmt_or_log::debug;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Instant};
use heapless::FnvIndexMap;
use ieee80211::{
    common::TU,
    mac_parser::{MACAddress, ZERO},
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SynchronizationState {
    pub channel_sequence: [u8; 16],
    /// The epoch of the TSF of the peer, relative to our system time. Unit: µs since boot.
    pub tsf_epoch_in_us: i64,
}
impl SynchronizationState {
    pub(crate) const AW_PER_SLOT: usize = 4;
    pub(crate) const TU_PER_AW: usize = 16;
    pub(crate) const SLOTS_PER_CHANNEL_SEQUENCE: usize = 16;
    pub(crate) const TU_IN_EMBASSY_TIME: Duration = Duration::from_micros(TU.as_micros() as u64);
    pub(crate) const AW_DURATION_IN_US: u64 =
        Self::TU_IN_EMBASSY_TIME.as_micros() * Self::TU_PER_AW as u64;
    pub(crate) const CHANNEL_SEQUENCE_DURATION: Duration = Duration::from_micros(
        Self::TU_IN_EMBASSY_TIME.as_micros()
            * Self::SLOTS_PER_CHANNEL_SEQUENCE as u64
            * Self::AW_PER_SLOT as u64
            * Self::TU_PER_AW as u64,
    );
    pub(crate) const SLOT_DURATION: Duration =
        Duration::from_micros((Self::AW_PER_SLOT * Self::TU_PER_AW) as u64 * TU.as_micros() as u64);

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
        let channel_sequence_start_timestamp = now - elapsed_since_chan_seq_start;
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
    pub fn generate_tlv(&self, master_address: MACAddress) -> SynchronizationParametersTLV {
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
            master_address,
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
    pub sync_peer: Option<MACAddress>,
    pub root_peer: MACAddress,
    pub distance_to_master: u32,
}
impl ElectionState {
    /// Uninitialized election state.
    ///
    /// This is intended for const init.
    pub(crate) const UNINIT: ElectionState = ElectionState::new(ZERO);

    /// Create a new election state with default parameters.
    pub const fn new(address: MACAddress) -> Self {
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
    pub fn set_master_to_self(&mut self, address: MACAddress) {
        *self = Self::new(address);
    }
    /// Is the specified address the master.
    pub fn is_master(&self, address: &MACAddress) -> bool {
        self.sync_peer.unwrap_or(self.root_peer) == *address
    }
    /// Adopt the election parameters as the master parameters.
    pub fn adopt_master(
        &mut self,
        master_address: &MACAddress,
        master_election_state: &ElectionState,
    ) {
        self.master_metric = master_election_state.master_metric;
        self.master_counter = master_election_state.master_counter;
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
                sync_peer: Some(tlv.sync_address),
                root_peer: tlv.master_address,
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
                        root_peer: tlv.master_address,
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
            master_address: self.root_peer,
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
            master_address: self.root_peer,
            sync_address: self.sync_peer?,
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
    pub is_airdrop: bool,
    pub is_airplay: bool,
}
impl AwdlPeer {
    /*
    fn extract_hostname_from_af(
        awdl_action_body: &DefaultAWDLActionFrame,
    ) -> Option<heapless::String<256>> {
        awdl_action_body
            .tagged_data
            .get_first_tlv::<DefaultArpaTLV>()
            .and_then(|tlv| tlv.arpa.labels.clone().next().as_deref().cloned())
            .map(|hostname| {
                let mut string = heapless::String::new();
                let _ = string.push_str(hostname);
                string
            })
    }
    */
    /// Create a peer with the data from an action frame.
    pub(crate) fn new_with_action_frame(
        awdl_action_body: &DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) -> Option<Self> {
        Some(Self {
            synchronization_state: SynchronizationState::from_af(awdl_action_body, rx_timestamp)?,
            election_state: ElectionState::from_af(awdl_action_body)?,
            last_frame: Instant::now(),
            is_airdrop: false,
            is_airplay: false,
        })
    }
    /// Update the peer with the information from the provided action frame.
    pub(crate) fn update(
        &mut self,
        transmitter: MACAddress,
        awdl_action_body: &DefaultAWDLActionFrame<'_>,
        rx_timestamp: Instant,
    ) {
        self.last_frame = Instant::now();
        if let Some(synchronization_state) =
            SynchronizationState::from_af(awdl_action_body, rx_timestamp)
        {
            if self.synchronization_state.channel_sequence != synchronization_state.channel_sequence
            {
                debug!(
                    "Peer {} changed it's channel sequence. {} -> {}.",
                    transmitter,
                    self.synchronization_state.channel_sequence,
                    synchronization_state.channel_sequence
                );
            }
            self.synchronization_state = synchronization_state;
        }
        if let Some(election_state) = ElectionState::from_af(awdl_action_body) {
            self.election_state = election_state;
        }
        for service_response in awdl_action_body
            .tagged_data
            .get_tlvs::<ServiceResponseTLV<'_, ReadLabelIterator<'_>>>()
        {
            match service_response.name.domain {
                AWDLDnsCompression::AirPlayTcpLocal => {
                    self.is_airplay = true;
                }
                AWDLDnsCompression::AirDropTcpLocal => {
                    self.is_airdrop = true;
                }
                _ => {}
            }
        }
        /*
                let hostname = Self::extract_hostname_from_af(awdl_action_body);
                if self.hostname.is_none() {
                    if let Some(hostname) = Self::extract_hostname_from_af(awdl_action_body) {
                        debug!(
                            "Found hostname: {} for peer: {}.",
                            hostname.as_str(),
                            transmitter
                        );
                        self.hostname = Some(hostname);
                    }
                }
        */
    }
}

/// The peer cache is full, so no peer could be added.
pub(crate) struct PeerCacheFullError;
/// The state of overlapping slots between us and a peer.
pub(crate) enum OverlapSlotState {
    /// No slots overlap.
    NoOverlappingSlots,
    /// The peer isn't in cache
    PeerNotInCache,
    /// The slots currently overlap.
    CurrentlyOverlapping,
    /// The slots will overlap at the specified timestamp.
    OverlappingAt(Instant),
}
/// A statically allocated cache of AWDL peers.
pub(crate) struct StaticAwdlPeerCache {
    peers: NoopMutex<RefCell<FnvIndexMap<MACAddress, AwdlPeer, PEER_CACHE_SIZE>>>,
}
impl StaticAwdlPeerCache {
    /// Create a new peer cache.
    pub const fn new() -> Self {
        Self {
            peers: NoopMutex::new(RefCell::new(FnvIndexMap::new())),
        }
    }
    /// Modify the peer, or attempt to add it if it's not present.
    ///
    /// If the peer cache is full a [PeerCacheFullError].
    pub fn modify_or_add_peer<I: FnOnce(&mut AwdlPeer), A: FnOnce() -> Option<AwdlPeer>>(
        &self,
        address: &MACAddress,
        inspect: I,
        add: A,
    ) -> Result<(), PeerCacheFullError> {
        self.peers.lock(|peers| {
            let mut peers = peers.borrow_mut();
            match peers.get_mut(address) {
                Some(peer) => (inspect)(peer),
                None => {
                    if peers.len() != peers.capacity() {
                        let Some(peer) = (add)() else { return Ok(()) };
                        let _ = peers.insert(*address, peer);
                    } else {
                        return Err(PeerCacheFullError);
                    }
                }
            };
            Ok(())
        })
    }
    /// Inspect the specified peer.
    ///
    /// If the peer is not present [None] will be returned.
    pub fn inspect_peers(&self, mut f: impl FnMut(&MACAddress, &AwdlPeer)) {
        self.peers.lock(|peers| {
            peers
                .borrow()
                .iter()
                .for_each(|(address, peer)| (f)(address, peer));
        })
    }
    /*
    /// Inspect a specific peer.
    pub fn inspect_peer<O>(
        &self,
        address: &MACAddress,
        f: impl FnOnce(&AwdlPeer) -> O,
    ) -> Option<O> {
        self.peers.lock(|peers| peers.borrow().get(address).map(f))
    }
    */
    /// Remove all peers, from which we haven't received a frame within the specified timeout.
    pub fn purge_stale_peers(&self, timeout: Duration) {
        self.peers.lock(|peers| {
            peers.borrow_mut().retain(|address, peer| {
                let retain_peer = peer.last_frame.elapsed() < timeout;
                if !retain_peer {
                    debug!("Removing {} from peer cache due to inactivity.", address);
                }
                retain_peer
            })
        });
    }
    /// Remove all peers from the cache.
    pub fn clear(&self) {
        self.peers.lock(|peers| peers.borrow_mut().clear());
    }
    /// Find the master from the current peer cache.
    ///
    /// If we're master None will be returned.
    pub fn find_master(
        &self,
        self_election_state: &ElectionState,
        address: &MACAddress,
    ) -> Option<(MACAddress, ElectionState, SynchronizationState)> {
        self.peers.lock(|peers| {
            let peers = peers.borrow_mut();
            let mut most_eligible_peer = (address, self_election_state);

            for current_peer in peers
                .iter()
                .map(|(address, peer)| (address, &peer.election_state))
            {
                if most_eligible_peer.1.is_more_eligible(current_peer.1) {
                    most_eligible_peer = current_peer;
                }
            }
            if most_eligible_peer.0 == address {
                None
            } else {
                Some((
                    *most_eligible_peer.0,
                    *most_eligible_peer.1,
                    peers[most_eligible_peer.0].synchronization_state,
                ))
            }
        })
    }
    /// Check if the specified peer is in cache.
    pub fn is_peer_in_cache(&self, address: &MACAddress) -> bool {
        self.peers
            .lock(|peers| peers.borrow().contains_key(address))
    }
    /// Get the nearest common slot with the peer.
    pub fn get_common_slot_for_peer(
        &self,
        address: &MACAddress,
        synchronization_state: &SynchronizationState,
    ) -> OverlapSlotState {
        self.peers.lock(|peers| {
            let peers = peers.borrow();
            if address.is_multicast() {
                OverlapSlotState::OverlappingAt(
                    synchronization_state.multicast_slot_start_timestamp(),
                )
            } else if let Some(peer) = peers.get(address) {
                peer.synchronization_state
                    .next_overlap_timestamp(synchronization_state)
            } else {
                OverlapSlotState::PeerNotInCache
            }
        })
    }
}
