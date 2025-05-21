use core::cell::Cell;

use defmt_or_log::derive_format_or_debug;
use embassy_net_driver_channel::State as NetState;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use foa::util::rx_router::RxRouter;
use ieee80211::mac_parser::MACAddress;

use crate::NET_TX_BUFFERS;
use crate::rx_router::MeshRxRouter;
use crate::{MTU, NET_RX_BUFFERS};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
/// State of the mesh interface pushed to the runner.
pub(crate) enum MeshState {
    Active {
        our_address: MACAddress,
        channel: u8,
        mesh_id: heapless::String<32>,
    },
    Inactive,
}

#[derive(Clone, PartialEq, Eq, Hash, Copy)]
#[derive_format_or_debug]
pub enum MPMFSMSubState {
    // Open sent, but nothing received ye
    OpnSnt,
    // Received Confirm, but no Open yet => so also no Confirm sent yet
    CnfRcvd,
    // Received Open, but not Confirm => we also sent Confirm on receiving the Open
    OpnRcvd,
    // Received Open and Confirm, also sent Open and Confirm
    Estab,
}

#[derive(Clone, PartialEq, Eq, Hash, Default, Copy)]
#[derive_format_or_debug]
pub enum MPMFSMState {
    #[default]
    Idle,
    Active {
        mac_addr: MACAddress,
        local_link_id: u16,
        peer_link_id: u16,
        substate: MPMFSMSubState,
    },
    // Closing the peering instance
    Holding {
        mac_addr: MACAddress,
        local_link_id: u16,
        peer_link_id: Option<u16>,
    },
}

impl MPMFSMState {
    pub fn get_mac_address(self) -> Option<MACAddress> {
        match self {
            Self::Idle => None,
            Self::Active { mac_addr, .. } => Some(mac_addr),
            Self::Holding { mac_addr, .. } => Some(mac_addr),
        }
    }
}

/// Parameters that may change over the course of a session.
pub(crate) struct DynamicSessionParameters {
    pub(crate) is_mesh_gate: Cell<bool>,
    pub(crate) peers: [Cell<MPMFSMState>; 5],
}

impl DynamicSessionParameters {
    pub fn new() -> Self {
        Self {
            is_mesh_gate: Cell::new(false),
            peers: Default::default(),
        }
    }

    pub fn find_relevant_peer(&self, addr: MACAddress) -> Option<&Cell<MPMFSMState>> {
        for i in 0..(self.peers.len()) {
            let elem = &self.peers[i];
            let state = elem.get();
            if state.get_mac_address() == Some(addr) {
                return Some(elem);
            }
        }
        for i in 0..(self.peers.len()) {
            let elem = &self.peers[i];
            let state = elem.get();
            if state == MPMFSMState::Idle {
                return Some(elem);
            }
        }
        None
    }
}

pub struct CommonResources {
    // State signaling
    /// Indicates the current status of the interface to the runner.
    pub(crate) state_signal: Signal<NoopRawMutex, MeshState>,

    // State
    /// Dynamically changing parameters.
    pub(crate) dynamic_session_parameters: DynamicSessionParameters,
}

impl CommonResources {
    pub fn new() -> Self {
        Self {
            state_signal: Signal::new(),
            dynamic_session_parameters: DynamicSessionParameters::new(),
        }
    }
    /// Initialize the parameters for a new session.
    pub fn initialize_session_parameters(&self, _channel: u8, _address: MACAddress) {
        // TODO use _channel and _address
    }
}

/// Resources for the Mesh interface.
pub struct MeshResources<'foa> {
    /// Resources common to all components.
    pub common_resources: CommonResources,
    /// Resources for embassy_net_driver_channel.
    pub net_state: NetState<MTU, NET_RX_BUFFERS, NET_TX_BUFFERS>,
    pub rx_router: MeshRxRouter<'foa>,
}
impl MeshResources<'_> {
    pub fn new() -> Self {
        Self {
            rx_router: RxRouter::new(),
            common_resources: CommonResources::new(),
            net_state: NetState::new(),
        }
    }
}
impl Default for MeshResources<'_> {
    fn default() -> Self {
        Self::new()
    }
}
