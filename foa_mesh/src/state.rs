use core::cell::{Cell, RefCell, RefMut};

use defmt_or_log::derive_format_or_debug;
use embassy_net_driver_channel::State as NetState;
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use foa::util::rx_router::RxRouter;
use ieee80211::common::AssociationID;
use ieee80211::mac_parser::MACAddress;

use crate::NET_TX_BUFFERS;
use crate::peer_state::{MeshPeerList, StaticMeshPeerList};
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

/// States of the Mesh Peering Management (MPM) Finite State Machine
// See also Figure 14-2 'Finite state machine of the MPM protocol' in the 2020 edition of the 802.11 standard
// I recommend looking at the figure, it's a lot clearer than the text
#[derive(Clone, PartialEq, Eq, Hash, Copy)]
#[derive_format_or_debug]
pub enum MPMFSMSubState {
    // Open sent, but nothing received ye
    OpnSnt,
    // Received Confirm, but no Open yet => so also no Confirm sent yet
    CnfRcvd,
    // Received Open, but not Confirm => we also sent Confirm on receiving the Open
    OpnRcvd,
}

#[derive(Clone, PartialEq, Eq, Hash, Default, Copy)]
#[derive_format_or_debug]
pub enum MPMFSMState {
    #[default]
    Idle,
    Setup {
        mac_addr: MACAddress,
        local_link_id: u16,
        peer_link_id: u16,
        substate: MPMFSMSubState,
        local_aid: AssociationID,
        remote_aid: Option<AssociationID>,
    },
    // Received Open and Confirm, also sent Open and Confirm
    Estab {
        mac_addr: MACAddress,
        local_link_id: u16,
        peer_link_id: u16,
        local_aid: AssociationID,
        remote_aid: AssociationID,
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
            Self::Estab { mac_addr, .. } => Some(mac_addr),
            Self::Setup { mac_addr, .. } => Some(mac_addr),
            Self::Holding { mac_addr, .. } => Some(mac_addr),
        }
    }
}

/// Parameters that may change over the course of a session.
pub(crate) struct DynamicSessionParameters {
    pub(crate) is_mesh_gate: Cell<bool>,
}

impl DynamicSessionParameters {
    pub fn new() -> Self {
        Self {
            is_mesh_gate: Cell::new(false),
        }
    }
}
pub(crate) type MeshPeerListImplementation = StaticMeshPeerList;

pub struct CommonResources {
    // State signaling
    /// Indicates the current status of the interface to the runner.
    pub(crate) state_signal: Signal<NoopRawMutex, MeshState>,

    // State
    /// Dynamically changing parameters.
    pub(crate) dynamic_session_parameters: DynamicSessionParameters,
    /// Stores all currently known peers.
    pub(crate) peer_list: NoopMutex<RefCell<MeshPeerListImplementation>>,

    association_id_ctr: Cell<u16>,
}

impl CommonResources {
    pub fn new() -> Self {
        Self {
            state_signal: Signal::new(),
            dynamic_session_parameters: DynamicSessionParameters::new(),
            peer_list: NoopMutex::new(RefCell::new(MeshPeerListImplementation::UNINIT)),
            association_id_ctr: Cell::new(1),
        }
    }
    /// Initialize the parameters for a new session.
    pub fn initialize_session_parameters(&self, _channel: u8, _address: MACAddress) {
        // TODO use _channel and _address
    }
    /// Acquire mutable access to the peer list in the closure.
    pub fn lock_peer_list<O>(
        &self,
        f: impl FnOnce(RefMut<'_, MeshPeerListImplementation>) -> O,
    ) -> O {
        self.peer_list.lock(|peer_list| (f)(peer_list.borrow_mut()))
    }

    pub fn new_association_id(&self) -> AssociationID {
        let counter = self.association_id_ctr.update(|counter| {
            (counter + 1 % AssociationID::MAX_AID)
                .clamp(AssociationID::MIN_AID, AssociationID::MAX_AID)
        });

        AssociationID::new_checked(counter).unwrap()
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
