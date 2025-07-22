use crate::RX_QUEUE_DEPTH;
use foa::util::rx_router::{
    HasScanOperation, RxRouter, RxRouterEndpoint, RxRouterInput, RxRouterOperation,
    RxRouterScopedOperation,
};
use ieee80211::{
    common::{FrameType, ManagementFrameSubtype},
    mac_parser::MACAddress,
    GenericFrame,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Operations used by the STA interface.
pub enum StaRxRouterOperation {
    /// Authenticating with a network.
    Authenticating { own_address: MACAddress },
    /// Associating with a network.
    Associating { own_address: MACAddress },
    /// Performing a 4WHS with a network.
    CryptoHandshake { own_address: MACAddress },
    /// Scanning for networks.
    Scanning,
}
impl StaRxRouterOperation {
    /// Get the address we're currently using to connect to a network.
    pub const fn connecting_mac_address(self) -> Option<MACAddress> {
        match self {
            Self::Authenticating { own_address }
            | Self::Associating { own_address }
            | Self::CryptoHandshake { own_address } => Some(own_address),
            _ => None,
        }
    }
}
impl RxRouterOperation for StaRxRouterOperation {
    fn frame_relevant_for_operation(&self, generic_frame: &GenericFrame<'_>) -> bool {
        let frame_type = generic_frame.frame_control_field().frame_type();
        match self {
            Self::Scanning => matches!(
                frame_type,
                FrameType::Management(
                    ManagementFrameSubtype::Beacon | ManagementFrameSubtype::ProbeResponse
                )
            ),
            Self::Authenticating { .. } => {
                frame_type == FrameType::Management(ManagementFrameSubtype::Authentication)
            }
            Self::Associating { .. } => {
                frame_type == FrameType::Management(ManagementFrameSubtype::AssociationResponse)
            }
            Self::CryptoHandshake { .. } => generic_frame.is_eapol_key_frame(),
        }
    }
}
impl HasScanOperation for StaRxRouterOperation {
    const SCAN_OPERATION: Self = Self::Scanning;
}

pub type StaRxRouterScopedOperation<'foa, 'router, 'endpoint> =
    RxRouterScopedOperation<'foa, 'router, 'endpoint, StaRxRouterOperation>;
pub type StaRxRouterEndpoint<'foa, 'router> = RxRouterEndpoint<'foa, 'router, StaRxRouterOperation>;
pub type StaRxRouterInput<'foa, 'router> = RxRouterInput<'foa, 'router, StaRxRouterOperation>;
pub type StaRxRouter<'foa> = RxRouter<'foa, RX_QUEUE_DEPTH, StaRxRouterOperation>;
