use crate::RX_QUEUE_DEPTH;
use foa::rx_router::{RxRouter, RxRouterEndpoint, RxRouterInput, RxRouterOperation};
use ieee80211::{
    common::{FrameType, ManagementFrameSubtype},
    mac_parser::MACAddress,
    GenericFrame,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StaRxRouterOperation {
    Connecting(MACAddress),
    Scanning,
}
impl StaRxRouterOperation {
    pub const fn connecting_mac_address(self) -> Option<MACAddress> {
        match self {
            Self::Connecting(mac_address) => Some(mac_address),
            _ => None,
        }
    }
}
impl RxRouterOperation for StaRxRouterOperation {
    fn frame_relevant_for_operation(&self, generic_frame: GenericFrame<'_>) -> bool {
        let frame_type = generic_frame.frame_control_field().frame_type();
        if let Self::Connecting(_) = self {
            matches!(
                frame_type,
                FrameType::Management(
                    ManagementFrameSubtype::Authentication
                        | ManagementFrameSubtype::AssociationResponse
                )
            )
        } else {
            matches!(
                frame_type,
                FrameType::Management(
                    ManagementFrameSubtype::Beacon | ManagementFrameSubtype::ProbeResponse
                )
            )
        }
    }
}

pub type StaRxRouterEndpoint<'foa, 'router> =
    RxRouterEndpoint<'foa, 'router, RX_QUEUE_DEPTH, StaRxRouterOperation>;
pub type StaRxRouterInput<'foa, 'router> =
    RxRouterInput<'foa, 'router, RX_QUEUE_DEPTH, StaRxRouterOperation>;
pub type StaRxRouter<'foa> = RxRouter<'foa, RX_QUEUE_DEPTH, StaRxRouterOperation>;
