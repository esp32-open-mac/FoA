use crate::RX_QUEUE_DEPTH;
use foa::rx_router::{RxRouter, RxRouterEndpoint, RxRouterInput, RxRouterOperation};
use ieee80211::{
    GenericFrame,
    common::{FrameType, ManagementFrameSubtype},
};

#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MeshRxRouterOperation {
    Scanning,
}
impl RxRouterOperation for MeshRxRouterOperation {
    fn frame_relevant_for_operation(&self, generic_frame: GenericFrame<'_>) -> bool {
        let frame_type = generic_frame.frame_control_field().frame_type();
        match self {
            MeshRxRouterOperation::Scanning => matches!(
                frame_type,
                FrameType::Management(ManagementFrameSubtype::Beacon)
            ),
        }
    }
}
pub type MeshRxRouter<'foa> = RxRouter<'foa, RX_QUEUE_DEPTH, MeshRxRouterOperation>;
pub type MeshRxRouterEndpoint<'foa, 'router> =
    RxRouterEndpoint<'foa, 'router, RX_QUEUE_DEPTH, MeshRxRouterOperation>;
pub type MeshRxRouterInput<'foa, 'router> =
    RxRouterInput<'foa, 'router, RX_QUEUE_DEPTH, MeshRxRouterOperation>;
