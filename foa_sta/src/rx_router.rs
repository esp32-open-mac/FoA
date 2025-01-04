use core::cell::Cell;

use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    signal::Signal,
};
use ieee80211::{
    common::{FrameType, ManagementFrameSubtype},
    GenericFrame,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Connecting,
    Scanning,
}
impl Operation {
    /// Check if that frame type is relevant for this operation.
    fn frame_type_relevant_for_operation(&self, frame_type: FrameType) -> bool {
        if *self == Self::Connecting {
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
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The two queues used for RX.
pub enum RxQueue {
    User,
    Background,
}
/// Handles routing of received frames and operations.
pub struct RxRouter {
    rx_state: NoopMutex<Cell<Option<(RxQueue, Operation)>>>,
    operation_done: Signal<NoopRawMutex, ()>,
}
impl RxRouter {
    /// Create a new [RxRouter].
    pub const fn new() -> Self {
        Self {
            rx_state: NoopMutex::new(Cell::new(None)),
            operation_done: Signal::new(),
        }
    }
    /// Try to begin an operation for the specified queue.
    ///
    /// This returns `false`, if an operation is already in progress by another queue.
    pub fn try_begin_operation(&self, rx_queue: RxQueue, operation: Operation) -> bool {
        self.rx_state.lock(|cell| {
            if let Some((current_rx_queue, _)) = cell.get() {
                if current_rx_queue != rx_queue {
                    return false;
                }
            }
            cell.set(Some((rx_queue, operation)));
            true
        })
    }
    /// Begin an operation and wait for a previous one to finish.
    pub async fn begin_operation(&self, rx_queue: RxQueue, operation: Operation) {
        self.wait_for_operation_to_finish().await;
        let _ = self.try_begin_operation(rx_queue, operation);
    }
    /// End an operation for the specified queue.
    ///
    /// If no operations are in progress by that queue, this doesn't do anything.
    pub fn end_operation(&self, rx_queue: RxQueue) {
        self.rx_state.lock(|cell| {
            // By taking the state here, it will be reset to None, since that's the default.
            let Some(state) = cell.take() else {
                // We don't signal a completed operation here, since none was in progress.
                return;
            };
            // If the queues don't match, we restore the state.
            if state.0 != rx_queue {
                cell.set(Some(state));
            }
            self.operation_done.signal(());
        })
    }
    /// Wait for any pending operations to finish.
    pub async fn wait_for_operation_to_finish(&self) {
        if self.rx_state.lock(Cell::get).is_none() {
            return;
        }
        self.operation_done.wait().await
    }
    /// Determine the queue, the frame should be routed to.
    pub fn target_queue_for_frame(&self, generic_frame: &GenericFrame<'_>) -> RxQueue {
        match self.rx_state.lock(Cell::get) {
            Some((rx_queue, operation)) => {
                if operation.frame_type_relevant_for_operation(
                    generic_frame.frame_control_field().frame_type(),
                ) {
                    rx_queue
                } else {
                    RxQueue::Background
                }
            }
            None => RxQueue::Background,
        }
    }
}
