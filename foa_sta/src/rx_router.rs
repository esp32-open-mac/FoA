use core::cell::Cell;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use ieee80211::{
    common::{FrameType, ManagementFrameSubtype},
    mac_parser::MACAddress,
    GenericFrame,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Connecting(MACAddress),
    Scanning,
}
impl Operation {
    /// Check if that frame type is relevant for this operation.
    fn frame_type_relevant_for_operation(&self, frame_type: FrameType) -> bool {
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
/// An [RxRouter] operation, that self terminates when going out of scope.
pub struct ScopedRouterOperation<'a> {
    rx_router: &'a RxRouter,
    router_queue: RouterQueue,
}
impl ScopedRouterOperation<'_> {
    /// Mark this operation as completed.
    ///
    /// This doesn't actually do anything, except consume `self` and therefore run the drop code.
    /// It's meant as a way to make the life cycle of an operation clearer, then using the implicit
    /// drop.
    pub fn complete(self) {}
}
impl Drop for ScopedRouterOperation<'_> {
    fn drop(&mut self) {
        self.rx_router.end_operation(self.router_queue);
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// The two queues used for RX.
pub enum RouterQueue {
    User,
    Background,
}
/// Handles routing of received frames and operations.
pub struct RxRouter {
    rx_state: Cell<Option<(RouterQueue, Operation)>>,
    operation_done: Signal<NoopRawMutex, ()>,
}
impl RxRouter {
    /// Create a new [RxRouter].
    pub const fn new() -> Self {
        Self {
            rx_state: Cell::new(None),
            operation_done: Signal::new(),
        }
    }
    /// Try to begin an operation for the specified queue.
    ///
    /// This returns `false`, if an operation is already in progress by another queue.
    pub fn try_begin_operation(&self, router_queue: RouterQueue, operation: Operation) -> bool {
        if let Some((current_router_queue, _)) = self.rx_state.get() {
            if current_router_queue != router_queue {
                return false;
            }
        }
        self.rx_state.set(Some((router_queue, operation)));
        true
    }
    #[allow(dead_code)]
    /// Attempt to initiate a scoped operation.
    pub fn try_begin_scoped_operation(
        &self,
        router_queue: RouterQueue,
        operation: Operation,
    ) -> Option<ScopedRouterOperation<'_>> {
        self.try_begin_operation(router_queue, operation)
            .then(|| ScopedRouterOperation {
                rx_router: self,
                router_queue,
            })
    }
    /// Begin an operation and wait for a previous one to finish.
    pub async fn begin_operation(&self, router_queue: RouterQueue, operation: Operation) {
        self.wait_for_operation_to_finish().await;
        let _ = self.try_begin_operation(router_queue, operation);
    }
    /// Initiate a scoped operation and wait for another one to finish first.
    pub async fn begin_scoped_operation(
        &self,
        router_queue: RouterQueue,
        operation: Operation,
    ) -> ScopedRouterOperation<'_> {
        self.begin_operation(router_queue, operation).await;
        ScopedRouterOperation {
            rx_router: self,
            router_queue,
        }
    }
    /// End an operation for the specified queue.
    ///
    /// If no operations are in progress by that queue, this doesn't do anything.
    pub fn end_operation(&self, rx_queue: RouterQueue) {
        // By taking the state here, it will be reset to None, since that's the default.
        let Some(state) = self.rx_state.take() else {
            // We don't signal a completed operation here, since none was in progress.
            return;
        };
        // If the queues don't match, we restore the state.
        if state.0 != rx_queue {
            self.rx_state.set(Some(state));
        }
        self.operation_done.signal(());
    }
    /// Wait for any pending operations to finish.
    pub async fn wait_for_operation_to_finish(&self) {
        if self.rx_state.get().is_none() {
            return;
        }
        self.operation_done.wait().await
    }
    /// Determine the queue, the frame should be routed to.
    pub fn target_queue_for_frame(&self, generic_frame: &GenericFrame<'_>) -> RouterQueue {
        match self.rx_state.get() {
            Some((rx_queue, operation)) => {
                if operation.frame_type_relevant_for_operation(
                    generic_frame.frame_control_field().frame_type(),
                ) {
                    rx_queue
                } else {
                    RouterQueue::Background
                }
            }
            None => RouterQueue::Background,
        }
    }
    /// Returns the MAC address in [Operation::Connecting].
    pub fn get_connecting_mac_address(&self) -> Option<MACAddress> {
        match self.rx_state.get() {
            Some((_, Operation::Connecting(mac_address))) => Some(mac_address),
            _ => None,
        }
    }
}
