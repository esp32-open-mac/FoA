//! This module contains a utility for routing received frames inside an interface.
//!
//! The design of some interface implementations (see `foa_sta`) requires processing some frames in
//! the background task and others in the users task, when a certain operation is initiated through
//! the interface control. `foa_sta` uses this for connecting and scanning, since those can (in
//! theory) be initiated by the user and the background task.
//!
//! ## Basic Operation
//! The RX router has two queues, which are labeled foreground and background, and routes the frame
//! based on the currently active operation. The [RxRouter] struct has a generic parameter for the
//! operation, which should be an enum containing all the different operations supported by the
//! interface implementation. For the STA interface, this would currently be scanning or
//! connecting. By default all frames are routed to the background queue, which would usually
//! terminate in the background task. If an operation is started through the [RxRouterEndpoint] of
//! the foreground queue all frames, for which [RxRouterOperation::frame_relevant_for_operation]
//! returned `true`, will be routed to foreground queue. It is also possible to start an operation
//! through the [RxRouterEndpoint] of the background queue, although this doesn't influence
//! routing, since all otherwise unmatched frames go to the background queue, but instead prevents
//! starting an operation from the endpoint of the foreground queue.
//!
//! ## Usage
//! Since the [RxRouter], [RxRouterEndpoint] and [RxRouterInput] structs have two generic type
//! parameters, that are dependent on the interface implementation, it's recommended to define type
//! aliases for these, since it would be fairly cumbersome to have the generics in the entire code
//! base.
//!
//! When using this in an interface implementation, the [RxRouter] struct should be placed in the
//! resources struct for the interface and [RxRouter::split] called on initialization. The returned
//! [RxRouterInput] should then be used by the part of the code handling RX, where
//! [RxRouterInput::route_frame] can be used to route the frame provided to the correct queue
//! depending on the currently active operation. The [RxRouterEndpoint]s should be passed to the
//! entity handling foreground operations (usually user control) and the background task, where
//! they can be used to receive frames and start operations with
//! [RxRouterEndpoint::start_router_operation]. The returned [RxRouterScopedOperation]
//! auto-terminates, when it's dropped.
//!
//! ### Important Note:
//! If at all possible, data frames should be processed before passing through the RX router,
//! unless they are strictly related to an operation. This is due to the RX router introducing
//! extra latency, since the frames have to pass through one more queue than strictly necessary,
//! which is acceptable for management frames but not data frames carrying actual MSDUs.

use core::{cell::Cell, ops::Deref};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, DynamicReceiveFuture},
    signal::Signal,
};
use ieee80211::GenericFrame;

use crate::ReceivedFrame;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
/// The queues inside the [RxRouter].
pub enum RxRouterQueue {
    #[default]
    /// Queue for foreground operations.
    ///
    /// This is usually for running operations, which were initiated through the user control
    /// struct, in the user context.
    Foreground,
    /// Queue for the background task.
    Background,
}
impl RxRouterQueue {
    /// Get the opposing router queue.
    pub const fn opposite(&self) -> Self {
        match self {
            Self::Foreground => Self::Background,
            Self::Background => Self::Foreground,
        }
    }
}

/// A trait indicating the presence of a scan like operation.
///
/// It is expected, that frames like beacons and probe responses will be matched for this operation.
pub trait HasScanOperation: RxRouterOperation {
    const SCAN_OPERATION: Self;
}

/// A trait implemented by an enum containing operations for the [RxRouter].
pub trait RxRouterOperation: Clone + Copy {
    /// Indicates support for concurrent operations.
    ///
    /// This is used for a optimization in the routing. A consequnce of this optimization is, that
    /// if this is false, [RxRouterOperation::compatible_with] will not be called.
    const CONCORRUENT_OPERATIONS_SUPPORTED: bool = false;

    /// Check if the frame is relevant for this operation.
    fn frame_relevant_for_operation(&self, generic_frame: &GenericFrame<'_>) -> bool;

    /// Check if this operation can happen concurrently, with the specified one.
    fn compatible_with(&self, _other: &Self) -> bool {
        false
    }
}

/// Errors returned by the RX router.
pub enum RxRouterError {
    OperationAlreadyInProgress,
    IncompatibleOperations,
    QueueFull,
    InvalidFrame,
}

/// Used for dynamic dispatch.
trait Router<'foa, Operation: RxRouterOperation> {
    /// Receive a frame from the specified [RxRouterQueue].
    fn receive(&self, router_queue: RxRouterQueue)
        -> DynamicReceiveFuture<'_, ReceivedFrame<'foa>>;
    //// Route a frame to a specific queue.
    fn route_frame(&self, frame: ReceivedFrame<'foa>) -> Result<(), RxRouterError>;
    /// Get the operation state for the specified [RxRouterQueue].
    fn operation_state(&self, router_queue: RxRouterQueue) -> &Cell<Option<Operation>>;
    /// Get the operation completion signal.
    fn completion_signal(&self) -> &Signal<NoopRawMutex, ()>;
}

/// State of one of the router queues.
struct QueueState<'foa, const QUEUE_DEPTH: usize, O: RxRouterOperation> {
    queue: Channel<NoopRawMutex, ReceivedFrame<'foa>, QUEUE_DEPTH>,
    operation_state: Cell<Option<O>>,
}
impl<const QUEUE_DEPTH: usize, O: RxRouterOperation> QueueState<'_, QUEUE_DEPTH, O> {
    /// Create a new queue state.
    pub const fn new() -> Self {
        Self {
            queue: Channel::new(),
            operation_state: Cell::new(None),
        }
    }
    /// Check if the specified frame is relevant for the currently active operation.
    ///
    /// Returns `false` if none is active.
    fn frame_relevant_for_operation(&self, generic_frame: &GenericFrame<'_>) -> bool {
        self.operation_state
            .get()
            .map(|operation| operation.frame_relevant_for_operation(generic_frame))
            .unwrap_or(false)
    }
    /// Check if an operation is active.
    fn operation_active(&self) -> bool {
        self.operation_state.get().is_some()
    }
}

/// The core of the RX router.
///
/// This is only used for allocating the resources necessary for the RX router. All functionality
/// is exposed through the [RxRouterInput] and [RxRouterEndpoint] structs, which are returned by
/// [RxRouter::split].
pub struct RxRouter<'foa, const QUEUE_DEPTH: usize, Operation: RxRouterOperation> {
    foreground_queue_state: QueueState<'foa, QUEUE_DEPTH, Operation>,
    background_queue_state: QueueState<'foa, QUEUE_DEPTH, Operation>,

    completion_signal: Signal<NoopRawMutex, ()>,
}
impl<'foa, const QUEUE_DEPTH: usize, Operation: RxRouterOperation>
    RxRouter<'foa, QUEUE_DEPTH, Operation>
{
    /// Create a new RX router.
    pub const fn new() -> Self {
        Self {
            foreground_queue_state: QueueState::new(),
            background_queue_state: QueueState::new(),

            completion_signal: Signal::new(),
        }
    }
    /// Split the RX router.
    ///
    /// The first [RxRouterEndpoint] is for the foreground queue and the second for the background
    /// queue.
    pub const fn split<'router>(
        &'router mut self,
    ) -> (
        RxRouterInput<'foa, 'router, Operation>,
        [RxRouterEndpoint<'foa, 'router, Operation>; 2],
    ) {
        (
            RxRouterInput { rx_router: self },
            [
                RxRouterEndpoint {
                    rx_router: self,
                    router_queue: RxRouterQueue::Foreground,
                },
                RxRouterEndpoint {
                    rx_router: self,
                    router_queue: RxRouterQueue::Background,
                },
            ],
        )
    }
    fn queue_state(
        &self,
        router_queue: RxRouterQueue,
    ) -> &QueueState<'foa, QUEUE_DEPTH, Operation> {
        match router_queue {
            RxRouterQueue::Foreground => &self.foreground_queue_state,
            RxRouterQueue::Background => &self.background_queue_state,
        }
    }
}
impl<'foa, const QUEUE_DEPTH: usize, Operation: RxRouterOperation> Router<'foa, Operation>
    for RxRouter<'foa, QUEUE_DEPTH, Operation>
{
    fn receive(
        &self,
        router_queue: RxRouterQueue,
    ) -> DynamicReceiveFuture<'_, ReceivedFrame<'foa>> {
        self.queue_state(router_queue).queue.receive().into()
    }
    fn route_frame(&self, frame: ReceivedFrame<'foa>) -> Result<(), RxRouterError> {
        // Usually the RX handler will have already created a GenericFrame, however we can't
        // reasonably assume that, so we recreate it here and hope the compiler opimizes away.
        let Ok(generic_frame) = GenericFrame::new(frame.mpdu_buffer(), false) else {
            return Err(RxRouterError::InvalidFrame);
        };
        let router_queue = if self
            .foreground_queue_state
            .frame_relevant_for_operation(&generic_frame)
        {
            RxRouterQueue::Foreground
        } else if !Operation::CONCORRUENT_OPERATIONS_SUPPORTED
            || !self.background_queue_state.operation_active()
            || self
                .background_queue_state
                .frame_relevant_for_operation(&generic_frame)
        {
            RxRouterQueue::Background
        } else {
            return Ok(());
        };
        if self.queue_state(router_queue).queue.try_send(frame).is_ok() {
            Ok(())
        } else {
            Err(RxRouterError::QueueFull)
        }
    }
    fn operation_state(&self, router_queue: RxRouterQueue) -> &Cell<Option<Operation>> {
        &self.queue_state(router_queue).operation_state
    }
    fn completion_signal(&self) -> &Signal<NoopRawMutex, ()> {
        &self.completion_signal
    }
}
impl<const QUEUE_DEPTH: usize, Operation: RxRouterOperation> Default
    for RxRouter<'_, QUEUE_DEPTH, Operation>
{
    fn default() -> Self {
        Self::new()
    }
}

/// Input for the [RxRouter].
pub struct RxRouterInput<'foa, 'router, Operation: RxRouterOperation> {
    rx_router: &'router dyn Router<'foa, Operation>,
}

impl<'foa, Operation: RxRouterOperation> RxRouterInput<'foa, '_, Operation> {
    /// Route the provided frame to the correct queue.
    pub fn route_frame(&self, frame: ReceivedFrame<'foa>) -> Result<(), RxRouterError> {
        self.rx_router.route_frame(frame)
    }
    pub fn operation(&self, router_queue: RxRouterQueue) -> Option<Operation> {
        self.rx_router.operation_state(router_queue).get()
    }
}
/// A scoped router operation.
///
/// The operation will end, if either [RxRouterScopedOperation::complete] or this is dropped.
/// To start a scoped router operation call [RxRouterEndpoint::start_router_operation].
pub struct RxRouterScopedOperation<'foa, 'router, 'endpoint, Operation: RxRouterOperation> {
    endpoint: &'endpoint RxRouterEndpoint<'foa, 'router, Operation>,
}
impl<Operation: RxRouterOperation> RxRouterScopedOperation<'_, '_, '_, Operation> {
    /// Mark the operation as completed.
    pub fn complete(self) {}
}
impl<'foa, 'router, Operation: RxRouterOperation> Deref
    for RxRouterScopedOperation<'foa, 'router, '_, Operation>
{
    type Target = RxRouterEndpoint<'foa, 'router, Operation>;
    fn deref(&self) -> &Self::Target {
        self.endpoint
    }
}
impl<O: RxRouterOperation> Drop for RxRouterScopedOperation<'_, '_, '_, O> {
    fn drop(&mut self) {
        // Due to how `Cell` works, this resets it back to `None`.
        self.endpoint
            .rx_router
            .operation_state(self.endpoint.router_queue)
            .take();
        self.endpoint.rx_router.completion_signal().signal(());
    }
}
/// An endpoint for one of the two [RxRouter] queues.
///
/// See the module level documentation for more information.
pub struct RxRouterEndpoint<'foa, 'router, Operation: RxRouterOperation> {
    rx_router: &'router dyn Router<'foa, Operation>,
    router_queue: RxRouterQueue,
}
impl<'foa, 'router, Operation: RxRouterOperation> RxRouterEndpoint<'foa, 'router, Operation> {
    /// Get the router queue of this endpoint.
    pub const fn router_queue(&self) -> RxRouterQueue {
        self.router_queue
    }
    /// Wait for a [ReceivedFrame] to arrive from the endpoints RX queue.
    pub fn receive(&self) -> DynamicReceiveFuture<'router, ReceivedFrame<'foa>> {
        self.rx_router.receive(self.router_queue)
    }
    /// Check if the operation can be started.
    fn can_start_operation(&self, operation: Operation) -> Result<(), RxRouterError> {
        if let Some(opposing_operation) = self
            .rx_router
            .operation_state(self.router_queue.opposite())
            .get()
        {
            if !Operation::CONCORRUENT_OPERATIONS_SUPPORTED {
                Err(RxRouterError::OperationAlreadyInProgress)
            } else if !opposing_operation.compatible_with(&operation) {
                Err(RxRouterError::IncompatibleOperations)
            } else {
                Ok(())
            }
        } else {
            Ok(())
        }
    }
    /// Set the internal state to start the opperation.
    fn start_operation_internal(&self, operation: Operation) {
        self.rx_router.completion_signal().reset();
        self.rx_router
            .operation_state(self.router_queue)
            .set(Some(operation));
    }
    /// Attempt to start a router operation.
    ///
    /// If another operation is currently active, [RxRouterError::OperationAlreadyInProgress] will
    /// be returned.
    pub fn try_start_operation<'endpoint>(
        &'endpoint mut self,
        operation: Operation,
    ) -> Result<RxRouterScopedOperation<'foa, 'router, 'endpoint, Operation>, RxRouterError> {
        self.can_start_operation(operation)?;
        self.start_operation_internal(operation);
        Ok(RxRouterScopedOperation { endpoint: self })
    }
    pub async fn start_operation<'endpoint>(
        &'endpoint mut self,
        operation: Operation,
    ) -> RxRouterScopedOperation<'foa, 'router, 'endpoint, Operation> {
        if self.can_start_operation(operation).is_err() {
            self.rx_router.completion_signal().wait().await;
        }
        self.start_operation_internal(operation);
        RxRouterScopedOperation { endpoint: self }
    }
}
