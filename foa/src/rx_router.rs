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
    channel::{Channel, ReceiveFuture},
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

/// A trait implemented by an enum containing operations for the [RxRouter].
pub trait RxRouterOperation: Clone + Copy {
    /// Check if the frame is relevant for this operation.
    fn frame_relevant_for_operation(&self, generic_frame: GenericFrame<'_>) -> bool;
}

/// Errors returned by the RX router.
pub enum RxRouterError {
    OperationAlreadyInProgress,
    QueueFull,
    InvalidFrame,
}

/// The core of the RX router.
///
/// This is only used for allocating the resources necessary for the RX router. All functionality
/// is exposed through the [RxRouterInput] and [RxRouterEndpoint] structs, which are returned by
/// [RxRouter::split].
pub struct RxRouter<'foa, const QUEUE_DEPTH: usize, O: RxRouterOperation> {
    background_queue: Channel<NoopRawMutex, ReceivedFrame<'foa>, QUEUE_DEPTH>,
    foreground_queue: Channel<NoopRawMutex, ReceivedFrame<'foa>, QUEUE_DEPTH>,
    operation_state: Cell<Option<(RxRouterQueue, O)>>,
}
impl<'foa, const QUEUE_DEPTH: usize, O: RxRouterOperation> RxRouter<'foa, QUEUE_DEPTH, O> {
    /// Create a new RX router.
    pub const fn new() -> Self {
        Self {
            background_queue: Channel::new(),
            foreground_queue: Channel::new(),
            operation_state: Cell::new(None),
        }
    }
    /// Split the RX router.
    ///
    /// The first [RxRouterEndpoint] is for the foreground queue and the second for the background
    /// queue.
    pub const fn split<'router>(
        &'router mut self,
    ) -> (
        RxRouterInput<'foa, 'router, QUEUE_DEPTH, O>,
        [RxRouterEndpoint<'foa, 'router, QUEUE_DEPTH, O>; 2],
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
    /// Check if an operation is in progress.
    pub fn operation_in_progress(&self) -> bool {
        self.operation_state.get().is_some()
    }
    /// Get the currently active operation, if any.
    pub fn current_operation(&self) -> Option<O> {
        self.operation_state
            .get()
            .map(|(_router_queue, operation)| operation)
    }
    /// Get the queue currently performing an operation, if any.
    pub fn currently_operating_queue(&self) -> Option<RxRouterQueue> {
        self.operation_state
            .get()
            .map(|(router_queue, _operation)| router_queue)
    }
    /// Get the currently active foreground operation, if any.
    pub fn foreground_operation(&self) -> Option<O> {
        self.operation_state
            .get()
            .and_then(|(router_queue, operation)| {
                (router_queue == RxRouterQueue::Foreground).then_some(operation)
            })
    }
    /// Try to start an operation.
    ///
    /// Returns [RxRouterError::OperationAlreadyInProgress], if another operation is currently in
    /// progress.
    fn start_operation(
        &self,
        router_queue: RxRouterQueue,
        operation: O,
    ) -> Result<(), RxRouterError> {
        if self.operation_in_progress() {
            Err(RxRouterError::OperationAlreadyInProgress)
        } else {
            self.operation_state.set(Some((router_queue, operation)));
            Ok(())
        }
    }
}
impl<const QUEUE_DEPTH: usize, O: RxRouterOperation> Default for RxRouter<'_, QUEUE_DEPTH, O> {
    fn default() -> Self {
        Self::new()
    }
}

/// Input for the [RxRouter].
pub struct RxRouterInput<'foa, 'router, const QUEUE_DEPTH: usize, O: RxRouterOperation> {
    rx_router: &'router RxRouter<'foa, QUEUE_DEPTH, O>,
}

impl<'foa, const QUEUE_DEPTH: usize, O: RxRouterOperation> RxRouterInput<'foa, '_, QUEUE_DEPTH, O> {
    /// Route the provided frame to the correct queue.
    pub fn route_frame(&self, frame: ReceivedFrame<'foa>) -> Result<(), RxRouterError> {
        // Usually the RX handler will have already created a GenericFrame, however we can't
        // reasonably assume that, so we recreate it here and hope the compiler opimizes away.
        let Ok(generic_frame) = GenericFrame::new(frame.mpdu_buffer(), false) else {
            return Err(RxRouterError::InvalidFrame);
        };
        match self.rx_router.foreground_operation() {
            Some(operation) if operation.frame_relevant_for_operation(generic_frame) => {
                &self.rx_router.foreground_queue
            }
            _ => &self.rx_router.background_queue,
        }
        .try_send(frame)
        .map_err(|_| RxRouterError::QueueFull)
    }
}
impl<'foa, const QUEUE_DEPTH: usize, O: RxRouterOperation> Deref
    for RxRouterInput<'foa, '_, QUEUE_DEPTH, O>
{
    type Target = RxRouter<'foa, QUEUE_DEPTH, O>;
    fn deref(&self) -> &Self::Target {
        self.rx_router
    }
}
/// A scoped router operation.
///
/// The operation will end, if either [RxRouterScopedOperation::complete] or this is dropped.
/// To start a scoped router operation call [RxRouterEndpoint::start_router_operation].
pub struct RxRouterScopedOperation<'router, O: RxRouterOperation> {
    operation_state: &'router Cell<Option<(RxRouterQueue, O)>>,
}
impl<O: RxRouterOperation> RxRouterScopedOperation<'_, O> {
    /// Mark the operation as completed.
    pub fn complete(self) {}
}
impl<O: RxRouterOperation> Drop for RxRouterScopedOperation<'_, O> {
    fn drop(&mut self) {
        // Due to how `Cell` works, this resets it back to `None`.
        self.operation_state.take();
    }
}
/// An endpoint for one of the two [RxRouter] queues.
///
/// See the module level documentation for more information.
pub struct RxRouterEndpoint<'foa, 'router, const QUEUE_DEPTH: usize, O: RxRouterOperation> {
    rx_router: &'router RxRouter<'foa, QUEUE_DEPTH, O>,
    router_queue: RxRouterQueue,
}
impl<'foa, 'router, const QUEUE_DEPTH: usize, O: RxRouterOperation>
    RxRouterEndpoint<'foa, 'router, QUEUE_DEPTH, O>
{
    /// Wait for a [ReceivedFrame] to arrive from the endpoints RX queue.
    pub fn receive(
        &self,
    ) -> ReceiveFuture<'router, NoopRawMutex, ReceivedFrame<'foa>, QUEUE_DEPTH> {
        match self.router_queue {
            RxRouterQueue::Foreground => &self.rx_router.foreground_queue,
            RxRouterQueue::Background => &self.rx_router.background_queue,
        }
        .receive()
    }
    /// Attempt to start a router operation.
    ///
    /// If another operation is currently active, [RxRouterError::OperationAlreadyInProgress] will
    /// be returned.
    pub fn start_router_operation(
        &self,
        operation: O,
    ) -> Result<RxRouterScopedOperation<'router, O>, RxRouterError> {
        self.rx_router
            .start_operation(self.router_queue, operation)
            .and(Ok(RxRouterScopedOperation {
                operation_state: &self.rx_router.operation_state,
            }))
    }
}
impl<'foa, const QUEUE_DEPTH: usize, O: RxRouterOperation> Deref
    for RxRouterEndpoint<'foa, '_, QUEUE_DEPTH, O>
{
    type Target = RxRouter<'foa, QUEUE_DEPTH, O>;
    fn deref(&self) -> &Self::Target {
        self.rx_router
    }
}
