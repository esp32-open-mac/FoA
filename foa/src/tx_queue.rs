use core::{
    cell::{Cell, RefCell},
    future::poll_fn,
    sync::atomic::Ordering,
    task::Poll,
};

use embassy_sync::{
    blocking_mutex::{self, raw::NoopRawMutex},
    mutex::Mutex,
    waitqueue::AtomicWaker,
};
use esp_wifi_hal::{
    ll::EdcaAccessCategory,
    prelude::{TxError, TxErrorBehaviour, TxMacParameters, TxPlcpParameters, TxQueueEndpoint},
};
use portable_atomic::AtomicBool;

use crate::{TX_BUFFER_COUNT, TxBuffer, tx_buffer_management::TxBufferManager};

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// Should the transmission be retried if an error occurs, and if so how.
pub enum RetryBehaviour {
    #[default]
    /// Drop the MPDU.
    Drop,
    /// Retry as many times, as specified.
    ///
    /// With the same rate.
    RetryUntil(u8),
    #[cfg(feature = "multi_rate_retry")]
    /// Retry with the specified rates.
    ///
    /// Rate control algorithms like this.
    MultiRateRetry(heapless::Vec<esp_wifi_hal::rates::TxPhyRate, 3>),
}
impl RetryBehaviour {
    /// Convert this to a [TxErrorBehaviour].
    fn as_driver_behaviour(&self) -> TxErrorBehaviour<'_> {
        match self {
            Self::Drop => TxErrorBehaviour::Drop,
            Self::RetryUntil(retries) => TxErrorBehaviour::RetryUntil(*retries),
    #[cfg(feature = "multi_rate_retry")]
            Self::MultiRateRetry(rates) => TxErrorBehaviour::MultiRateRetry(rates.as_slice()),
        }
    }
}
/// A frame pending for transmission.
pub struct PendingFrame {
    /// The buffer that should be transmitted.
    frame: TxBuffer<'static>,
    frame_length: usize,

    // Transmission parameters.
    plcp_parameters: TxPlcpParameters,
    mac_parameters: TxMacParameters,
    interface: u8,
    retry_behaviour: RetryBehaviour,
}
/// Data returned by transmitting.
pub struct TxReturnData<'res> {
    /// The transmission result.
    pub result: Result<u8, TxError>,
    /// The transmitted frame.
    ///
    /// This is provided, in case you want to retransmit the same buffer.
    pub frame: TxBuffer<'res>,
    /// The length of the transmitted frame.
    pub frame_length: usize,
}
#[derive(Default)]
/// A slot in the TX queue.
enum TxQueueSlot {
    #[default]
    /// The slot is empty and ready to be used.
    Empty,
    /// The slot contains a frame pending for TX.
    Pending(PendingFrame),
    /// Transmission of the frame previously in this slot is in progress.
    InProgress,
    /// Return data is available for the transmission of the frame previously in this slot.
    ///
    /// NOTE: A slot in this state does not count as 'occupied' and can be overwritten at any time.
    /// This is to prevent a task, that takes to long to retrieve it's return data, from blocking
    /// the queue.
    ReturnDataAvailable(TxReturnData<'static>),
}
struct TxQueueState {
    /// The queue items.
    ///
    /// The waker is used to signal completion of transmissions.
    /// The [AtomicBool] indicates, whether return data is expected for the slot or not.
    queue_items: [(Cell<TxQueueSlot>, AtomicWaker, AtomicBool); TX_BUFFER_COUNT],
    /// Continuously running counter of the queue.
    ///
    /// This is used to track frames that are in flight, as the slot may be overwritten, by the
    /// time it is read.
    counter: u64,
    /// Capacity left in the queue.
    capacity: usize,
    /// Signals that a frame was enqueued.
    /// 
    /// This is technically not necessary, as we could also use the waker on the front slot, if it is
    /// emtpy, however calculating the front slot requires a remainder operation, which isn't exactly
    /// free. Therefore we trade four bytes for a non negligible performance improvement.
    queue_update_waker: AtomicWaker,
}
impl TxQueueState {
    pub const fn new() -> Self {
        Self {
            queue_items: [const {
                (
                    Cell::new(TxQueueSlot::Empty),
                    AtomicWaker::new(),
                    AtomicBool::new(true),
                )
            }; TX_BUFFER_COUNT],
            counter: 0,
            capacity: TX_BUFFER_COUNT,
            queue_update_waker: AtomicWaker::new()
        }
    }
    /// Get the index of the slot, where the next queue item should be inserted.
    const fn next_slot_index(&self) -> usize {
        (self.counter % TX_BUFFER_COUNT as u64) as usize
    }
    /// Get the index of oldest item in the queue.
    fn front_index(&self) -> usize {
        let len = self.len();
        // Index of the most recently inserted item.
        let next_slot_index = self.next_slot_index();
        if next_slot_index >= len {
            // The ring is currently not wrapped.
            next_slot_index - len
        } else {
            let overrun_count = len - next_slot_index;
            TX_BUFFER_COUNT - overrun_count
        }
    }
    /// Increase the queue counter and get the previous value.
    const fn increase_queue_counter(&mut self) -> u64 {
        let counter = self.counter;
        self.counter += 1;

        counter
    }
    /// Is the queue empty.
    const fn is_empty(&self) -> bool {
        self.capacity == TX_BUFFER_COUNT
    }
    /// The number of frames currently in the queue.
    const fn len(&self) -> usize {
        TX_BUFFER_COUNT - self.capacity
    }
    /// Has the internal counter looped over the provided counter value.
    const fn is_counter_out_of_bounds(&self, counter: u64) -> bool {
        (self.counter - counter) >= TX_BUFFER_COUNT as u64
    }
}

/// An asynchronous transmit queue.
pub struct TxQueue {
    /// Internal state of the queue.
    inner: blocking_mutex::NoopMutex<RefCell<TxQueueState>>,
}
impl TxQueue {
    pub const fn new() -> Self {
        Self {
            inner: blocking_mutex::NoopMutex::new(RefCell::new(TxQueueState::new())),
        }
    }
    /// Enqueue a frame for transmission.
    pub fn enqueue_frame(&self, frame: PendingFrame) -> PendingTransmission<'_> {
        self.inner.lock(|rc| {
            let mut queue_state = rc.borrow_mut();

            let was_queue_empty = queue_state.is_empty();

            let next_slot_index = queue_state.next_slot_index();
            let counter = queue_state.increase_queue_counter();
            queue_state.capacity -= 1;

            queue_state.queue_items[next_slot_index]
                .0
                .set(TxQueueSlot::Pending(frame));

            // Prevents unnecessary wakes of the background task.
            if was_queue_empty {
                queue_state.queue_update_waker.wake();
            }

            PendingTransmission {
                tx_queue: self,
                counter,
            }
        })
    }
}
pub struct TxQueueRunner<'res> {
    pub(crate) tx_queue: &'res TxQueue,
    pub(crate) tx_endpoint: TxQueueEndpoint<'res>,
}
impl<'res> TxQueueRunner<'res> {
    /// Waits for the queue to not be empty (duh).
    fn wait_queue_not_empty(&self) -> impl Future<Output = ()> {
        poll_fn(|cx| {
            self.tx_queue.inner.lock(|rc| {
                let tx_queue_state = rc.borrow();

                if tx_queue_state.is_empty() {
                    tx_queue_state.queue_update_waker.register(cx.waker());
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
        })
    }
    /// Wait for a frame to become available in the queue.
    fn try_receive<'a>(
        tx_queue: &'a TxQueue,
    ) -> Option<(InProgressTransmission<'a>, PendingFrame)> {
            tx_queue.inner.lock(|rc| {
                // We look for a pending frame at the front of the queue.
                let mut queue_state = rc.borrow_mut();

                // This is an optimisation for polls on an empty queue.
                if queue_state.is_empty() {
                    return None;
                }

                let front_index = queue_state.front_index();

                let (
                    ref mut front_slot_cell, 
                    _, 
                    ref return_data_expeceted
                ) = queue_state.queue_items[front_index];
                match front_slot_cell.get_mut() {
                    TxQueueSlot::Empty | TxQueueSlot::ReturnDataAvailable(_) => {
                        unreachable!("Queue wasn't marked as empty, however the front slot was.");
                    }
                    TxQueueSlot::Pending(_) => {
                        // If a frame is pending, we take it out of the queue and return the pending frame.
                        let swapped_slot_state = front_slot_cell.replace(TxQueueSlot::InProgress);

                        let TxQueueSlot::Pending(pending_frame) = swapped_slot_state else {
                            unreachable!();
                        };

                        Some(
                            (InProgressTransmission {
                                tx_queue,
                                index: front_index,
                                return_data_expected: return_data_expeceted.load(Ordering::Relaxed),
                            }, pending_frame)
                        )
                    }
                    TxQueueSlot::InProgress => {
                        unreachable!(
                            "TX queue front slot is in progress. This should not be possible, unless an 
                            ActiveTransmission was passed to core::mem::forget."
                        )
                    }
                }
            })
    }
    pub async fn run(&mut self) {
        loop {
            self.wait_queue_not_empty().await;
            while let Some((in_progress_transmission, pending_frame)) = Self::try_receive(self.tx_queue) {
                in_progress_transmission.transmit(pending_frame, &mut self.tx_endpoint).await;
            }
        }
    }
}
/// An actively processed transmission.
///
///
/// This exists to decouple [TxQueueReceiver::receive] from the actual transmitting part,
/// while enforcing any internal invariants.
pub struct InProgressTransmission<'res> {
    tx_queue: &'res TxQueue,
    /// The queue slot, to which this transmission corresponds.
    index: usize,
    return_data_expected: bool,
}
impl InProgressTransmission<'_> {
    /// Execute an active transmission.
    pub async fn transmit(self, mut pending_frame: PendingFrame, tx_endpoint: &mut TxQueueEndpoint<'_>) {
        let result = tx_endpoint
            .transmit(
                pending_frame.interface as usize,
                &pending_frame.plcp_parameters,
                &pending_frame.mac_parameters,
                pending_frame.retry_behaviour.as_driver_behaviour(),
                &mut pending_frame.frame[..pending_frame.frame_length],
            )
            .await;
        if self.return_data_expected {
            self.finish(TxQueueSlot::ReturnDataAvailable(TxReturnData {
                result,
                frame: pending_frame.frame,
                frame_length: pending_frame.frame_length,
            }));
        } else {
            self.finish(TxQueueSlot::Empty);
        }
        core::mem::forget(self);

    }
    /// Finish the transmission, by updating the queue slot state and by calling the waker.
    fn finish(&self, new_slot_state: TxQueueSlot) {
        self.tx_queue.inner.lock(|ref_cell| {
            let mut tx_queue_state = ref_cell.borrow_mut();
            tx_queue_state.capacity += 1;
            tx_queue_state.queue_items[self.index].0.set(new_slot_state);
            tx_queue_state.queue_items[self.index].1.wake();
            tx_queue_state.queue_items[self.index]
                .2
                .store(true, Ordering::Relaxed);
        })
    }
}
impl Drop for InProgressTransmission<'_> {
    fn drop(&mut self) {
        // In case we drop an active transmission, we reset the slot to empty.
        // NOTE: This should not happen.
        self.finish(TxQueueSlot::Empty);
        error!("Active transmission dropped. This is a bug.")
    }
}
/// Status of a pending transmission.
pub enum PendingTransmissionStatus {
    /// The transmission hasn't started yet.
    Pending,
    /// The background task is currently executing the transmission.
    InProgress,
    /// Return data is available.
    ReturnDataAvailable,
    /// The transmission has already completed and the slot was overwritten.
    ///
    /// This happens if more than [TX_BUFFER_COUNT] frames were enqueued, since this
    /// transmission was completed.
    SlotNoLongerValid,
}

/// A transmission, currently waiting for completion.
pub struct PendingTransmission<'res> {
    tx_queue: &'res TxQueue,
    counter: u64,
}
impl<'res> PendingTransmission<'res> {
    /// Get the slot index corresponding to this transmission.
    ///
    /// This does not check, whether the slot is still valid.
    const fn slot_index(&self) -> usize {
        (self.counter % TX_BUFFER_COUNT as u64) as usize
    }
    /// Get the current status of the transmission.
    pub fn status(&self) -> PendingTransmissionStatus {
        self.tx_queue.inner.lock(|rc| {
            let mut tx_queue_state = rc.borrow_mut();

            if tx_queue_state.is_counter_out_of_bounds(self.counter) {
                PendingTransmissionStatus::SlotNoLongerValid
            } else {
                match tx_queue_state.queue_items[self.slot_index()].0.get_mut() {
                    TxQueueSlot::InProgress => PendingTransmissionStatus::InProgress,
                    TxQueueSlot::Pending(_) => PendingTransmissionStatus::Pending,
                    TxQueueSlot::ReturnDataAvailable(_) => {
                        PendingTransmissionStatus::ReturnDataAvailable
                    }
                    TxQueueSlot::Empty => {
                        error!("Slot was empty, even though counter didn't wrap.");
                        PendingTransmissionStatus::SlotNoLongerValid
                    }
                }
            }
        })
    }
    /// Wait for the transmission to complete.
    ///
    /// If you call this directly after starting the transmission, you should always get the return data.
    pub fn wait_for_completion(self) -> impl Future<Output = Option<TxReturnData<'res>>> {
        poll_fn(move |cx| {
            self.tx_queue.inner.lock(|rc| {
                let mut tx_queue_state = rc.borrow_mut();

                if tx_queue_state.is_counter_out_of_bounds(self.counter) {
                    Poll::Ready(None)
                } else {
                    let (cell, waker, _) = &mut tx_queue_state.queue_items[self.slot_index()];
                    if let TxQueueSlot::ReturnDataAvailable(_) = cell.get_mut() {
                        let TxQueueSlot::ReturnDataAvailable(return_data) = cell.take() else {
                            unreachable!();
                        };
                        Poll::Ready(Some(unsafe {
                            core::mem::transmute::<TxReturnData<'static>, TxReturnData<'res>>(
                                return_data,
                            )
                        }))
                    } else {
                        waker.register(cx.waker());
                        Poll::Pending
                    }
                }
            })
        })
    }
}
impl Drop for PendingTransmission<'_> {
    fn drop(&mut self) {
        self.tx_queue.inner.lock(|rc| {
            rc.borrow_mut().queue_items[self.slot_index()]
                .2
                .store(false, Ordering::Relaxed)
        })
    }
}
/// Provides access to all transmission queues.
///
/// The four EDCA queues, are actual software queues, so transmitting a frame using
/// [Self::transmit_edca] will enqueue the frame for the background task to handle.
/// Transmissions using the software queues require you to pass the [TxBuffer] directly,
/// as they are processed asynchronously.
/// The beacon "queue" is special, as it isn't a software queue, and transmissions
/// using [Self::transmit_beacon] happen in the calling context.
pub struct TxEndpoint<'res> {
    pub(crate) beacon_tx_endpoint: &'res Mutex<NoopRawMutex, TxQueueEndpoint<'static>>,
    /// Access to the TX buffer manager.
    pub(crate) dyn_tx_buffer_manager: &'res TxBufferManager<'res>,
    pub(crate) edca_tx_endpoints: [&'res TxQueue; 4],
    pub(crate) interface: usize,
}
impl<'res> TxEndpoint<'res> {
    /// Allocate a [TxBuffer] from the buffer manager.
    pub fn alloc_tx_buf(&self) -> impl Future<Output = TxBuffer<'res>> + use<'res, '_> {
        self.dyn_tx_buffer_manager.alloc()
    }
    /// Try allocating a [TxBuffer] from the buffer mananger.
    pub fn try_alloc_tx_buf(&self) -> Option<TxBuffer<'res>> {
        self.dyn_tx_buffer_manager.try_alloc()
    }
    /// Transmit using an EDCA queue.
    ///
    /// This will enqueue the frame. The transmission will start, as soon as the background task
    /// picks it up. You can use the [PendingTransmission] to wait for that.
    pub fn transmit_edca(
        &self,
        edca_access_category: EdcaAccessCategory,
        frame: TxBuffer<'res>,
        frame_length: usize,
        plcp_parameters: TxPlcpParameters,
        mac_parameters: TxMacParameters,
        retry_behaviour: RetryBehaviour,
    ) -> PendingTransmission<'res> {
        self.edca_tx_endpoints[edca_access_category.hardware_slot() - 1].enqueue_frame(PendingFrame {
            frame: unsafe { core::mem::transmute::<TxBuffer<'res>, TxBuffer<'static>>(frame) },
            frame_length,
            plcp_parameters,
            mac_parameters,
            interface: self.interface as u8,
            retry_behaviour,
        })
    }
    /// Transmit using the beacon queue.
    ///
    /// The beacon queue is not a software queue, so the transmission will be done in the calling
    /// context.
    pub async fn transmit_beacon(
        &self,
        frame_buf: &mut [u8],
        plcp_parameters: TxPlcpParameters,
        mac_parameters: TxMacParameters,
        retry_behaviour: RetryBehaviour,
    ) -> Result<u8, TxError> {
        self.beacon_tx_endpoint
            .lock()
            .await
            .transmit(
                self.interface,
                &plcp_parameters,
                &mac_parameters,
                retry_behaviour.as_driver_behaviour(),
                frame_buf,
            )
            .await
    }
    /// Transmit using the beacon queue with a hook.
    ///
    /// The transmission will only be attempted once.
    ///
    /// The beacon queue is not a software queue, so the transmission will be done in the calling
    /// context.
    pub async fn transmit_beacon_with_hook(
        &self,
        frame_buf: &mut [u8],
        plcp_parameters: TxPlcpParameters,
        mac_parameters: TxMacParameters,
        hook: impl FnOnce(&mut [u8]),
    ) -> Result<(), TxError> {
        let mut mutex_guard = self.beacon_tx_endpoint.lock().await;
        hook(frame_buf);
        mutex_guard
            .transmit_oneshot(self.interface, &plcp_parameters, &mac_parameters, frame_buf)
            .await
    }
}
