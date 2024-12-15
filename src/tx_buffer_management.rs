//! This module implements buffer management for TX.
//!
//! You can use [LMacTransmitEndpoint::alloc_tx_buf](crate::lmac::LMacTransmitEndpoint::alloc_tx_buf) to wait for a TX buffer to become available.
use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{self, Channel, DynamicReceiver, DynamicSender},
};

/// A TX buffer borrowed from the TX buffer manager.
///
/// When dropped, this will automatically return the buffer to the TX buffer manager it was
/// borrowed from.
/// WARNING:
/// You must not [core::mem::forget] a [TxBuffer], since this will cause it to never be returned to
/// the TX buffer manager.
#[clippy::has_significant_drop]
pub struct TxBuffer<'res> {
    /// A pointer to the buffer taken from the buffer_queue.
    /// # SAFETY:
    /// When initialising the [TxBufferManager] we acquire a reference to the buffer, this points
    /// to, which lives for the duration, that the [TxBufferManager] lives. We know, that every
    /// [TxBuffer] will have to outlive the [TxBufferManager], due to the sender having a reference
    /// to it. Due to this, the pointer can never be dangling.
    /// The buffer queue also ensures, that only one [TxBuffer] can point to a certain buffer, so
    /// no race conditions can occur.
    buffer: *mut [u8],
    /// A sender to the buffer queue.
    sender: channel::DynamicSender<'res, *mut [u8]>,
}
impl<'res> Deref for TxBuffer<'res> {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        unsafe { self.buffer.as_ref() }.unwrap()
    }
}
impl<'res> DerefMut for TxBuffer<'res> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.buffer.as_mut() }.unwrap()
    }
}
impl<'res> Drop for TxBuffer<'res> {
    fn drop(&mut self) {
        // We ignore the result here, since this can't fail, because we previously took this buffer
        // out from the queue, so the [free_capacity](channel::Channel::free_capacity) is always
        // equal to the number of [TxBuffer]s in existence.
        let _ = self.sender.try_send(self.buffer);
    }
}

/// A dynamic [TxBufferManager], with generics elided.
#[derive(Clone, Copy)]
pub(crate) struct DynTxBufferManager<'res> {
    buffer_sender: DynamicSender<'res, *mut [u8]>,
    buffer_receiver: DynamicReceiver<'res, *mut [u8]>,
}
impl<'res> DynTxBufferManager<'res> {
    /// Allocate a new [TxBuffer].
    ///
    /// This will wait for a new buffer to become available from the buffer queue and can't fail.
    pub async fn alloc(&self) -> TxBuffer<'res> {
        TxBuffer {
            buffer: self.buffer_receiver.receive().await,
            sender: self.buffer_sender,
        }
    }
}

/// A struct managing the allocation of [TxBuffer]s from a pre-allocated slab of memory.
pub(crate) struct TxBufferManager<'res, const TX_BUFFER_SIZE: usize, const TX_BUFFER_COUNT: usize> {
    buffer_queue: channel::Channel<NoopRawMutex, *mut [u8], TX_BUFFER_COUNT>,
    _phantom: PhantomData<&'res ()>,
}
impl<'res, const TX_BUFFER_SIZE: usize, const TX_BUFFER_COUNT: usize>
    TxBufferManager<'res, TX_BUFFER_SIZE, TX_BUFFER_COUNT>
{
    /// Create a new [TxBufferManager], with the provided buffers.
    pub fn new(buffers: &'res mut [[u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT]) -> Self {
        let buffer_queue = Channel::new();

        for buffer in buffers.iter_mut() {
            let _ = buffer_queue.try_send(buffer.as_mut_slice() as *mut _);
        }

        Self {
            buffer_queue,
            _phantom: PhantomData,
        }
    }
    /// Acquire a [DynTxBufferManager].
    pub fn dyn_tx_buffer_manager(&'res self) -> DynTxBufferManager<'res> {
        DynTxBufferManager {
            buffer_sender: self.buffer_queue.dyn_sender(),
            buffer_receiver: self.buffer_queue.dyn_receiver(),
        }
    }
}
