//! This module implements buffer management for TX.
//!
//! You can use [LMacTransmitEndpoint::alloc_tx_buf](crate::lmac::LMacInterfaceControl::alloc_tx_buf) to wait for a TX buffer to become available.
use core::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
    ptr::NonNull,
};

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use futures_util::FutureExt;

use crate::{TX_BUFFER_COUNT, TX_BUFFER_SIZE};

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
    buffer: NonNull<[u8; TX_BUFFER_SIZE]>,
    /// A sender to the buffer queue.
    buffer_queue: &'res Channel<NoopRawMutex, NonNull<[u8; TX_BUFFER_SIZE]>, TX_BUFFER_COUNT>,
}
impl Deref for TxBuffer<'_> {
    type Target = [u8; TX_BUFFER_SIZE];
    fn deref(&self) -> &Self::Target {
        unsafe { self.buffer.as_ref() }
    }
}
impl DerefMut for TxBuffer<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { self.buffer.as_mut() }
    }
}
impl Drop for TxBuffer<'_> {
    fn drop(&mut self) {
        self.fill(0);
        // We ignore the result here, since this can't fail, because we previously took this buffer
        // out from the queue, so the [free_capacity](channel::Channel::free_capacity) is always
        // equal to the number of [TxBuffer]s in existence.
        let _ = self.buffer_queue.try_send(self.buffer);
    }
}

/// A struct managing the allocation of [TxBuffer]s from a pre-allocated slab of memory.
pub(crate) struct TxBufferManager<'res> {
    buffer_queue: Channel<NoopRawMutex, NonNull<[u8; TX_BUFFER_SIZE]>, TX_BUFFER_COUNT>,
    _phantom: PhantomData<&'res ()>,
}
impl<'res> TxBufferManager<'res> {
    /// Create a new [TxBufferManager], with the provided buffers.
    pub fn new(buffers: &'res mut [[u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT]) -> Self {
        let buffer_queue = Channel::new();

        for buffer in buffers {
            let _ = buffer_queue.try_send(NonNull::from(buffer));
        }

        Self {
            buffer_queue,
            _phantom: PhantomData,
        }
    }
    /// Allocate a [TxBuffer].
    ///
    /// This will wait for a new buffer to become available from the buffer queue and can't fail.
    pub fn alloc(&self) -> impl Future<Output = TxBuffer<'_>> + use<'_> {
        self.buffer_queue.receive().map(|buffer| TxBuffer {
            buffer,
            buffer_queue: &self.buffer_queue,
        })
    }
    /// Try allocating a [TxBuffer].
    pub fn try_alloc(&self) -> Option<TxBuffer<'_>> {
        self.buffer_queue.try_receive().ok().map(|buffer| TxBuffer {
            buffer,
            buffer_queue: &self.buffer_queue,
        })
    }
}
