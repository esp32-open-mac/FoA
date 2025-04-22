//! This module implements the background task for the stack.
//!
//! The job of the background runner is to route the received frames to the appropriate interface

use core::sync::atomic::Ordering;

use embassy_futures::join::{join, join_array};
use embassy_sync::channel;
use esp_wifi_hal::prelude::*;
use futures_util::FutureExt;
use portable_atomic::AtomicBool;

#[cfg(feature = "arc_buffers")]
use crate::RxArcPool;
use crate::{ReceivedFrame, tx_queue::TxQueueRunner};

pub(crate) struct TxRunner<'res> {
    pub(crate) edca_tx_runners: [TxQueueRunner<'res>; 4],
}
impl<'res> TxRunner<'res> {
    fn run(&mut self) -> impl Future<Output = ()> + use<'res, '_> {
        join_array(
            self.edca_tx_runners
                .each_mut()
                .map(|tx_queue_runner| tx_queue_runner.run()),
        )
        .map(|_| ())
    }
}
pub(crate) struct RxRunner<'res> {
    pub(crate) rx_endpoint: AsyncRxEndpoint<'res>,
    pub(crate) rx_queue_senders: [(
        channel::DynamicSender<'res, ReceivedFrame<'res>>,
        &'res AtomicBool,
    ); INTERFACE_COUNT],
    #[cfg(feature = "arc_buffers")]
    pub(crate) rx_arc_pool: &'res RxArcPool,
}
impl<'res> RxRunner<'res> {
    #[cfg(feature = "arc_buffers")]
    fn process_packet(&mut self, buffer: BorrowedBuffer<'res>) {
        let Some(rx_arc_buffer) = self.rx_arc_pool.try_alloc(buffer) else {
            return;
        };
        for interface in rx_arc_buffer.interface_iterator() {
            if !self.rx_queue_senders[interface].1.load(Ordering::Relaxed) {
                continue;
            }
            #[allow(clippy::if_same_then_else)]
            if self.rx_queue_senders[interface]
                .0
                .try_send(rx_arc_buffer.clone())
                .is_err()
            {
                trace!("RX queue for interface {} is full.", interface);
            } else {
                trace!(
                    "Enqueued frame into the RX queue for interface {}.",
                    interface
                );
            }
        }
    }
    #[cfg(not(feature = "arc_buffers"))]
    fn process_packet(&mut self, buffer: BorrowedBuffer<'res>) {
        let mut interface_iterator = buffer.interface_iterator();
        let Some(interface) = interface_iterator.next() else {
            return;
        };
        if interface_iterator.next().is_some() {
            return;
        }
        drop(interface_iterator);
        if self.rx_queue_senders[interface].1.load(Ordering::Relaxed) {
            let _ = self.rx_queue_senders[interface].0.try_send(buffer);
        }
    }
    async fn run(&mut self) -> ! {
        loop {
            let received = self.rx_endpoint.receive().await;
            self.process_packet(received);
        }
    }
}

/// The FoA background runner.
pub struct FoARunner<'res> {
    pub(crate) tx_runner: TxRunner<'res>,
    pub(crate) rx_runner: RxRunner<'res>,
}
impl<'res> FoARunner<'res> {
    /// Run the FoA background task.
    pub fn run(&mut self) -> impl Future<Output = ()> + use<'res, '_> {
        debug!("FoA MAC runner active with {} interfaces.", INTERFACE_COUNT);

        join(self.tx_runner.run(), self.rx_runner.run()).map(|_| ())
    }
}
