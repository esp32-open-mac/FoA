//! This module implements the background task for the stack.
//!
//! The job of the background runner is to route the received frames to the appropriate interface

use embassy_sync::channel;
use esp_wifi_hal::{BorrowedBuffer, WiFi};

#[cfg(feature = "arc_buffers")]
use crate::RxArcPool;
use crate::{lmac::LMacReceiveEndpoint, ReceivedFrame};

/// The FoA background runner.
pub struct FoARunner<'res> {
    pub(crate) lmac_receive_endpoint: LMacReceiveEndpoint<'res>,
    pub(crate) rx_queue_senders:
        [channel::DynamicSender<'res, ReceivedFrame<'res>>; WiFi::INTERFACE_COUNT],
    #[cfg(feature = "arc_buffers")]
    pub(crate) rx_arc_pool: &'res RxArcPool,
}
impl<'res> FoARunner<'res> {
    #[cfg(feature = "arc_buffers")]
    fn process_packet(&mut self, buffer: BorrowedBuffer<'res>) {
        let Some(rx_arc_buffer) = self.rx_arc_pool.try_alloc(buffer) else {
            return;
        };
        for interface in rx_arc_buffer.interface_iterator() {
            #[allow(clippy::if_same_then_else)]
            if self.rx_queue_senders[interface]
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
        let interface_count = buffer.interface_iterator().count();
        if interface_count != 1 {
            return;
        }
        let interface = buffer.interface_iterator().next().unwrap();
        let _ = self.rx_queue_senders[interface].try_send(buffer);
    }
    /// Run the FoA background task.
    pub async fn run(&mut self) -> ! {
        debug!(
            "FoA MAC runner active with {} interfaces.",
            WiFi::INTERFACE_COUNT
        );
        loop {
            let received = self.lmac_receive_endpoint.receive().await;
            self.process_packet(received);
        }
    }
}
