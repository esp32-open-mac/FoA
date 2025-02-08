//! This module implements the background task for the stack.
//!
//! The job of the background runner is to route the received frames to the appropriate interface

use embassy_sync::channel;
use esp_wifi_hal::WiFi;
use log::{debug, error};

use crate::{lmac::LMacReceiveEndpoint, ReceivedFrame};

/// The FoA background runner.
pub struct FoARunner<'res> {
    pub(crate) lmac_receive_endpoint: LMacReceiveEndpoint<'res>,
    pub(crate) rx_queue_senders:
        [channel::DynamicSender<'res, ReceivedFrame<'res>>; WiFi::INTERFACE_COUNT],
}
impl FoARunner<'_> {
    /// Run the FoA background task.
    pub async fn run(&mut self) -> ! {
        debug!(
            "FoA MAC runner active with {} interfaces.",
            WiFi::INTERFACE_COUNT
        );
        loop {
            let received = self.lmac_receive_endpoint.receive().await;
            // We can't process frames for multiple interfaces yet.
            if received.interface_iterator().count() == 1 {
                let interface = received.interface_iterator().next().unwrap();
                let _ = self.rx_queue_senders[interface].try_send(received);
            } else {
                error!("Frame arrived for multiple interfaces.");
            }
        }
    }
}
