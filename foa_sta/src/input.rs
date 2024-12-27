use core::sync::atomic::Ordering;

use embassy_net_driver_channel::RxRunner;
use ethernet::{Ethernet2Frame, Ethernet2Header};
use foa::{esp32_wifi_hal_rs::BorrowedBuffer, interface::InterfaceInput};
use ieee80211::{
    common::{FrameType, ManagementFrameSubtype},
    data_frame::{DataFrame, DataFrameReadPayload},
    scroll::{Pread, Pwrite},
    GenericFrame,
};
use llc::SnapLlcFrame;
use log::debug;

use crate::{
    ConnectionStateTracker, StaRxManagement, ASSOCIATING, AUTHENTICATING, MTU, NO_OPERATION,
    SCANNING,
};

/// Interface input for the [StaInterface](crate::StaInterface).
pub struct StaInput<'res> {
    pub(crate) rx_management: &'res StaRxManagement<'res>,
    pub(crate) rx_runner: RxRunner<'res, MTU>,
    pub(crate) connection_state: &'res ConnectionStateTracker,
}
impl StaInput<'_> {
    /// Forward a received data frame to higher layers.
    async fn handle_data_rx(
        rx_runner: &mut RxRunner<'_, MTU>,
        data_frame: DataFrame<'_, DataFrameReadPayload<'_>>,
        connection_state: &ConnectionStateTracker,
    ) {
        let Some(_connection_info) = connection_state.connection_info().await else {
            return;
        };
        let Some(destination_address) = data_frame.header.destination_address() else {
            return;
        };
        let Some(source_address) = data_frame.header.source_address() else {
            return;
        };
        let Some(DataFrameReadPayload::Single(payload)) = data_frame.payload else {
            return;
        };
        let Ok(llc_payload) = payload.pread::<SnapLlcFrame>(0) else {
            return;
        };
        let Some(rx_buf) = rx_runner.try_rx_buf() else {
            debug!("Dropping MSDU, because no buffers are available.");
            return;
        };
        let Ok(written) = rx_buf.pwrite(
            Ethernet2Frame {
                header: Ethernet2Header {
                    dst: *destination_address,
                    src: *source_address,
                    ether_type: llc_payload.ether_type,
                },
                payload: llc_payload.payload,
            },
            0,
        ) else {
            return;
        };
        rx_runner.rx_done(written);
        debug!("Received {written} bytes from {}", source_address);
    }
}
impl<'res> InterfaceInput<'res> for StaInput<'res> {
    /// All frames, we receive for this interface arrive through this function.
    async fn interface_input(&mut self, borrowed_buffer: BorrowedBuffer<'res, 'res>) {
        // We create a generic_frame, to do matching.
        let Ok(generic_frame) = GenericFrame::new(borrowed_buffer.mpdu_buffer(), false) else {
            return;
        };
        match (
            generic_frame.frame_control_field().frame_type(),
            self.rx_management
                .user_operation_status
                .load(Ordering::Relaxed),
        ) {
            // If the frame is of interest for the user, we put it in the user queue.
            (FrameType::Management(ManagementFrameSubtype::Authentication), AUTHENTICATING)
            | (FrameType::Management(ManagementFrameSubtype::AssociationResponse), ASSOCIATING)
            | (FrameType::Management(ManagementFrameSubtype::Beacon), SCANNING) => {
                // It is important here, that we only try to send the buffer, since otherwise this
                // will could wait forever and stall the LMAC.
                let _ = self.rx_management.user_rx_queue.try_send(borrowed_buffer);
            }
            // If it's data, we process it here directly.
            (FrameType::Data(_), NO_OPERATION) => {
                let Some(Ok(data_frame)) = generic_frame.parse_to_typed() else {
                    return;
                };
                Self::handle_data_rx(&mut self.rx_runner, data_frame, self.connection_state).await;
            }
            // All other frames go to the background runner.
            _ => {
                let _ = self.rx_management.bg_rx_queue.try_send(borrowed_buffer);
            }
        }
    }
}
