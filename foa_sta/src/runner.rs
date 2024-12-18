use core::marker::PhantomData;

use embassy_futures::select::{select, select3, Either3};
use embassy_net::driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel::{StateRunner, TxRunner};
use embassy_sync::channel;
use ethernet::Ethernet2Frame;
use foa::{
    esp32_wifi_hal_rs::{BorrowedBuffer, RxFilterBank, TxErrorBehaviour},
    interface::InterfaceRunner,
    lmac::{LMacInterfaceControl, LMacTransmitEndpoint},
};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, SequenceControl},
    data_frame::{header::DataFrameHeader, DataFrame},
    match_frames,
    mgmt_frame::DeauthenticationFrame,
    scroll::{Pread, Pwrite},
};
use llc::SnapLlcFrame;
use log::debug;

use crate::{
    ConnectionInfo, ConnectionState, ConnectionStateMachineSubscriber, DEFAULT_PHY_RATE, MTU,
};

/// Interface runner for the [StaInterface](crate::StaInterface).
pub struct StaRunner<'res> {
    // Low level RX/TX.
    pub(crate) bg_rx: channel::DynamicReceiver<'res, BorrowedBuffer<'res, 'res>>,
    pub(crate) transmit_endpoint: LMacTransmitEndpoint<'res>,
    pub(crate) interface_control: &'res LMacInterfaceControl<'res>,

    // Upper layer control.
    pub(crate) tx_runner: TxRunner<'res, MTU>,
    pub(crate) state_runner: StateRunner<'res>,

    // Connection management.
    pub(crate) connection_state_subscriber: Option<ConnectionStateMachineSubscriber<'res>>,
}
impl StaRunner<'_> {
    /// Transmit a data frame to the AP.
    async fn handle_data_tx(
        buffer: &[u8],
        interface_control: &LMacInterfaceControl<'_>,
        transmit_endpoint: &LMacTransmitEndpoint<'_>,
        connection_info: &ConnectionInfo,
    ) {
        let Ok(ethernet_frame) = buffer.pread::<Ethernet2Frame>(0) else {
            return;
        };
        let mut tx_buf = transmit_endpoint.alloc_tx_buf().await;
        let data_frame = DataFrame {
            header: DataFrameHeader {
                subtype: DataFrameSubtype::Data,
                fcf_flags: FCFFlags::new().with_to_ds(true),
                address_1: connection_info.bssid,
                address_2: connection_info.own_address,
                address_3: ethernet_frame.header.dst,
                sequence_control: SequenceControl::new()
                    .with_sequence_number(interface_control.get_and_increase_sequence_number()),
                ..Default::default()
            },
            payload: Some(SnapLlcFrame {
                oui: [0x00; 3],
                ether_type: ethernet_frame.header.ether_type,
                payload: ethernet_frame.payload,
                _phantom: PhantomData,
            }),
            _phantom: PhantomData,
        };
        let Ok(written) = tx_buf.pwrite(data_frame, 0) else {
            return;
        };
        let _ = transmit_endpoint
            .transmit(
                &tx_buf[..written + 4],
                DEFAULT_PHY_RATE,
                TxErrorBehaviour::Drop,
            )
            .await;
        debug!(
            "Transmitted {} bytes to {}",
            buffer.len(),
            ethernet_frame.header.dst
        );
    }
    /// Handle a deauth frame.
    ///
    /// NOTE: Currently this immediately leads to disconnection.
    async fn handle_deauth(
        &mut self,
        deauth: DeauthenticationFrame<'_>,
        connection_state_subscriber: &ConnectionStateMachineSubscriber<'_>,
    ) {
        debug!(
            "Received deauthentication frame from {}, reason: {:?}.",
            deauth.header.transmitter_address, deauth.reason
        );
        connection_state_subscriber
            .signal_state_change(ConnectionState::Disconnected)
            .await;
    }
    /// Handle a frame arriving on the background queue, during a connection.
    async fn handle_bg_rx(
        &mut self,
        buffer: BorrowedBuffer<'_, '_>,
        connection_state_subscriber: &ConnectionStateMachineSubscriber<'_>,
    ) {
        let _ = match_frames! {
            buffer.mpdu_buffer(),
            deauth = DeauthenticationFrame => {
                self.handle_deauth(deauth, connection_state_subscriber).await;
            }
        };
    }
    /// Run the background task, while connected.
    async fn run_connection(
        &mut self,
        connection_info: &ConnectionInfo,
        connection_state_subscriber: &ConnectionStateMachineSubscriber<'_>,
    ) -> ! {
        loop {
            // We wait for one of three things to happen.
            // 1. An off channel request arrives, which we grant immediately and wait for its
            //    completion.
            // 2. A frame to arrive from the backrground queue.
            // 3. A frame arriving for TX.
            match select3(
                self.interface_control.wait_for_off_channel_request(),
                self.bg_rx.receive(),
                self.tx_runner.tx_buf(),
            )
            .await
            {
                Either3::First(off_channel_request) => {
                    off_channel_request.grant();
                    self.interface_control
                        .wait_for_off_channel_completion()
                        .await;
                }
                Either3::Second(buffer) => {
                    self.handle_bg_rx(buffer, connection_state_subscriber).await
                }
                Either3::Third(data) => {
                    Self::handle_data_tx(
                        data,
                        self.interface_control,
                        &self.transmit_endpoint,
                        connection_info,
                    )
                    .await;
                    self.tx_runner.tx_done();
                }
            }
        }
    }
}
impl InterfaceRunner for StaRunner<'_> {
    /// Run the station interface.
    async fn run(&mut self) -> ! {
        debug!("STA runner active.");
        let connection_state_subscriber = self.connection_state_subscriber.take().unwrap();
        loop {
            let connection_info = connection_state_subscriber.wait_for_connection().await;
            self.state_runner
                .set_hardware_address(HardwareAddress::Ethernet(*connection_info.own_address));
            self.state_runner.set_link_state(LinkState::Up);
            debug!("Link went up.");
            // At this point, the channel will have been locked, so we'll only receive off channel
            // requests, while we're connected.

            // Run the connection, until we're disconnected.
            select(
                connection_state_subscriber.wait_for_disconnection(),
                self.run_connection(&connection_info, &connection_state_subscriber),
            )
            .await;

            // We reset all connection specific parameters here.
            self.interface_control
                .set_filter_status(RxFilterBank::BSSID, false);
            self.interface_control.unlock_channel().await;
            self.state_runner.set_link_state(LinkState::Down);
            debug!("Link went down.");
        }
    }
}
