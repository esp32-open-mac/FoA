use core::marker::PhantomData;

use embassy_futures::{
    join::join,
    select::{select, select4, Either4},
};
use embassy_net::driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel::{RxRunner, StateRunner, TxRunner};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, DynamicSender},
};
use embassy_time::{Duration, Ticker};
use ethernet::{Ethernet2Frame, Ethernet2Header};
use foa::{
    esp_wifi_hal::{BorrowedBuffer, RxFilterBank, TxParameters},
    lmac::LMacInterfaceControl,
    ReceivedFrame, RxQueueReceiver,
};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, FrameType, SequenceControl},
    data_frame::{header::DataFrameHeader, DataFrame, DataFrameReadPayload},
    match_frames,
    mgmt_frame::{BeaconFrame, DeauthenticationFrame},
    scroll::{Pread, Pwrite},
    GenericFrame,
};
use llc_rs::SnapLlcFrame;
use log::debug;

use crate::{
    rx_router::{RxQueue, RxRouter},
    ConnectionState, ConnectionStateTracker, DEFAULT_PHY_RATE, MTU,
};

pub(crate) struct ConnectionRunner<'vif, 'foa> {
    // Low level RX/TX.
    pub(crate) bg_rx_queue: &'vif Channel<NoopRawMutex, BorrowedBuffer<'foa>, 4>,
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,

    // Upper layer control.
    pub(crate) tx_runner: TxRunner<'vif, MTU>,
    pub(crate) state_runner: StateRunner<'vif>,

    // Connection management.
    pub(crate) connection_state: &'vif ConnectionStateTracker,
}
impl ConnectionRunner<'_, '_> {
    /// Set the internal state to disconnected.
    async fn set_disconnected(&self) {
        self.interface_control.unlock_channel();
        self.connection_state
            .signal_state(ConnectionState::Disconnected)
            .await;
    }
    /// Transmit a data frame to the AP.
    async fn handle_data_tx(
        buffer: &[u8],
        interface_control: &LMacInterfaceControl<'_>,
        connection_state: &ConnectionStateTracker,
    ) {
        let Some(connection_info) = connection_state.connection_info().await else {
            return;
        };
        let Ok(ethernet_frame) = buffer.pread::<Ethernet2Frame>(0) else {
            return;
        };
        let mut tx_buf = interface_control.alloc_tx_buf().await;
        let data_frame = DataFrame {
            header: DataFrameHeader {
                subtype: DataFrameSubtype::Data,
                fcf_flags: FCFFlags::new().with_to_ds(true),
                address_1: connection_info.bssid,
                address_2: connection_info.own_address,
                address_3: ethernet_frame.header.dst,
                sequence_control: SequenceControl::new(),
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
        let _ = interface_control
            .transmit(
                &mut tx_buf[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
                },
                true,
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
    async fn handle_deauth(&mut self, deauth: DeauthenticationFrame<'_>) {
        debug!(
            "Received deauthentication frame from {}, reason: {:?}.",
            deauth.header.transmitter_address, deauth.reason
        );
        self.set_disconnected().await;
    }
    /// Handle a frame arriving on the background queue, during a connection.
    async fn handle_bg_rx(&mut self, buffer: BorrowedBuffer<'_>, beacon_timeout: &mut Ticker) {
        let _ = match_frames! {
            buffer.mpdu_buffer(),
            deauth = DeauthenticationFrame => {
                self.handle_deauth(deauth).await;
            }
            _beacon = BeaconFrame => {
                beacon_timeout.reset();
            }
        };
    }
    /// Run the background task, while connected.
    async fn run_connection(&mut self) -> ! {
        let mut beacon_timeout = Ticker::every(Duration::from_secs(3));
        loop {
            // We wait for one of three things to happen.
            // 1. An off channel request arrives, which we grant immediately and wait for its
            //    completion.
            // 2. A frame to arrive from the background queue.
            // 3. A frame arriving for TX.
            match select4(
                self.interface_control.wait_for_off_channel_request(),
                self.bg_rx_queue.receive(),
                self.tx_runner.tx_buf(),
                beacon_timeout.next(),
            )
            .await
            {
                Either4::First(off_channel_request) => {
                    off_channel_request.grant();
                    self.interface_control
                        .wait_for_off_channel_completion()
                        .await;
                }
                Either4::Second(buffer) => self.handle_bg_rx(buffer, &mut beacon_timeout).await,
                Either4::Third(data) => {
                    Self::handle_data_tx(data, self.interface_control, self.connection_state).await;
                    self.tx_runner.tx_done();
                }
                Either4::Fourth(_) => {
                    self.connection_state
                        .signal_state(ConnectionState::Disconnected)
                        .await;
                    self.interface_control.unlock_channel();
                    debug!("Disconnected from BSS due to beacon timeout.");
                }
            }
        }
    }
    async fn run(&mut self) -> ! {
        loop {
            let connection_info = self.connection_state.wait_for_connection().await;
            self.state_runner
                .set_hardware_address(HardwareAddress::Ethernet(*connection_info.own_address));
            self.state_runner.set_link_state(LinkState::Up);
            debug!("Link went up.");
            // At this point, the channel will have been locked, so we'll only receive off channel
            // requests, while we're connected.

            // Run the connection, until we're disconnected.
            select(
                self.connection_state.wait_for_disconnection(),
                self.run_connection(),
            )
            .await;

            // We reset all connection specific parameters here.
            self.interface_control
                .set_filter_status(RxFilterBank::BSSID, false);
            self.interface_control.unlock_channel();
            self.state_runner.set_link_state(LinkState::Down);
            debug!("Link went down.");
        }
    }
}
pub(crate) struct RoutingRunner<'vif, 'foa> {
    // Low level RX/TX.
    pub(crate) bg_queue_sender: DynamicSender<'vif, ReceivedFrame<'foa>>,
    pub(crate) interface_rx_queue: &'vif RxQueueReceiver<'foa>,
    pub(crate) user_queue_sender: DynamicSender<'vif, BorrowedBuffer<'foa>>,

    // Upper layer control.
    pub(crate) rx_runner: RxRunner<'vif, MTU>,
    // Connection management.
    pub(crate) connection_state: &'vif ConnectionStateTracker,
    pub(crate) rx_router: &'vif RxRouter,
}
impl RoutingRunner<'_, '_> {
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
    async fn run(&mut self) -> ! {
        loop {
            let borrowed_buffer = self.interface_rx_queue.receive().await;
            // We create a generic_frame, to do matching.
            let Ok(generic_frame) = GenericFrame::new(borrowed_buffer.mpdu_buffer(), false) else {
                continue;
            };
            if let FrameType::Data(_) = generic_frame.frame_control_field().frame_type() {
                let Some(Ok(data_frame)) = generic_frame.parse_to_typed() else {
                    continue;
                };
                Self::handle_data_rx(&mut self.rx_runner, data_frame, self.connection_state).await;
            } else {
                let _ = match self.rx_router.target_queue_for_frame(&generic_frame) {
                    RxQueue::User => &self.user_queue_sender,
                    RxQueue::Background => &self.bg_queue_sender,
                }
                .try_send(borrowed_buffer);
            }
        }
    }
}
/// Interface runner for the [StaInterface](crate::StaInterface).
pub struct StaRunner<'vif, 'foa> {
    pub(crate) connection_runner: ConnectionRunner<'vif, 'foa>,
    pub(crate) routing_runner: RoutingRunner<'vif, 'foa>,
}
impl StaRunner<'_, '_> {
    /// Run the station interface.
    pub async fn run(&mut self) -> ! {
        debug!("STA runner active.");
        join(self.connection_runner.run(), self.routing_runner.run()).await;
        unreachable!()
    }
}
