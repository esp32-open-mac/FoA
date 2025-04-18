use core::marker::PhantomData;

use embassy_futures::{
    join::join,
    select::{select3, Either3},
};
use embassy_net::driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel::{RxRunner, StateRunner, TxRunner};
use embassy_time::{Duration, Ticker};
use ethernet::{Ethernet2Frame, Ethernet2Header};
use foa::{
    esp_wifi_hal::{RxFilterBank, TxParameters, WiFiRate},
    LMacInterfaceControl, ReceivedFrame, RxQueueReceiver,
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

use crate::{
    operations::deauth::send_deauth,
    rx_router::{StaRxRouterEndpoint, StaRxRouterInput, StaRxRouterOperation},
    ConnectionInfo, ConnectionState, ConnectionStateTracker, StaTxRx, MTU,
};
pub(crate) struct ConnectionRunner<'foa, 'vif> {
    // Low level RX/TX.
    pub(crate) rx_router_endpoint: StaRxRouterEndpoint<'foa, 'vif>,
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,

    // Upper layer control.
    pub(crate) state_runner: StateRunner<'vif>,
}
impl ConnectionRunner<'_, '_> {
    /// Set the internal state to disconnected.
    fn set_disconnected(&self) {
        self.sta_tx_rx.interface_control.unlock_channel();
        self.sta_tx_rx
            .connection_state
            .signal_state(ConnectionState::Disconnected);
        self.sta_tx_rx.reset_phy_rate();
    }
    /// Handle a deauth frame.
    ///
    /// NOTE: Currently this immediately leads to disconnection.
    fn handle_deauth(&self, deauth: DeauthenticationFrame<'_>) {
        debug!(
            "Received deauthentication frame from {}, reason: {:?}.",
            deauth.header.transmitter_address, deauth.reason
        );
        self.set_disconnected();
    }
    /// Handle a frame arriving on the background queue, during a connection.
    fn handle_bg_rx(&self, buffer: ReceivedFrame<'_>, beacon_timeout: &mut Ticker) {
        let _ = match_frames! {
            buffer.mpdu_buffer(),
            deauth = DeauthenticationFrame => {
                self.handle_deauth(deauth);
            }
            _beacon = BeaconFrame => {
                beacon_timeout.reset();
            }
        };
    }
    /// Run the background task, while connected.
    async fn run_connection(
        &self,
        ConnectionInfo {
            bssid, own_address, ..
        }: ConnectionInfo,
    ) {
        let mut beacon_timeout = Ticker::every(Duration::from_secs(3));
        loop {
            // We wait for one of three things to happen.
            // 1. An off channel request arrives, which we grant immediately and wait for its
            //    completion.
            // 2. A frame to arrive from the background queue.
            // 3. A beacon timeout to occur.
            match select3(
                self.sta_tx_rx
                    .interface_control
                    .wait_for_off_channel_request(),
                self.rx_router_endpoint.receive(),
                beacon_timeout.next(),
            )
            .await
            {
                Either3::First(off_channel_request) => {
                    off_channel_request.grant();
                }
                Either3::Second(buffer) => self.handle_bg_rx(buffer, &mut beacon_timeout),
                Either3::Third(_) => {
                    // Since we assume the network either can't or barely hear us, we use the
                    // lowest PHY rate.
                    if send_deauth(
                        self.sta_tx_rx.interface_control,
                        bssid,
                        own_address,
                        WiFiRate::PhyRate1ML,
                    )
                    .await
                    .is_err()
                    {
                        error!("The TX buffer was too small to transmit a deauth.");
                    }
                    self.set_disconnected();
                    debug!("Disconnected from BSS due to beacon timeout.");
                    return;
                }
            }
        }
    }
    /// Handle the tranmsission of MSDUs.
    async fn run_msdu_tx(tx_runner: &mut TxRunner<'_, MTU>, sta_tx_rx: &StaTxRx<'_, '_>) -> ! {
        loop {
            let msdu = tx_runner.tx_buf().await;
            // We don't want to accidentally transmit a MSDU, while we're not on channel.
            sta_tx_rx
                .interface_control
                .wait_for_off_channel_completion()
                .await;
            let Some(connection_info) = sta_tx_rx.connection_state.connection_info() else {
                continue;
            };
            let Ok(ethernet_frame) = msdu.pread::<Ethernet2Frame>(0) else {
                continue;
            };
            let mut tx_buf = sta_tx_rx.interface_control.alloc_tx_buf().await;
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
                continue;
            };
            let _ = sta_tx_rx
                .interface_control
                .transmit(
                    &mut tx_buf[..written],
                    &TxParameters {
                        rate: sta_tx_rx.phy_rate(),
                        ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
                    },
                    true,
                )
                .await;
            trace!(
                "Transmitted {} bytes to {}",
                msdu.len(),
                ethernet_frame.header.dst
            );
            tx_runner.tx_done();
        }
    }
    /// Run all actual background operations.
    async fn run(&mut self, tx_runner: &mut TxRunner<'_, MTU>) -> ! {
        loop {
            let connection_info = self.sta_tx_rx.connection_state.wait_for_connection().await;
            self.state_runner
                .set_hardware_address(HardwareAddress::Ethernet(*connection_info.own_address));
            self.state_runner.set_link_state(LinkState::Up);
            debug!("Link went up.");
            // At this point, the channel will have been locked, so we'll only receive off channel
            // requests, while we're connected.

            // Run the connection, until we're disconnected.
            select3(
                self.sta_tx_rx.connection_state.wait_for_disconnection(),
                self.run_connection(connection_info),
                Self::run_msdu_tx(tx_runner, self.sta_tx_rx),
            )
            .await;

            // We reset all connection specific parameters here.
            // Unlocking the channel was already done, by any path leading to disconnection.
            self.sta_tx_rx
                .interface_control
                .set_filter_status(RxFilterBank::BSSID, false);
            debug!("Link went down.");
        }
    }
}
pub(crate) struct RoutingRunner<'foa, 'vif> {
    // Low level RX/TX.
    pub(crate) rx_router_input: StaRxRouterInput<'foa, 'vif>,
    pub(crate) interface_rx_queue: &'vif RxQueueReceiver<'foa>,
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,

    // Upper layer control.
    pub(crate) rx_runner: RxRunner<'vif, MTU>,
}
impl RoutingRunner<'_, '_> {
    /// Forward a received data frame to higher layers.
    fn handle_data_rx(
        rx_runner: &mut RxRunner<'_, MTU>,
        data_frame: DataFrame<'_, DataFrameReadPayload<'_>>,
        connection_state: &ConnectionStateTracker,
    ) {
        // (Frostie314159) NOTE: This is extremely ugly.
        let Some(_connection_info) = connection_state.connection_info() else {
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
        // The body of every data frame contains a logical link control (LLC) frame, as specified
        // in IEEE 802.2.
        let Ok(llc_payload) = payload.pread::<SnapLlcFrame>(0) else {
            return;
        };
        // We don't wait on an RX buffer becoming available here, since doing so could stall the
        // routing task.
        let Some(rx_buf) = rx_runner.try_rx_buf() else {
            trace!("Dropping MSDU, because no buffers are available.");
            return;
        };
        // Here we serialize the ethernet frame.
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
        trace!("Received {} bytes from {}", written, source_address);
    }
    /// Run the routing task.
    async fn run(&mut self) -> ! {
        loop {
            let borrowed_buffer = self.interface_rx_queue.receive().await;
            // We create a generic frame, to do matching.
            let Ok(generic_frame) = GenericFrame::new(borrowed_buffer.mpdu_buffer(), false) else {
                continue;
            };
            let address_1 = generic_frame.address_1();
            // Here we toss out frames, where the first address doesn't meet one of these conditions:
            // 1. Is multicast
            // 2. Is the address, with which we're already associated with a BSS.
            // 3. Is the address, with which we're currently associating with a BSS.
            if !address_1.is_multicast() {
                if let Some(own_address) = self
                    .sta_tx_rx
                    .connection_state
                    .connection_info()
                    .map(|connection_info| connection_info.own_address)
                    .or_else(|| {
                        self.rx_router_input
                            .current_operation()
                            .and_then(StaRxRouterOperation::connecting_mac_address)
                    })
                {
                    if own_address != address_1 {
                        continue;
                    }
                }
            }
            // We won't process any frames, while another interface is doing an off channel
            // operation.
            if !self.sta_tx_rx.in_off_channel_operation()
                && self
                    .sta_tx_rx
                    .interface_control
                    .off_channel_operation_in_progress()
            {
                continue;
            }
            // To reduce latency, we process all data frames here directly.
            if let FrameType::Data(_) = generic_frame.frame_control_field().frame_type() {
                let Some(Ok(data_frame)) = generic_frame.parse_to_typed() else {
                    continue;
                };
                // We don't want to process data frames during an off channel operation, since
                // otherwise it would be possible to inject frames on other channels.
                if self.sta_tx_rx.in_off_channel_operation() {
                    continue;
                }
                Self::handle_data_rx(
                    &mut self.rx_runner,
                    data_frame,
                    self.sta_tx_rx.connection_state,
                );
            } else {
                // We ask the RX router, where all other frames should go.
                let _ = self.rx_router_input.route_frame(borrowed_buffer);
            }
        }
    }
}
/// Interface runner for the STA interface.
pub struct StaRunner<'foa, 'vif> {
    pub(crate) tx_runner: TxRunner<'vif, MTU>,
    pub(crate) connection_runner: ConnectionRunner<'foa, 'vif>,
    pub(crate) routing_runner: RoutingRunner<'foa, 'vif>,
}
impl StaRunner<'_, '_> {
    /// Run the station interface.
    pub async fn run(&mut self) -> ! {
        debug!("STA runner active.");
        join(
            self.connection_runner.run(&mut self.tx_runner),
            self.routing_runner.run(),
        )
        .await;
        unreachable!()
    }
}
