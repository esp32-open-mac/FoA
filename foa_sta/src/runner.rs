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
    util::{operations::deauthenticate, rx_router::RxRouterQueue},
    LMacInterfaceControl, ReceivedFrame, RxQueueReceiver,
};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, FrameType, SequenceControl},
    crypto::{CryptoHeader, MicState},
    data_frame::{
        header::DataFrameHeader, DataFrame, DataFrameReadPayload, PotentiallyWrappedPayload,
    },
    mac_parser::MACAddress,
    match_frames,
    mgmt_frame::{BeaconFrame, DeauthenticationFrame},
    scroll::{Pread, Pwrite},
    GenericFrame,
};
use llc_rs::SnapLlcFrame;

use crate::{
    connection_state::{ConnectionInfo, ConnectionState, DisconnectionReason},
    rx_router::{StaRxRouterEndpoint, StaRxRouterInput, StaRxRouterOperation},
    StaTxRx, MTU,
};
enum ConnectionRxEvent {
    Disconnected(DisconnectionReason),
    BeaconReceived,
}
pub(crate) struct ConnectionRunner<'foa, 'vif> {
    // Low level RX/TX.
    pub(crate) rx_router_endpoint: StaRxRouterEndpoint<'foa, 'vif>,
    pub(crate) sta_tx_rx: &'vif StaTxRx<'foa, 'vif>,

    // Upper layer control.
    pub(crate) state_runner: StateRunner<'vif>,
}
impl ConnectionRunner<'_, '_> {
    /// Handle a deauth frame.
    ///
    /// NOTE: Currently this immediately leads to disconnection.
    fn handle_deauth(&self, deauth: DeauthenticationFrame<'_>) -> ConnectionRxEvent {
        debug!(
            "Received deauthentication frame from {}, reason: {:?}.",
            deauth.header.transmitter_address, deauth.reason
        );
        ConnectionRxEvent::Disconnected(DisconnectionReason::Deauthenticated)
    }
    /// Handle a frame arriving on the background queue, during a connection.
    fn handle_bg_rx(&self, buffer: ReceivedFrame<'_>) -> Option<ConnectionRxEvent> {
        match_frames! {
            buffer.mpdu_buffer(),
            deauth = DeauthenticationFrame => {
                self.handle_deauth(deauth)
            }
            _beacon = BeaconFrame => {
                ConnectionRxEvent::BeaconReceived
            }
        }
        .ok()
    }
    /// Run the background task.
    ///
    /// This will return if we are deauthenticated or a beacon timeout occurs.
    async fn run_connection(
        &self,
        ConnectionInfo {
            bss, own_address, ..
        }: &ConnectionInfo,
    ) -> DisconnectionReason {
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
                    self.sta_tx_rx
                        .interface_control
                        .wait_for_off_channel_completion()
                        .await;
                }
                Either3::Second(buffer) => {
                    if let Some(connection_rx_event) = self.handle_bg_rx(buffer) {
                        match connection_rx_event {
                            ConnectionRxEvent::Disconnected(disconnection_reason) => {
                                return disconnection_reason
                            }
                            ConnectionRxEvent::BeaconReceived => beacon_timeout.reset(),
                        }
                    }
                }
                Either3::Third(_) => {
                    // Since we assume the network can either not or barely hear us, we use the
                    // lowest PHY rate.
                    deauthenticate(
                        self.sta_tx_rx.interface_control,
                        bss.bssid,
                        *own_address,
                        true,
                        WiFiRate::PhyRate1ML,
                    )
                    .await;
                    debug!("Disconnected from BSS due to beacon timeout.");
                    return DisconnectionReason::BeaconTimeout;
                }
            }
        }
    }
    /// Handle the tranmsission of MSDUs.
    async fn run_msdu_tx(
        tx_runner: &mut TxRunner<'_, MTU>,
        sta_tx_rx: &StaTxRx<'_, '_>,
        connection_info: &ConnectionInfo,
    ) -> ! {
        loop {
            let msdu = tx_runner.tx_buf().await;
            // We don't want to accidentally transmit a MSDU, while we're not on channel.
            sta_tx_rx
                .interface_control
                .wait_for_off_channel_completion()
                .await;
            let Ok(ethernet_frame) = msdu.pread::<Ethernet2Frame>(0) else {
                continue;
            };
            let mut tx_buf = sta_tx_rx.interface_control.alloc_tx_buf().await;
            let data_frame = DataFrame {
                header: DataFrameHeader {
                    subtype: DataFrameSubtype::Data,
                    fcf_flags: FCFFlags::new().with_to_ds(true),
                    address_1: connection_info.bss.bssid,
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
            let tx_crypto_info = sta_tx_rx.map_crypto_state(|crypto_state| {
                (
                    crypto_state
                        .security_associations
                        .ptksa
                        .next_packet_number(),
                    crypto_state.security_associations.ptksa.key_id,
                    crypto_state.ptk_key_slot.key_slot(),
                )
            });
            let Some((written, key_slot)) =
                (if let Some((new_packet_number, key_id, key_slot)) = tx_crypto_info {
                    tx_buf
                        .pwrite(
                            data_frame.crypto_wrap(
                                CryptoHeader::new(new_packet_number, key_id).unwrap(),
                                MicState::Short,
                            ),
                            0,
                        )
                        .ok()
                        .map(|written| (written, Some(key_slot)))
                } else {
                    tx_buf.pwrite(data_frame, 0).ok().zip(None)
                })
            else {
                continue;
            };
            let _ = sta_tx_rx
                .interface_control
                .transmit(
                    &mut tx_buf[..written],
                    &TxParameters {
                        rate: sta_tx_rx.phy_rate(),
                        key_slot,
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
            let disconnection_reason = match select3(
                self.sta_tx_rx.connection_state.wait_for_disconnection(),
                self.run_connection(&connection_info),
                Self::run_msdu_tx(tx_runner, self.sta_tx_rx, &connection_info),
            )
            .await
            {
                Either3::First(disconnection_reason) | Either3::Second(disconnection_reason) => {
                    disconnection_reason
                }
                Either3::Third(_) => unreachable!(),
            };
            // We reset all connection specific parameters here.
            // Unlocking the channel was already done, by any path leading to disconnection.
            self.sta_tx_rx.interface_control.unlock_channel();
            self.sta_tx_rx.reset_phy_rate();
            self.sta_tx_rx
                .interface_control
                .set_filter_status(RxFilterBank::BSSID, false);
            self.sta_tx_rx
                .connection_state
                .signal_state(ConnectionState::Disconnected(disconnection_reason));
            self.state_runner.set_link_state(LinkState::Down);
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
    fn process_potentially_wrapped_payload<'a>(
        &self,
        is_group: bool,
        payload: PotentiallyWrappedPayload<DataFrameReadPayload<'a>>,
    ) -> Option<DataFrameReadPayload<'a>> {
        Some(match payload {
            PotentiallyWrappedPayload::Unwrapped(payload) => payload,
            PotentiallyWrappedPayload::CryptoWrapped(crypto_wrapper) => self
                .sta_tx_rx
                .map_crypto_state(|crypto_state| {
                    let security_associations = &crypto_state.security_associations;
                    let packet_number = crypto_wrapper.crypto_header.packet_number();
                    let packet_number_valid = if is_group {
                        security_associations.gtksa.update_and_validate_replay_counter(packet_number)
                    } else {
                        security_associations.ptksa.update_and_validate_replay_counter(packet_number)
                    };
                    if !packet_number_valid {
                        debug!("RX packet number lower than own replay counter.");
                    }
                    packet_number_valid.then_some(crypto_wrapper.payload)
                })
                .flatten()?,
        })
    }
    /// Handover a single MSDU to embassy_net.
    fn handle_downlink_msdu(
        &mut self,
        payload: &[u8],
        source_address: MACAddress,
        destination_address: MACAddress,
    ) -> Option<()> {
        // The body of every data frame contains a logical link control (LLC) frame, as specified
        // in IEEE 802.2.
        let llc_payload = payload.pread::<SnapLlcFrame>(0).ok()?;
        // We don't wait on an RX buffer becoming available here, since doing so could stall the
        // routing task.
        let Some(rx_buf) = self.rx_runner.try_rx_buf() else {
            trace!("Dropping MSDU, because no buffers are available.");
            return None;
        };
        // Here we serialize the ethernet frame.
        let Ok(written) = rx_buf.pwrite(
            Ethernet2Frame {
                header: Ethernet2Header {
                    dst: destination_address,
                    src: source_address,
                    ether_type: llc_payload.ether_type,
                },
                payload: llc_payload.payload,
            },
            0,
        ) else {
            return None;
        };
        self.rx_runner.rx_done(written);
        info!("Received {} bytes from {}", written, source_address);
        Some(())
    }
    /// Forward a received data frame to higher layers.
    fn handle_data_rx(&mut self, data_frame: DataFrame<'_, &[u8]>) -> Option<()> {
        let destination_address = data_frame.header.destination_address()?;
        let source_address = data_frame.header.source_address()?;
        let Some(payload) = self.process_potentially_wrapped_payload(
            destination_address.is_multicast(),
            data_frame.potentially_wrapped_payload(Some(MicState::NotPresent))?,
        ) else {
            info!("Dropping MSDU.");
            return None;
        };
        match payload {
            DataFrameReadPayload::Single(payload) => {
                self.handle_downlink_msdu(payload, *source_address, *destination_address)
            }
            DataFrameReadPayload::AMSDU(mut amsdu_sub_frame_iterator) => amsdu_sub_frame_iterator
                .try_for_each(|sub_frame| {
                    self.handle_downlink_msdu(
                        sub_frame.payload,
                        sub_frame.source_address,
                        sub_frame.destination_address,
                    )
                }),
        }
    }
    fn connecting_mac_address(&self) -> Option<MACAddress> {
        self.rx_router_input
            .operation(RxRouterQueue::Foreground)
            .or_else(|| self.rx_router_input.operation(RxRouterQueue::Background))
            .and_then(StaRxRouterOperation::connecting_mac_address)
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
                    .or_else(|| self.connecting_mac_address())
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
            // To reduce latency, we process all data frames here directly, if we are connected.
            if self.sta_tx_rx.connection_state.connected() {
                if let FrameType::Data(_) = generic_frame.frame_control_field().frame_type() {
                    let Some(Ok(data_frame)) = generic_frame.parse_to_typed() else {
                        continue;
                    };
                    // We don't want to process data frames during an off channel operation, since
                    // otherwise it would be possible to inject frames on other channels.
                    if self.sta_tx_rx.in_off_channel_operation() {
                        continue;
                    }
                    self.handle_data_rx(data_frame);
                    continue;
                }
            }
            // We ask the RX router, where all other frames should go.
            let _ = self.rx_router_input.route_frame(borrowed_buffer);
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
