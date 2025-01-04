use core::marker::PhantomData;

use embassy_futures::select::{select, select4, Either4};
use embassy_net::driver::{HardwareAddress, LinkState};
use embassy_net_driver_channel::{StateRunner, TxRunner};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Ticker};
use ethernet::Ethernet2Frame;
use foa::{
    esp32_wifi_hal_rs::{BorrowedBuffer, RxFilterBank, TxParameters},
    interface::InterfaceRunner,
    lmac::{LMacInterfaceControl, LMacTransmitEndpoint},
};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, SequenceControl},
    data_frame::{header::DataFrameHeader, DataFrame},
    match_frames,
    mgmt_frame::{BeaconFrame, DeauthenticationFrame},
    scroll::{Pread, Pwrite},
};
use llc::SnapLlcFrame;
use log::debug;

use crate::{ConnectionState, ConnectionStateTracker, DEFAULT_PHY_RATE, MTU};

/// Interface runner for the [StaInterface](crate::StaInterface).
pub struct StaRunner<'res> {
    // Low level RX/TX.
    pub(crate) rx_queue: &'res Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,
    pub(crate) transmit_endpoint: LMacTransmitEndpoint<'res>,
    pub(crate) interface_control: &'res LMacInterfaceControl<'res>,

    // Upper layer control.
    pub(crate) tx_runner: TxRunner<'res, MTU>,
    pub(crate) state_runner: StateRunner<'res>,

    // Connection management.
    pub(crate) connection_state: &'res ConnectionStateTracker,
}
impl StaRunner<'_> {
    /// Set the internal state to disconnected.
    async fn set_disconnected(&self) {
        self.interface_control.unlock_channel().await;
        self.connection_state
            .signal_state(ConnectionState::Disconnected)
            .await;
    }
    /// Transmit a data frame to the AP.
    async fn handle_data_tx(
        buffer: &[u8],
        interface_control: &LMacInterfaceControl<'_>,
        transmit_endpoint: &LMacTransmitEndpoint<'_>,
        connection_state: &ConnectionStateTracker,
    ) {
        let Some(connection_info) = connection_state.connection_info().await else {
            return;
        };
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
        let _ = transmit_endpoint
            .transmit(
                &mut tx_buf[..written],
                &TxParameters {
                    rate: DEFAULT_PHY_RATE,
                    ..interface_control.get_default_tx_parameters()
                },
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
    async fn handle_bg_rx(&mut self, buffer: BorrowedBuffer<'_, '_>, beacon_timeout: &mut Ticker) {
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
                self.rx_queue.receive(),
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
                    Self::handle_data_tx(
                        data,
                        self.interface_control,
                        &self.transmit_endpoint,
                        self.connection_state,
                    )
                    .await;
                    self.tx_runner.tx_done();
                }
                Either4::Fourth(_) => {
                    self.connection_state
                        .signal_state(ConnectionState::Disconnected)
                        .await;
                    self.interface_control.unlock_channel().await;
                    debug!("Disconnected from BSS due to beacon timeout.");
                }
            }
        }
    }
}
impl InterfaceRunner for StaRunner<'_> {
    /// Run the station interface.
    async fn run(&mut self) -> ! {
        debug!("STA runner active.");
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
            self.interface_control.unlock_channel().await;
            self.state_runner.set_link_state(LinkState::Down);
            debug!("Link went down.");
        }
    }
}
