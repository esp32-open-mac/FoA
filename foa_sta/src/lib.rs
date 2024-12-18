#![no_std]

//! This module implements a station (STA) interface for FoA.
//!
//! ## Usage
//! The STA interface, that's provided consists of a few parts.
//!
//! | struct | purpose |
//! | -- | -- |
//! | [StaControl] | Controlling the STA interface. |
//! | [StaRunner] | The background runner for the STA interface. |
//! | [StaInput] | Entry point for frames addressed to this interface. |
//! | [StaSharedResources] | Shared resources for the STA interface. |
//! | [StaNetDevice] | [Device](ch::Device) for `embassy_net`. |
//!
//! For you the user, only the [StaInterface] and [StaControl] are relevant, since all other types
//! are internal.
//!
//! To use this interface, just call either [foa::new_with_single_interface] or [foa::new_with_multiple_interfaces], with [StaInterface] in the turbofish operator.
//! Which for this interface will return you `(StaControl, StaNetDevice)`
//!
//! ## Status
//! Currently only very basic operations are supported and much of this is work in progress. More
//! operations will follow in the future.
//!
//! ### Supported operations.
//! 1. Passive scanning
//! 2. Connecting to a network.
//! 3. Disconnecting from a network.
//! 4. Setting the MAC address.
use core::{
    array,
    sync::atomic::{AtomicUsize, Ordering},
};

use control::StaControl;
use embassy_net::driver::HardwareAddress;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex, channel::Channel, mutex::Mutex, watch::Watch,
};
use embassy_time::Duration;
use ethernet::{Ethernet2Frame, Ethernet2Header};
use ieee80211::{
    common::{AssociationID, FrameType, IEEE80211StatusCode, ManagementFrameSubtype},
    data_frame::{DataFrame, DataFrameReadPayload},
    mac_parser::MACAddress,
    scroll::{Pread, Pwrite},
    GenericFrame,
};
use llc::SnapLlcFrame;
use log::debug;

use embassy_net_driver_channel::{self as ch, RxRunner};
use foa::{
    esp32_wifi_hal_rs::{BorrowedBuffer, WiFiRate},
    interface::{Interface, InterfaceInput},
    lmac::{LMacError, LMacInterfaceControl, LMacTransmitEndpoint},
};

pub mod control;
mod runner;
pub use runner::StaRunner;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Errors than can occur, during STA operation.
pub enum StaError {
    /// An error occured in the lower MAC.
    LMacError(LMacError),
    /// The scan was unable to find the specified ESS.
    UnableToFindEss,
    /// An operation didn't complete in time.
    Timeout,
    /// Deserializing a received frame failed.
    FrameDeserilisationFailed,
    /// Authentication failed with the specified status code.
    AuthenticationFailure(IEEE80211StatusCode),
    /// Association failed with the specified status code.
    AssociationFailure(IEEE80211StatusCode),
    /// An operation couldn't be carried out, due to an active connection.
    StillConnected,
}

pub(crate) const DEFAULT_TIMEOUT: Duration = Duration::from_millis(200);

pub(crate) const NO_OPERATION: usize = 0;
pub(crate) const SCANNING: usize = 1;
pub(crate) const AUTHENTICATING: usize = 2;
pub(crate) const ASSOCIATING: usize = 3;

pub const MTU: usize = 1514;
pub(crate) const DEFAULT_PHY_RATE: WiFiRate = WiFiRate::PhyRate9M;

/// RX management for the STA interface.
pub(crate) struct StaRxManagement<'res> {
    bg_rx_queue: Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,
    user_operation_status: AtomicUsize,
    user_rx_queue: Channel<NoopRawMutex, BorrowedBuffer<'res, 'res>, 4>,
}
impl StaRxManagement<'_> {
    /// This will set the new user operation status and clear the user RX queue.
    pub fn begin_user_operation(&self, new_status: usize) {
        self.user_operation_status
            .store(new_status, Ordering::Relaxed);
        self.user_rx_queue.clear();
    }
    /// Reset the user operation status.
    pub fn clear_user_operation(&self) {
        self.user_operation_status
            .store(NO_OPERATION, Ordering::Relaxed);
    }
}
impl Default for StaRxManagement<'_> {
    fn default() -> Self {
        Self {
            bg_rx_queue: Channel::new(),
            user_operation_status: AtomicUsize::new(NO_OPERATION),
            user_rx_queue: Channel::new(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Information about the current connection.
pub(crate) struct ConnectionInfo {
    /// The address of the BSS, we're connected to.
    pub bssid: MACAddress,
    /// Our own address.
    pub own_address: MACAddress,
    /// The association ID assigned by the AP.
    pub aid: AssociationID,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// All possible conneciton states, that the STA can be in.
pub(crate) enum ConnectionState {
    Disconnected,
    Connected(ConnectionInfo),
}
/// Tracks the state of the connection and allows awaiting state changes.
struct ConnectionStateMachine {
    state: Mutex<NoopRawMutex, ConnectionState>,
    state_change_watch: Watch<NoopRawMutex, (), 3>,
}
impl ConnectionStateMachine {
    /// Split the state machine into three subscribers.
    ///
    /// They are for [StaControl], [StaRunner] and [StaInput].
    fn split(&mut self) -> [ConnectionStateMachineSubscriber<'_>; 3] {
        array::from_fn(|_| ConnectionStateMachineSubscriber {
            state_machine: self,
        })
    }
}
impl Default for ConnectionStateMachine {
    fn default() -> Self {
        Self {
            state: Mutex::new(ConnectionState::Disconnected),
            state_change_watch: Watch::new(),
        }
    }
}
/// Allows accessing the [ConnectionStateMachine].
pub(crate) struct ConnectionStateMachineSubscriber<'res> {
    state_machine: &'res ConnectionStateMachine,
}
impl ConnectionStateMachineSubscriber<'_> {
    /// Checks if the current conenction state is [ConnectionState::Connected] and returns the
    /// [ConnectionInfo].
    pub async fn is_connected(&self) -> Option<ConnectionInfo> {
        if let ConnectionState::Connected(info) = *self.state_machine.state.lock().await {
            Some(info)
        } else {
            None
        }
    }
    /// Checks if the current connection state is [ConnectionState::Disconnected].
    pub async fn is_disconnected(&self) -> bool {
        *self.state_machine.state.lock().await == ConnectionState::Disconnected
    }
    /// Wait for a state change to occur.
    async fn wait_for_state_change(&self) {
        // We clear this here, so it doesn't fire immediatly.
        self.state_machine.state_change_watch.sender().clear();
        self.state_machine
            .state_change_watch
            .dyn_receiver()
            .unwrap()
            .get()
            .await;
    }
    /// Wait's for a state transition to [ConnectionState::Connected] and returns the
    /// [ConnectionInfo].
    pub async fn wait_for_connection(&self) -> ConnectionInfo {
        if let Some(info) = self.is_connected().await {
            return info;
        }
        self.wait_for_state_change().await;
        self.is_connected().await.unwrap()
    }
    /// Wait's for a state transition to [ConnectionState::Disconnected].
    pub async fn wait_for_disconnection(&self) {
        if !self.is_disconnected().await {
            self.wait_for_state_change().await;
        }
    }
    /// Signal a state transition to all other subscribers.
    pub async fn signal_state_change(&self, state: ConnectionState) {
        *self.state_machine.state.lock().await = state;
        self.state_machine.state_change_watch.sender().send(());
    }
}

/// Shared resources for the [StaInterface].
pub struct StaSharedResources<'res> {
    // RX management.
    rx_management: StaRxManagement<'res>,

    // Networking.
    channel_state: ch::State<MTU, 4, 4>,
    connection_state: ConnectionStateMachine,

    // Misc.
    interface_control: Option<LMacInterfaceControl<'res>>,
}
impl Default for StaSharedResources<'_> {
    fn default() -> Self {
        Self {
            rx_management: StaRxManagement::default(),
            channel_state: ch::State::new(),
            connection_state: ConnectionStateMachine::default(),
            interface_control: None,
        }
    }
}

/// Interface input for the [StaInterface].
pub struct StaInput<'res> {
    rx_management: &'res StaRxManagement<'res>,
    rx_runner: RxRunner<'res, MTU>,
    connection_state_subscriber: ConnectionStateMachineSubscriber<'res>,
}
impl StaInput<'_> {
    /// Forward a received data frame to higher layers.
    async fn handle_data_rx(
        connection_state_subscriber: &ConnectionStateMachineSubscriber<'_>,
        rx_runner: &mut RxRunner<'_, MTU>,
        data_frame: DataFrame<'_, DataFrameReadPayload<'_>>,
    ) {
        if connection_state_subscriber.is_disconnected().await {
            return;
        }
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
                Self::handle_data_rx(
                    &self.connection_state_subscriber,
                    &mut self.rx_runner,
                    data_frame,
                )
                .await;
            }
            // All other frames go to the background runner.
            _ => {
                let _ = self.rx_management.bg_rx_queue.try_send(borrowed_buffer);
            }
        }
    }
}

/// Initialization info for the [StaInterface].
pub struct StaInitInfo;

pub type StaNetDevice<'a> = embassy_net_driver_channel::Device<'a, MTU>;

/// A dummy type for a station interface.
///
/// This is intended to be used, with
/// [foa::new_with_single_interface].
pub struct StaInterface;
impl Interface for StaInterface {
    const NAME: &str = "STA";
    type SharedResourcesType<'res> = StaSharedResources<'res>;
    type ControlType<'res> = (StaControl<'res>, StaNetDevice<'res>);
    type RunnerType<'res> = StaRunner<'res>;
    type InputType<'res> = StaInput<'res>;
    type InitInfo = StaInitInfo;
    async fn new<'res>(
        sta_shared_state: &'res mut Self::SharedResourcesType<'res>,
        _init_info: Self::InitInfo,
        transmit_endpoint: LMacTransmitEndpoint<'res>,
        interface_control: LMacInterfaceControl<'res>,
        mac_address: [u8; 6],
    ) -> (
        Self::ControlType<'res>,
        Self::RunnerType<'res>,
        Self::InputType<'res>,
    ) {
        // Create a reference to the interface control, with the 'res lifetime.
        sta_shared_state.interface_control = Some(interface_control);
        let interface_control = sta_shared_state.interface_control.as_ref().unwrap();

        // Initialize embassy_net.
        let (net_runner, net_device) = ch::new(
            &mut sta_shared_state.channel_state,
            HardwareAddress::Ethernet(mac_address),
        );
        let (state_runner, rx_runner, tx_runner) = net_runner.split();

        let [sta_control_connection_state, sta_runner_connection_state, sta_input_connection_state] =
            sta_shared_state.connection_state.split();

        (
            (
                StaControl {
                    transmit_endpoint,
                    interface_control,
                    mac_address: MACAddress::new(mac_address),
                    rx_management: &sta_shared_state.rx_management,
                    connection_state_subscriber: sta_control_connection_state,
                },
                net_device,
            ),
            StaRunner {
                bg_rx: sta_shared_state.rx_management.bg_rx_queue.dyn_receiver(),
                transmit_endpoint,
                interface_control,
                tx_runner,
                state_runner,
                connection_state_subscriber: Some(sta_runner_connection_state),
            },
            StaInput {
                rx_runner,
                rx_management: &sta_shared_state.rx_management,
                connection_state_subscriber: sta_input_connection_state,
            },
        )
    }
}
