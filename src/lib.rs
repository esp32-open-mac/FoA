#![no_std]

use core::{marker::PhantomData, mem::discriminant};

use embassy_futures::select::{select, Either};
use embassy_net_driver_channel::{self as ch, driver::LinkState, RxRunner, TxRunner};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::Duration;
use esp32_wifi_hal_rs::{DMAResources, TxErrorBehaviour, WiFiError, WiFiRate};
use esp_hal::peripherals::{ADC2, RADIO_CLK, WIFI};
use ethernet::{Ethernet2Frame, Ethernet2Header};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, IEEE80211StatusCode, SequenceControl},
    data_frame::{header::DataFrameHeader, DataFrame, DataFrameReadPayload},
    elements::{
        rates::{EncodedRate, ExtendedSupportedRatesElement, SupportedRatesElement},
        DSSSParameterSetElement, ReadElements, SSIDElement,
    },
    extended_supported_rates,
    mac_parser::MACAddress,
    match_frames,
    mgmt_frame::DeauthenticationFrame,
    scroll::{self, Pread, Pwrite},
    supported_rates,
};
use llc::SnapLlcFrame;
use log::debug;
use lower_mac::LowerMAC;

extern crate alloc;
mod sta_control;
pub use sta_control::Control;
use static_cell::StaticCell;

#[cfg(feature = "lmac_access")]
pub mod lower_mac;
#[cfg(not(feature = "lmac_access"))]
mod lower_mac;

const LMAC_ACCESSORS_COUNT: usize = 1;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct KnownESS {
    pub ssid: heapless::String<32>,
    pub channel: u8,
    pub bssid: MACAddress,
}
impl KnownESS {
    pub fn from_elements(bssid: MACAddress, elements: ReadElements, current_channel: u8) -> Self {
        let mut ssid = heapless::String::new();
        let _ = ssid.push_str(
            elements
                .get_first_element::<SSIDElement>()
                .unwrap_or_default()
                .ssid(),
        );
        let channel = elements
            .get_first_element::<DSSSParameterSetElement>()
            .map(|dsss_parameter_set| dsss_parameter_set.current_channel)
            .unwrap_or(current_channel);
        Self {
            ssid,
            channel,
            bssid,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ScanMode<'a> {
    Sweep,
    Custom(&'a [u8]),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanConfig<'a> {
    pub scan_mode: ScanMode<'a>,
    pub channel_remain_time: Duration,
}
impl Default for ScanConfig<'_> {
    fn default() -> Self {
        Self {
            scan_mode: ScanMode::Sweep,
            channel_remain_time: Duration::from_millis(400),
        }
    }
}
#[derive(Debug)]
pub enum StaError {
    MACError(WiFiError),
    SerializationError(scroll::Error),
    Timeout,
    UnableToFindESS,
    AuthFailure(IEEE80211StatusCode),
    AssocFailure(IEEE80211StatusCode),
    NotConnected,
    AlreadyConnected,
}
pub type StaResult<T> = Result<T, StaError>;

const DEFAULT_SUPPORTED_RATES: SupportedRatesElement<[EncodedRate; 8]> = supported_rates![
    5.5 B,
    11 B,
    1 B,
    2 B,
    6,
    12,
    24,
    48
];
const DEFAULT_XRATES: ExtendedSupportedRatesElement<[EncodedRate; 4]> =
    extended_supported_rates![54, 9, 18, 36];
const RESPONSE_TIMEOUT: Duration = Duration::from_millis(400);

#[macro_export]
macro_rules! wait_for_frame {
    ($wifi:expr, $frame_type:ty, $binding:ident => $b:block) => {
        async {
            loop {
                let received = $wifi.receive().await;
                let Ok(generic_frame) = ieee80211::GenericFrame::new(received.mpdu_buffer(), false)
                else {
                    continue;
                };
                if let Some(Ok($binding)) = generic_frame.parse_to_typed::<$frame_type>() {
                    match $b {
                        Some(output) => break output,
                        None => continue,
                    }
                }
            }
        }
    };
}

const MTU: usize = 1514;

pub type NetDriver<'a> = ch::Device<'a, 1514>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AssociationState {
    ap_address: MACAddress,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum ConnectionState {
    Disconnected,
    Connected(AssociationState),
}

pub(crate) struct ConnectionStateMachine {
    connection_state: Mutex<CriticalSectionRawMutex, ConnectionState>,
    state_change_signal: Signal<CriticalSectionRawMutex, ()>,
}
impl ConnectionStateMachine {
    pub const fn new() -> Self {
        Self {
            connection_state: Mutex::new(ConnectionState::Disconnected),
            state_change_signal: Signal::new(),
        }
    }
    pub async fn is_connected(&self) -> bool {
        matches!(
            *self.connection_state.lock().await,
            ConnectionState::Connected(_)
        )
    }
    async fn wait_for_state_change(&self) {
        self.state_change_signal.wait().await;
        self.state_change_signal.reset();
    }
    pub async fn wait_for_connection(&self) -> AssociationState {
        if let ConnectionState::Connected(assoc_state) = *self.connection_state.lock().await {
            return assoc_state;
        }
        self.wait_for_state_change().await;
        match *self.connection_state.lock().await {
            ConnectionState::Connected(assoc_state) => assoc_state,
            ConnectionState::Disconnected => unreachable!("This shouldn't happen."),
        }
    }
    pub async fn wait_for_disconnection(&self) {
        if self.is_connected().await {
            self.wait_for_state_change().await;
        }
    }
    pub async fn get_association_state(&self) -> Option<AssociationState> {
        match *self.connection_state.lock().await {
            ConnectionState::Connected(assoc_state) => Some(assoc_state),
            ConnectionState::Disconnected => None,
        }
    }
    pub async fn set_state(&self, new_connection_state: ConnectionState) -> bool {
        let mut current_connection_state = self.connection_state.lock().await;
        if discriminant(&*current_connection_state) == discriminant(&new_connection_state) {
            return false;
        }
        *current_connection_state = new_connection_state;
        self.state_change_signal.signal(());
        true
    }
}

/// The resources required by the WiFi stack.
pub struct StackResources {
    ch: ch::State<MTU, 4, 4>,
    dma_resources: DMAResources<1600, 10>,
    connection_state_machine: ConnectionStateMachine,
}
impl StackResources {
    pub const fn new() -> Self {
        Self {
            ch: ch::State::new(),
            dma_resources: DMAResources::new(),
            connection_state_machine: ConnectionStateMachine::new(),
        }
    }
}
impl Default for StackResources {
    fn default() -> Self {
        Self::new()
    }
}

pub fn new(
    state: &'static mut StackResources,
    wifi: WIFI,
    radio_clock: RADIO_CLK,
    adc2: ADC2,
    mac_address: Option<MACAddress>,
) -> (Control<'_>, Runner<'_>, NetDriver<'_>) {
    let lower_mac = mk_static!(
        LowerMAC,
        LowerMAC::new(
            wifi,
            radio_clock,
            adc2,
            mac_address,
            &mut state.dma_resources,
        )
    );
    let control = Control::new(lower_mac, &state.connection_state_machine);
    let (ch_runner, device) = ch::new(
        &mut state.ch,
        ch::driver::HardwareAddress::Ethernet(lower_mac.get_mac_address().0),
    );
    let runner = Runner {
        ch: ch_runner,
        lower_mac,
        connection_state_machine: &state.connection_state_machine,
    };
    (control, runner, device)
}

const DATA_TX_BUFFER_SIZE: usize = MTU + 30;
pub struct Runner<'a> {
    ch: ch::Runner<'a, MTU>,
    lower_mac: &'a LowerMAC,
    connection_state_machine: &'a ConnectionStateMachine,
}
impl Runner<'_> {
    async fn send_msdu_up(
        rx_runner: &mut RxRunner<'_, MTU>,
        header: DataFrameHeader,
        payload: &[u8],
    ) {
        let rx_buf = rx_runner.rx_buf().await;
        let Ok(llc_payload) = payload.pread::<SnapLlcFrame>(0) else {
            return;
        };
        let Ok(written) = rx_buf.pwrite(
            Ethernet2Frame {
                header: Ethernet2Header {
                    dst: *header.destination_address().unwrap(),
                    src: *header.source_address().unwrap(),
                    ether_type: llc_payload.ether_type,
                },
                payload: llc_payload.payload,
            },
            0,
        ) else {
            return;
        };
        debug!(
            "Received {} bytes of payload from {}",
            llc_payload.payload.len(),
            header.transmitter_address()
        );
        rx_runner.rx_done(written);
    }
    async fn handle_rx(
        lower_mac: &LowerMAC,
        connection_state_machine: &ConnectionStateMachine,
        rx_runner: &mut RxRunner<'_, MTU>,
    ) {
        let received = lower_mac.receive().await;
        let _ = match_frames! {
            received.mpdu_buffer(),
            data_frame = DataFrame => {
                if let Some(DataFrameReadPayload::Single(payload)) = data_frame.payload {
                    Self::send_msdu_up(rx_runner,  data_frame.header, payload).await;
                }
            }
            deauthentication_frame = DeauthenticationFrame => {
                debug!(
                    "Received deauthentication from {} with reason {:?}.",
                    deauthentication_frame.header.transmitter_address,
                    deauthentication_frame.body.reason
                );
                connection_state_machine.set_state(ConnectionState::Disconnected).await;
            }
        };
    }
    async fn handle_tx(
        lower_mac: &LowerMAC,
        association_state: &AssociationState,
        tx_buf: &mut [u8],
        data_tx_buffer: &mut [u8],
    ) {
        let Ok(ethernet_frame) = tx_buf.pread::<Ethernet2Frame>(0) else {
            return;
        };
        let data_frame = DataFrame {
            header: DataFrameHeader {
                subtype: DataFrameSubtype::Data,
                fcf_flags: FCFFlags::new().with_to_ds(true),
                address_1: association_state.ap_address,
                address_2: lower_mac.get_mac_address(),
                address_3: ethernet_frame.header.dst,
                sequence_control: SequenceControl::new()
                    .with_sequence_number(lower_mac.get_and_increase_sequence_number()),
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
        let ra = data_frame.header.receiver_address();
        match lower_mac
            .transmit(
                data_frame,
                data_tx_buffer,
                WiFiRate::PhyRateMCS0LGI,
                TxErrorBehaviour::RetryUntil(4),
            )
            .await
        {
            Ok(sent) => {
                debug!("Transmitted {sent} bytes to {ra}.");
            }
            Err(_) => {
                debug!("Dropping MSDU, due to repeated TX failure.");
            }
        }
    }
    async fn run_after_assoc(
        lower_mac: &LowerMAC,
        connection_state_machine: &ConnectionStateMachine,
        association_state: &AssociationState,
        rx_runner: &mut RxRunner<'_, MTU>,
        tx_runner: &mut TxRunner<'_, MTU>,
        data_tx_buffer: &mut [u8],
    ) {
        loop {
            match select(
                tx_runner.tx_buf(),
                Self::handle_rx(lower_mac, connection_state_machine, rx_runner),
            )
            .await
            {
                Either::First(tx_buf) => {
                    Self::handle_tx(lower_mac, association_state, tx_buf, data_tx_buffer).await;
                    tx_runner.tx_done();
                }
                Either::Second(_) => {}
            }
        }
    }
    pub async fn run(self) -> ! {
        debug!("WiFi background runner active.");
        let (state_runner, mut rx_runner, mut tx_runner) = self.ch.split();
        static DATA_TX_BUFFER: StaticCell<[u8; DATA_TX_BUFFER_SIZE]> = StaticCell::new();
        let data_tx_buffer = DATA_TX_BUFFER.init([0x00; DATA_TX_BUFFER_SIZE]);
        loop {
            let association_state = self.connection_state_machine.wait_for_connection().await;
            state_runner.set_link_state(LinkState::Up);
            debug!("Link went up.");
            select(
                Self::run_after_assoc(
                    self.lower_mac,
                    self.connection_state_machine,
                    &association_state,
                    &mut rx_runner,
                    &mut tx_runner,
                    data_tx_buffer.as_mut_slice(),
                ),
                self.connection_state_machine.wait_for_disconnection(),
            )
            .await;
            state_runner.set_link_state(LinkState::Down);
            debug!("Link went down.");
        }
    }
}
