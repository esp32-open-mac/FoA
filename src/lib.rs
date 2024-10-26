#![no_std]

use core::{
    future::{pending, Future},
    marker::PhantomData,
};

use embassy_futures::{
    join::join,
    select::{select, select3, Either, Either3},
};
use embassy_net_driver_channel::{self as ch, driver::LinkState, RxRunner, TxRunner};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp32_wifi_hal_rs::{WiFiError, WiFiRate};
use esp_hal::peripherals::{ADC2, RADIO_CLK, WIFI};
use ether_type::EtherType;
use ethernet::{Ethernet2Frame, Ethernet2Header};
use ieee80211::{
    common::{DataFrameSubtype, FCFFlags, IEEE80211StatusCode, SequenceControl},
    data_frame::{
        builder::DataFrameBuilder, header::DataFrameHeader, DataFrame, DataFrameReadPayload,
    },
    elements::{
        rates::{EncodedRate, ExtendedSupportedRatesElement, SupportedRatesElement},
        DSSSParameterSetElement, ReadElements, SSIDElement,
    },
    extended_supported_rates,
    mac_parser::MACAddress,
    match_frames,
    scroll::{self, Pread, Pwrite},
    supported_rates,
};
use llc::SnapLlcFrame;
use log::debug;
use lower_mac::LowerMAC;

extern crate alloc;
mod lower_mac;
mod sta_control;
pub use sta_control::Control;
use static_cell::StaticCell;

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
}
pub type StaResult<T> = Result<T, StaError>;

async fn timeout<O>(timeout: Duration, f: impl Future<Output = O>) -> StaResult<O> {
    if let Either::First(output) = select(f, Timer::after(timeout)).await {
        Ok(output)
    } else {
        Err(StaError::Timeout)
    }
}
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
const RESPONSE_TIMEOUT: Duration = Duration::from_millis(200);

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

enum ConnectionState {
    Disassociated,
    Associated(MACAddress),
}

pub struct State {
    lower_mac: LowerMAC,
    state_signal: Signal<CriticalSectionRawMutex, ConnectionState>,
}
impl State {
    pub fn new(
        wifi: WIFI,
        radio_clock: RADIO_CLK,
        adc2: ADC2,
        mac_address: Option<MACAddress>,
    ) -> Self {
        Self {
            lower_mac: LowerMAC::new(wifi, radio_clock, adc2, mac_address),
            state_signal: Signal::new(),
        }
    }
}

pub fn new(state: &mut State) -> (Control<'_>, Runner<'_>, NetDriver<'_>) {
    static STACK_RESOURCES: StaticCell<ch::State<MTU, 4, 4>> = StaticCell::new();
    let control = Control::new(state);
    let (ch_runner, device) = ch::new(
        STACK_RESOURCES.init(ch::State::new()),
        ch::driver::HardwareAddress::Ethernet(state.lower_mac.get_mac_address().0),
    );
    let runner = Runner {
        ch: ch_runner,
        state,
    };
    (control, runner, device)
}

const DATA_TX_BUFFER_SIZE: usize = MTU + 30;
pub struct Runner<'a> {
    ch: ch::Runner<'a, MTU>,
    state: &'a State,
}
impl Runner<'_> {
    async fn send_msdu_up(
        &self,
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
        rx_runner.rx_done(written);
        debug!("RX");
    }
    async fn handle_rx(&self, rx_runner: &mut RxRunner<'_, MTU>) {
        loop {
            let received = self.state.lower_mac.receive().await;
            let _ = match_frames! {
                received.mpdu_buffer(),
                data_frame = DataFrame => {
                    if let Some(DataFrameReadPayload::Single(payload)) = data_frame.payload {
                        self.send_msdu_up(rx_runner,  data_frame.header, payload).await;
                    }
                }
            };
        }
    }
    async fn run_after_assoc(
        &self,
        rx_runner: &mut RxRunner<'_, MTU>,
        tx_runner: &mut TxRunner<'_, MTU>,
        data_tx_buffer: &'static mut [u8],
    ) {
        // join(self.handle_rx(rx_runner), pending()).await;
    }
    pub async fn run(self) -> ! {
        debug!("WiFi background runner active.");
        let (state_runner, mut rx_runner, mut tx_runner) = self.ch.split();
        let mut ap_address = None;
        static DATA_TX_BUFFER: StaticCell<[u8; DATA_TX_BUFFER_SIZE]> = StaticCell::new();
        let data_tx_buffer = DATA_TX_BUFFER.init([0x00; DATA_TX_BUFFER_SIZE]);
        loop {
            if ap_address.is_none() {
                if let ConnectionState::Associated(mac_address) =
                    self.state.state_signal.wait().await
                {
                    ap_address = Some(mac_address);
                    state_runner.set_link_state(LinkState::Up);
                } else {
                    continue;
                }
            }
            let host_tx = tx_runner.tx_buf();
            let wifi_rx = async {
                wait_for_frame!(self.state.lower_mac, DataFrame, data_frame => {
                    if let Some(DataFrameReadPayload::Single(payload)) = data_frame.payload {
                        let buf = rx_runner.rx_buf().await;
                        let Ok(llc_payload) = payload.pread::<SnapLlcFrame>(0) else {
                            continue;
                        };
                        let Ok(written) = buf.pwrite(Ethernet2Frame {
                            header: Ethernet2Header {
                                dst: *data_frame.header.destination_address().unwrap(),
                                src: *data_frame.header.source_address().unwrap(),
                                ether_type: llc_payload.ether_type
                            },
                            payload: llc_payload.payload
                        }, 0) else {
                            continue;
                        };
                        rx_runner.rx_done(written);
                        debug!("RX");
                        Some(())
                    } else {
                        None
                    }
                })
                .await
            };
            match select3(host_tx, wifi_rx, self.state.state_signal.wait()).await {
                Either3::First(tx_buf) => {
                    let Ok(ethernet_frame) = tx_buf.pread::<Ethernet2Frame>(0) else {
                        continue;
                    };
                    let data_frame = DataFrame {
                        header: DataFrameHeader {
                            subtype: DataFrameSubtype::Data,
                            fcf_flags: FCFFlags::new().with_to_ds(true),
                            address_1: ap_address.unwrap(),
                            address_2: self.state.lower_mac.get_mac_address(),
                            address_3: ethernet_frame.header.dst,
                            sequence_control: SequenceControl::new().with_sequence_number(
                                self.state.lower_mac.get_and_increase_sequence_number(),
                            ),
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
                    self.state
                        .lower_mac
                        .transmit(
                            data_frame,
                            data_tx_buffer.as_mut_slice(),
                            WiFiRate::PhyRate6M,
                        )
                        .await
                        .unwrap();
                    tx_runner.tx_done();
                    debug!("TX");
                }
                Either3::Second(_) => {}
                Either3::Third(ConnectionState::Disassociated) => {
                    ap_address = None;
                    state_runner.set_link_state(LinkState::Down);
                }
                _ => {}
            }
        }
    }
}
