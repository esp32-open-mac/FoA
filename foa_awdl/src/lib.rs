#![no_std]

use core::cell::Cell;

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Duration;
use esp_config::esp_config_int;
use foa::{esp_wifi_hal::RxFilterBank, VirtualInterface};
use ieee80211::mac_parser::MACAddress;
use peer::StaticAwdlPeerCache;
use rand_core::RngCore;

mod control;
mod peer;
mod runner;

pub use {control::AwdlControl, runner::AwdlRunner};

const AWDL_BSSID: MACAddress = MACAddress::new([0x00, 0x25, 0x00, 0xff, 0x94, 0x73]);
const APPLE_OUI: [u8; 3] = [0x00, 0x17, 0xf2];

pub(crate) const PEER_CACHE_SIZE: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_PEER_CACHE_SIZE");

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum AwdlState {
    Active {
        mac_address: MACAddress,
        channel: u8,
    },
    Inactive,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AwdlError {
    FailedToAcquireChannelLock,
}

pub(crate) struct DynamicSessionParameters {
    pub stale_peer_timeout: Cell<Duration>,
}
impl DynamicSessionParameters {
    pub const fn new() -> Self {
        Self {
            stale_peer_timeout: Cell::new(Duration::from_secs(3)),
        }
    }
}

pub(crate) struct CommonResources {
    state_signal: Signal<NoopRawMutex, AwdlState>,
    peer_cache: StaticAwdlPeerCache,
    dynamic_session_parameters: DynamicSessionParameters,
}
impl CommonResources {
    pub const fn new() -> Self {
        Self {
            state_signal: Signal::new(),
            peer_cache: StaticAwdlPeerCache::new(),
            dynamic_session_parameters: DynamicSessionParameters::new(),
        }
    }
}
pub struct AwdlResources {
    common_resources: CommonResources,
}
impl AwdlResources {
    pub const fn new() -> Self {
        Self {
            common_resources: CommonResources::new(),
        }
    }
}

pub fn new_awdl_interface<'foa, 'vif, Rng: RngCore + Clone>(
    virtual_interface: &'vif mut VirtualInterface<'foa>,
    resources: &'vif mut AwdlResources,
    rng: Rng,
) -> (AwdlControl<'foa, 'vif, Rng>, AwdlRunner<'foa, 'vif, Rng>) {
    virtual_interface.reset();
    let (interface_control, rx_queue) = virtual_interface.split();

    interface_control.set_filter_parameters(RxFilterBank::BSSID, *AWDL_BSSID, None);
    interface_control.set_filter_status(RxFilterBank::BSSID, true);

    (
        AwdlControl {
            interface_control,
            rng: rng.clone(),
            common_resources: &resources.common_resources,
            channel: 6,
            mac_address: MACAddress::new(interface_control.get_factory_mac_for_interface()),
        },
        AwdlRunner {
            interface_control,
            rx_queue,
            rng,
            common_resources: &resources.common_resources,
        },
    )
}
