#![no_std]
//! This crate implements an Apple Wireless Direct Link (AWDL) interface for FoA.
//!
//! ## What is AWDL?
//! AWDL is a wireless P2P protocol, that allows Apple device to communicate directly. It is a
//! significant improvmenet over IBSS in terms of flexibility and power consumption.
//!
//! For futher information on how AWDL I suggest you take a look at the [website of the owlink
//! project](https://owlink.org/wiki/) and [Milan Stute's dissertation](https://tuprints.ulb.tu-darmstadt.de/11457/1/dissertation_milan-stute_2020.pdf).
//!
//! ## What works
//! - Synchronization
//! - Election
//! - PSF & MIF Transmission
//! - Data TX & RX
//! - Basic Service Discovery
//! - Event Queue
//!
//! ## What doesn't work
//! - Channel Hopping (Because there's no use for it on the regular ESP32)
//! - Off Channel Operations
//! - Coexistence with interfaces on different channels
//!
//! ## Usage
//! Initialize the interface with [foa_awdl::new_awdl_interface](new_awdl_interface), which will
//! give you four things:
//! 1. [AwdlControl] to control the interface
//! 2. [AwdlRunner] to run the background task
//! 3. [AwdlNetDevice] to initialize `embassy_net`
//! 4. [AwdlEventQueueReceiver] to get events from the interface
//!
//! For the interface to work, you have to call [AwdlRunner::run] either in a separate task or join
//! the future with another. You can then use the net device to initialize the IP stack and start
//! the interface with [AwdlControl::start]. Once the interface is running, you can receive events
//! from the event queue, which report status changes from the interface.

use core::net::Ipv6Addr;

use embassy_net_driver_channel::driver::HardwareAddress;
use embassy_sync::channel::DynamicReceiver;
use esp_config::esp_config_int;
use foa::{esp_wifi_hal::RxFilterBank, VirtualInterface};
use ieee80211::mac_parser::MACAddress;
use rand_core::RngCore;

mod control;
mod event;
mod peer;
mod peer_cache;
mod runner;
mod state;

pub use {control::AwdlControl, event::AwdlEvent, runner::AwdlRunner, state::AwdlResources};

const AWDL_BSSID: MACAddress = MACAddress::new([0x00, 0x25, 0x00, 0xff, 0x94, 0x73]);
const APPLE_OUI: [u8; 3] = [0x00, 0x17, 0xf2];

pub(crate) const PEER_CACHE_SIZE: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_PEER_CACHE_SIZE");
pub(crate) const NET_TX_BUFFERS: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_NET_TX_BUFFERS");
pub(crate) const NET_RX_BUFFERS: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_NET_RX_BUFFERS");
pub(crate) const EVENT_QUEUE_DEPTH: usize =
    esp_config_int!(usize, "FOA_AWDL_CONFIG_EVENT_QUEUE_DEPTH");

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Errors that can occur with the AWDL interface.
pub enum AwdlError {
    FailedToAcquireChannelLock,
}

/// The maximum transmission unit of the AWDL interface.
pub const AWDL_MTU: usize = 1484;
/// The net device of the AWDL interface.
pub type AwdlNetDevice<'a> = embassy_net_driver_channel::Device<'a, AWDL_MTU>;
/// Receiver for the event queue of the AWDL interface.
pub type AwdlEventQueueReceiver<'a> = DynamicReceiver<'a, AwdlEvent>;

/// Initialize and AWDL interface.
///
/// You'll get a control interface, a runner, a net device and a receiver for the event queue.
pub fn new_awdl_interface<'foa, 'vif, Rng: RngCore + Clone>(
    virtual_interface: &'vif mut VirtualInterface<'foa>,
    resources: &'vif mut AwdlResources,
    rng: Rng,
) -> (
    AwdlControl<'foa, 'vif, Rng>,
    AwdlRunner<'foa, 'vif>,
    AwdlNetDevice<'vif>,
    AwdlEventQueueReceiver<'vif>,
) {
    virtual_interface.reset();
    let (interface_control, rx_queue) = virtual_interface.split();

    interface_control.set_filter_parameters(RxFilterBank::BSSID, *AWDL_BSSID, None);
    interface_control.set_filter_status(RxFilterBank::BSSID, true);

    let (net_runner, net_device) = embassy_net_driver_channel::new(
        &mut resources.net_state,
        HardwareAddress::Ethernet(interface_control.get_factory_mac_for_interface()),
    );

    (
        AwdlControl {
            interface_control,
            rng,
            common_resources: &resources.common_resources,
            channel: 6,
            mac_address: MACAddress::new(interface_control.get_factory_mac_for_interface()),
        },
        AwdlRunner::new(
            interface_control,
            rx_queue,
            &resources.common_resources,
            net_runner,
        ),
        net_device,
        resources.common_resources.event_queue.dyn_receiver(),
    )
}
/// Convert a MAC address to a link local IPv6 address.
pub fn hw_address_to_ipv6(address: &[u8; 6]) -> Ipv6Addr {
    Ipv6Addr::new(
        0xfe80,
        0x0000,
        0x0000,
        0x0000,
        u16::from_be_bytes([address[0] ^ 0x2, address[1]]),
        u16::from_be_bytes([address[2], 0xff]),
        u16::from_be_bytes([0xfe, address[3]]),
        u16::from_be_bytes([address[4], address[5]]),
    )
}
/// Convert a link local IPv6 address to a MAC address.
pub fn ipv6_to_hw_address(ipv6: &Ipv6Addr) -> MACAddress {
    let ipv6 = ipv6.octets();
    MACAddress::new([
        ipv6[8] ^ 0x2,
        ipv6[9],
        ipv6[10],
        ipv6[13],
        ipv6[14],
        ipv6[15],
    ])
}
