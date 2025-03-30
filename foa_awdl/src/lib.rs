#![no_std]

use core::net::Ipv6Addr;

use embassy_net_driver_channel::driver::HardwareAddress;
use esp_config::esp_config_int;
use foa::{esp_wifi_hal::RxFilterBank, VirtualInterface};
use ieee80211::mac_parser::MACAddress;
use rand_core::RngCore;
use runner::{AwdlManagementRunner, AwdlMsduTxRunner};

mod control;
mod peer;
mod runner;
mod state;

pub use {control::AwdlControl, runner::AwdlRunner, state::AwdlResources};

const AWDL_BSSID: MACAddress = MACAddress::new([0x00, 0x25, 0x00, 0xff, 0x94, 0x73]);
const APPLE_OUI: [u8; 3] = [0x00, 0x17, 0xf2];

pub(crate) const PEER_CACHE_SIZE: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_PEER_CACHE_SIZE");
pub(crate) const NET_TX_BUFFERS: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_NET_TX_BUFFERS");
pub(crate) const NET_RX_BUFFERS: usize = esp_config_int!(usize, "FOA_AWDL_CONFIG_NET_RX_BUFFERS");

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AwdlError {
    FailedToAcquireChannelLock,
}

pub const AWDL_MTU: usize = 1484;
pub type AwdlNetDevice<'a> = embassy_net_driver_channel::Device<'a, AWDL_MTU>;

pub fn new_awdl_interface<'foa, 'vif, Rng: RngCore + Clone>(
    virtual_interface: &'vif mut VirtualInterface<'foa>,
    resources: &'vif mut AwdlResources,
    rng: Rng,
) -> (
    AwdlControl<'foa, 'vif, Rng>,
    AwdlRunner<'foa, 'vif>,
    AwdlNetDevice<'vif>,
) {
    virtual_interface.reset();
    let (interface_control, rx_queue) = virtual_interface.split();

    interface_control.set_filter_parameters(RxFilterBank::BSSID, *AWDL_BSSID, None);
    interface_control.set_filter_status(RxFilterBank::BSSID, true);

    let (runner, device) = embassy_net_driver_channel::new(
        &mut resources.net_state,
        HardwareAddress::Ethernet(interface_control.get_factory_mac_for_interface()),
    );

    let (state_runner, rx_runner, tx_runner) = runner.split();

    (
        AwdlControl {
            interface_control,
            rng: rng.clone(),
            common_resources: &resources.common_resources,
            channel: 6,
            mac_address: MACAddress::new(interface_control.get_factory_mac_for_interface()),
        },
        AwdlRunner {
            management_runner: AwdlManagementRunner {
                interface_control,
                rx_queue,
                common_resources: &resources.common_resources,
                rx_runner,
            },
            msdu_tx_runner: AwdlMsduTxRunner {
                interface_control,
                tx_runner,
                common_resources: &resources.common_resources,
            },
            common_resources: &resources.common_resources,
            state_runner,
        },
        device,
    )
}
/// Convert a MAC address to a link local IPv6 address.
pub(crate) fn hw_address_to_ipv6(address: MACAddress) -> Ipv6Addr {
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
pub(crate) fn ipv6_to_hw_address(ipv6: &Ipv6Addr) -> MACAddress {
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
