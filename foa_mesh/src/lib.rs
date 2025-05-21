#![no_std]

mod control;
mod runner;
mod rx_router;
pub mod state;

use embassy_net_driver_channel::{Device as NetDevice, driver::HardwareAddress};
use esp_config::esp_config_int;
use foa::VirtualInterface;
use ieee80211::mac_parser::MACAddress;
use rand_core::RngCore;
use state::{CommonResources, MeshResources};

pub use {control::MeshControl, runner::MeshRunner};

pub(crate) const RX_QUEUE_DEPTH: usize = esp_config_int!(usize, "FOA_MESH_CONFIG_RX_QUEUE_DEPTH");
pub(crate) const NET_TX_BUFFERS: usize = esp_config_int!(usize, "FOA_MESH_CONFIG_NET_TX_BUFFERS");
pub(crate) const NET_RX_BUFFERS: usize = esp_config_int!(usize, "FOA_MESH_CONFIG_NET_RX_BUFFERS");

pub const MTU: usize = 1500;
pub type MeshNetDevice<'a> = NetDevice<'a, MTU>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Errors that can occur with the Mesh interface.
pub enum MeshError {
    FailedToAcquireChannelLock,
}

pub fn new_mesh_interface<'foa, 'vif, Rng: RngCore + Copy>(
    resources: &'vif mut MeshResources<'foa>,
    virtual_interface: &'vif mut VirtualInterface<'foa>,
    channel: u8,
    mesh_id: heapless::String<32>,
    rng: Rng,
) -> (
    MeshControl<'foa, 'vif>,
    MeshRunner<'foa, 'vif, Rng>,
    MeshNetDevice<'vif>,
) {
    virtual_interface.reset();

    let (interface_control, interface_rx_queue_receiver) = virtual_interface.split();
    let (rx_router_input, [foreground_endpoint, background_endpoint]) = resources.rx_router.split();
    let (net_runner, net_device) = embassy_net_driver_channel::new(
        &mut resources.net_state,
        HardwareAddress::Ethernet(interface_control.get_factory_mac_for_interface()),
    );
    (
        MeshControl {
            interface_control,
            rx_router_endpoint: foreground_endpoint,
            common_resources: &resources.common_resources,
            channel,
            mac_address: MACAddress::new(interface_control.get_factory_mac_for_interface()),
            mesh_id,
        },
        MeshRunner::new(
            net_runner,
            background_endpoint,
            rx_router_input,
            interface_rx_queue_receiver,
            interface_control,
            &resources.common_resources,
            rng.clone(),
        ),
        net_device,
    )
}
