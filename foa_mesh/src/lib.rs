#![no_std]

use embassy_net_driver_channel::{self as ch, driver::HardwareAddress, Device as NetDevice};
use esp_config::esp_config_int;
use foa::VirtualInterface;
use rx_router::MeshRxRouter;

mod control;
mod runner;
mod rx_router;

pub use {control::MeshControl, runner::MeshRunner};

pub(crate) const RX_QUEUE_DEPTH: usize = esp_config_int!(usize, "FOA_MESH_CONFIG_RX_QUEUE_DEPTH");
pub(crate) const NET_TX_BUFFERS: usize = esp_config_int!(usize, "FOA_MESH_CONFIG_NET_TX_BUFFERS");
pub(crate) const NET_RX_BUFFERS: usize = esp_config_int!(usize, "FOA_MESH_CONFIG_NET_RX_BUFFERS");

pub const MTU: usize = 1500;
pub type MeshNetDevice<'a> = NetDevice<'a, MTU>;

pub struct MeshResources<'foa> {
    rx_router: MeshRxRouter<'foa>,
    net_state: ch::State<MTU, NET_RX_BUFFERS, NET_TX_BUFFERS>,
}
pub fn new_mesh_interface<'foa, 'vif>(
    resources: &'vif mut MeshResources<'foa>,
    virtual_interface: &'vif mut VirtualInterface<'foa>,
) -> (
    MeshControl<'foa, 'vif>,
    MeshRunner<'foa, 'vif>,
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
        },
        MeshRunner::new(
            net_runner,
            background_endpoint,
            rx_router_input,
            interface_rx_queue_receiver,
            interface_control,
        ),
        net_device,
    )
}
