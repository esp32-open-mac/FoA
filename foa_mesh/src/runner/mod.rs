use defmt_or_log::debug;
use embassy_futures::select::select3;
use embassy_net_driver_channel::Runner as NetRunner;

use foa::{LMacInterfaceControl, RxQueueReceiver};
use management::MeshManagementRunner;
use rand_core::RngCore;
use rx::MeshRxRunner;
use tx::MeshTxRunner;

use crate::{
    MTU,
    rx_router::{MeshRxRouterEndpoint, MeshRxRouterInput},
    state::{CommonResources, MeshState},
};

mod management;
mod rx;
mod tx;

pub struct MeshRunner<'foa, 'vif, Rng: RngCore + Copy> {
    pub(crate) management_runner: MeshManagementRunner<'foa, 'vif, Rng>,
    pub(crate) tx_runner: MeshTxRunner<'foa, 'vif>,
    pub(crate) rx_runner: MeshRxRunner<'foa, 'vif>,
    pub(crate) common_resources: &'vif CommonResources,
}
impl<'foa, 'vif, Rng: RngCore + Copy> MeshRunner<'foa, 'vif, Rng> {
    pub(crate) fn new(
        net_runner: NetRunner<'vif, MTU>,
        background_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
        rx_router_input: MeshRxRouterInput<'foa, 'vif>,
        interface_rx_queue_receiver: &'vif RxQueueReceiver<'foa>,
        interface_control: &'vif LMacInterfaceControl<'foa>,
        common_resources: &'vif CommonResources,
        rng: Rng,
    ) -> Self {
        let (net_state_runner, net_rx_runner, net_tx_runner) = net_runner.split();
        Self {
            management_runner: MeshManagementRunner {
                interface_control,
                rx_router_endpoint: background_endpoint,
                net_state_runner,
                common_resources,
                rng: rng.clone(),
            },
            tx_runner: MeshTxRunner {
                interface_control,
                net_tx_runner,
                common_resources,
            },
            rx_runner: MeshRxRunner {
                rx_router_input,
                interface_rx_queue_receiver,
                net_rx_runner,
                common_resources,
            },
            common_resources,
        }
    }
    pub async fn run(&mut self) -> ! {
        debug!("Mesh runner active.");
        let mut state = MeshState::Inactive;

        let our_address;
        let channel;
        let mesh_id: heapless::String<32>;
        loop {
            let MeshState::Active {
                our_address: our_address_,
                channel: channel_,
                mesh_id: mesh_id_,
            } = state
            else {
                state = self.common_resources.state_signal.wait().await;
                continue;
            };
            our_address = our_address_;
            channel = channel_;
            mesh_id = mesh_id_;
            break;
        }

        let _ = select3(
            self.management_runner.run(&our_address, &mesh_id),
            self.tx_runner.run(),
            self.rx_runner.run(),
        )
        .await;
        unreachable!("a mesh task returned!")
    }
}
