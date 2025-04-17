use defmt_or_log::debug;
use embassy_futures::join::join3;
use embassy_net_driver_channel::Runner as NetRunner;

use foa::{LMacInterfaceControl, RxQueueReceiver};
use management::MeshManagementRunner;
use rx::MeshRxRunner;
use tx::MeshTxRunner;

use crate::{
    MTU,
    rx_router::{MeshRxRouterEndpoint, MeshRxRouterInput},
};

mod management;
mod rx;
mod tx;

pub struct MeshRunner<'foa, 'vif> {
    pub(crate) management_runner: MeshManagementRunner<'foa, 'vif>,
    pub(crate) tx_runner: MeshTxRunner<'foa, 'vif>,
    pub(crate) rx_runner: MeshRxRunner<'foa, 'vif>,
}
impl<'foa, 'vif> MeshRunner<'foa, 'vif> {
    pub(crate) fn new(
        net_runner: NetRunner<'vif, MTU>,
        background_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
        rx_router_input: MeshRxRouterInput<'foa, 'vif>,
        interface_rx_queue_receiver: &'vif RxQueueReceiver<'foa>,
        interface_control: &'vif LMacInterfaceControl<'foa>,
    ) -> Self {
        let (net_state_runner, net_rx_runner, net_tx_runner) = net_runner.split();
        Self {
            management_runner: MeshManagementRunner {
                interface_control,
                rx_router_endpoint: background_endpoint,
                net_state_runner,
            },
            tx_runner: MeshTxRunner {
                interface_control,
                net_tx_runner,
            },
            rx_runner: MeshRxRunner {
                rx_router_input,
                interface_rx_queue_receiver,
                net_rx_runner,
            },
        }
    }
    pub async fn run(&mut self) -> ! {
        debug!("Mesh runner active.");
        let _ = join3(
            self.management_runner.run(),
            self.tx_runner.run(),
            self.rx_runner.run(),
        )
        .await;
        unreachable!()
    }
}
