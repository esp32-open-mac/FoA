use embassy_net_driver_channel::RxRunner as NetRxRunner;
use foa::RxQueueReceiver;

use crate::{MTU, rx_router::MeshRxRouterInput};

pub struct MeshRxRunner<'foa, 'vif> {
    pub(crate) rx_router_input: MeshRxRouterInput<'foa, 'vif>,
    pub(crate) interface_rx_queue_receiver: &'vif RxQueueReceiver<'foa>,
    pub(crate) net_rx_runner: NetRxRunner<'vif, MTU>,
}
impl MeshRxRunner<'_, '_> {
    pub async fn run(&mut self) -> ! {
        loop {
            let received_frame = self.interface_rx_queue_receiver.receive().await;
            // Do some actual data RX handling here.
            let _ = self.rx_router_input.route_frame(received_frame);
        }
    }
}
