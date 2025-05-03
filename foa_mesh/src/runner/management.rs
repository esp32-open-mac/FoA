use embassy_net_driver_channel::StateRunner as NetStateRunner;
use foa::LMacInterfaceControl;

use crate::rx_router::MeshRxRouterEndpoint;

pub struct MeshManagementRunner<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_router_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
    pub(crate) net_state_runner: NetStateRunner<'vif>,
}
impl MeshManagementRunner<'_, '_> {
    pub async fn run(&mut self) -> ! {
        loop {
            let _ = self.rx_router_endpoint.receive().await;
        }
    }
}
