use foa::LMacInterfaceControl;

use crate::rx_router::MeshRxRouterEndpoint;

pub struct MeshControl<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_router_endpoint: MeshRxRouterEndpoint<'foa, 'vif>,
}
