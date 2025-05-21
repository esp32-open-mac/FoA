use crate::{MTU, state::CommonResources};
use defmt_or_log::debug;
use embassy_net_driver_channel::TxRunner as NetTxRunner;
use foa::LMacInterfaceControl;

pub struct MeshTxRunner<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) net_tx_runner: NetTxRunner<'vif, MTU>,
    pub(crate) common_resources: &'vif CommonResources,
}
impl MeshTxRunner<'_, '_> {
    pub async fn run(&mut self) -> ! {
        loop {
            debug!("Before TX");
            let _tx_buf = self.net_tx_runner.tx_buf().await;
            debug!("after tx");
            self.net_tx_runner.tx_done();
            debug!("after tx done");
        }
    }
}
