use crate::MTU;
use embassy_net_driver_channel::TxRunner as NetTxRunner;
use foa::LMacInterfaceControl;

pub struct MeshTxRunner<'foa, 'vif> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) net_tx_runner: NetTxRunner<'vif, MTU>,
}
impl MeshTxRunner<'_, '_> {
    pub async fn run(&mut self) -> ! {
        loop {
            let _tx_buf = self.net_tx_runner.tx_buf().await;

            self.net_tx_runner.tx_done();
        }
    }
}
