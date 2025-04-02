use crate::{
    state::{AwdlState, CommonResources},
    AWDL_MTU,
};
use embassy_futures::select::select4;
use embassy_net_driver_channel::{
    driver::{HardwareAddress, LinkState},
    Runner, StateRunner,
};
use embassy_sync::channel::DynamicReceiver;
use foa::{LMacInterfaceControl, ReceivedFrame};

mod management;
mod rx;
mod tx;

use management::AwdlManagementRunner;
use rx::AwdlMpduRxRunner;
use tx::AwdlMsduTxRunner;

/// Runner for the AWDL interface.
pub struct AwdlRunner<'foa, 'vif> {
    pub(crate) management_runner: AwdlManagementRunner<'foa, 'vif>,
    pub(crate) mpdu_rx_runner: AwdlMpduRxRunner<'foa, 'vif>,
    pub(crate) msdu_tx_runner: AwdlMsduTxRunner<'foa, 'vif>,
    pub(crate) common_resources: &'vif CommonResources,
    pub(crate) state_runner: StateRunner<'vif>,
}
impl<'foa, 'vif> AwdlRunner<'foa, 'vif> {
    pub(crate) fn new(
        interface_control: &'vif LMacInterfaceControl<'foa>,
        rx_queue: &'vif DynamicReceiver<'foa, ReceivedFrame<'foa>>,
        common_resources: &'vif CommonResources,
        net_runner: Runner<'vif, AWDL_MTU>,
    ) -> Self {
        let (state_runner, rx_runner, tx_runner) = net_runner.split();
        Self {
            management_runner: AwdlManagementRunner {
                interface_control,
                common_resources,
            },
            mpdu_rx_runner: AwdlMpduRxRunner {
                rx_queue,
                common_resources,
                rx_runner,
            },
            msdu_tx_runner: AwdlMsduTxRunner {
                interface_control,
                tx_runner,
                common_resources,
            },
            common_resources,
            state_runner,
        }
    }
    /// Run the AWDL interface background task.
    pub async fn run(&mut self) -> ! {
        let mut state = AwdlState::Inactive;
        loop {
            // We wait until we reach an active state, by looping until such a state is reached.
            // This also compensates for the user being stupid and repeatedly setting an inactive
            // state.
            let AwdlState::Active {
                our_address,
                channel,
            } = state
            else {
                state = self.common_resources.state_signal.wait().await;
                continue;
            };

            // Setup session parameters
            self.common_resources
                .initialize_session_parameters(channel, our_address);

            // We configure embassy_net here.
            self.state_runner
                .set_hardware_address(HardwareAddress::Ethernet(*our_address));
            self.state_runner.set_link_state(LinkState::Up);

            let _ = select4(
                self.management_runner.run_session(&our_address),
                self.mpdu_rx_runner.run_session(&our_address),
                self.msdu_tx_runner.run_session(),
                async {
                    state = self.common_resources.state_signal.wait().await;
                },
            )
            .await;
            // We clear the peer cache here and set the link down, so the next session starts of fresh.
            self.common_resources.clear_peer_cache();
            self.state_runner.set_link_state(LinkState::Down);
        }
    }
}
