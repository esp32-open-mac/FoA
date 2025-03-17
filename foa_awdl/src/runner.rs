use awdl_frame_parser::action_frame::DefaultAWDLActionFrame;
use defmt_or_log::debug;
use embassy_futures::select::{select, select3, Either3};
use embassy_sync::channel::DynamicReceiver;
use embassy_time::{Ticker, Timer};
use foa::{LMacInterfaceControl, ReceivedFrame};
use ieee80211::{
    mac_parser::MACAddress, match_frames, mgmt_frame::body::action::RawVendorSpecificActionFrame,
    scroll::Pread,
};
use rand_core::RngCore;

use crate::{peer::AwdlPeer, AwdlState, CommonResources, APPLE_OUI};

/// Runner for the AWDL interface.
pub struct AwdlRunner<'foa, 'vif, Rng: RngCore> {
    pub(crate) interface_control: &'vif LMacInterfaceControl<'foa>,
    pub(crate) rx_queue: &'vif DynamicReceiver<'foa, ReceivedFrame<'foa>>,
    pub(crate) rng: Rng,
    pub(crate) common_resources: &'vif CommonResources,
}
impl<Rng: RngCore> AwdlRunner<'_, '_, Rng> {
    async fn process_action_frame(
        &self,
        transmitter: MACAddress,
        awdl_action_body: DefaultAWDLActionFrame<'_>,
    ) {
        let _ = self.common_resources.peer_cache.inspect_or_add_peer(
            &transmitter,
            |peer| {
                peer.update(&awdl_action_body);
            },
            || {
                debug!("Added peer {} to cache.", transmitter);
                AwdlPeer::new_with_action_frame(&awdl_action_body)
            },
        );
    }
    async fn process_frame(&self) {
        let received = self.rx_queue.receive().await;
        let _ = match_frames! {
            received.mpdu_buffer(),
            action_frame = RawVendorSpecificActionFrame => {
                if action_frame.body.oui != APPLE_OUI {
                    return;
                }
                let Ok(awdl_action_body) = action_frame.body.payload.pread::<DefaultAWDLActionFrame>(0) else {
                    return;
                };
                self.process_action_frame(action_frame.header.transmitter_address, awdl_action_body).await;
            }
        };
    }
    async fn run_session(&mut self, _mac_address: MACAddress, _channel: u8) -> ! {
        let mut stale_peer_ticker = Ticker::every(
            self.common_resources
                .dynamic_session_parameters
                .stale_peer_timeout
                .get(),
        );
        loop {
            match select3(
                self.process_frame(),
                self.interface_control.wait_for_off_channel_request(),
                stale_peer_ticker.next(),
            )
            .await
            {
                Either3::First(_) => {}
                Either3::Second(off_channel_request) => off_channel_request.reject(),
                Either3::Third(_) => {
                    // Purge the stale peers with the current timeout and set the ticker to use
                    // that timeout.
                    let timeout = self
                        .common_resources
                        .dynamic_session_parameters
                        .stale_peer_timeout
                        .get();
                    self.common_resources.peer_cache.purge_stale_peers(timeout);
                    stale_peer_ticker = Ticker::every(timeout);
                }
            }
        }
    }
    /// Run the AWDL interface background task.
    pub async fn run(&mut self) -> ! {
        let state_signal = &self.common_resources.state_signal;
        let mut state = AwdlState::Inactive;
        loop {
            // We wait until we reach an active state, by looping until such a state is reached.
            // This also compensates for the user being stupid and repeatedly setting an inactive
            // state.
            let AwdlState::Active {
                mac_address,
                channel,
            } = state
            else {
                state = state_signal.wait().await;
                continue;
            };

            let _ = select(self.run_session(mac_address, channel), async {
                state = state_signal.wait().await;
            })
            .await;
        }
    }
}
