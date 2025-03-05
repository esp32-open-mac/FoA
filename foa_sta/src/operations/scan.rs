use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{with_timeout, Duration};
use foa::{esp_wifi_hal::ScanningMode, LMacInterfaceControl, ReceivedFrame};

use crate::{
    rx_router::{Operation, RouterQueue, RxRouter},
    StaError,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ScanStrategy<'a> {
    Single(u8),
    CurrentChannel,
    Linear,
    #[default]
    NonOverlappingFirst,
    Custom(&'a [u8]),
}

/// Configuration for a scan.
///
/// This allows configuring how the scan is performed and on which channels.
pub struct ScanConfig<'a> {
    /// The time we should remain on one channel, before jumping to the next one.
    ///
    /// The default is 200 ms. It is not recommended, to go below 100 ms, since that's the beacon
    /// interval of almost every network.
    pub channel_remain_time: Duration,
    /// The strategy, that should be used to scan channels.
    pub strategy: ScanStrategy<'a>,
}
impl Default for ScanConfig<'_> {
    fn default() -> Self {
        Self {
            channel_remain_time: Duration::from_millis(200),
            strategy: ScanStrategy::default(),
        }
    }
}
/// Scan for ESS's.
///
/// This is it's own struct, to have easier cancel safety, since we can implement drop directly on
/// this, which will restore everything to it's original state.
pub(crate) struct ScanOperation<'a, 'res> {
    pub(crate) rx_router: &'a RxRouter,
    pub(crate) rx_queue: &'a Channel<NoopRawMutex, ReceivedFrame<'res>, 4>,
    pub(crate) router_queue: RouterQueue,
    pub(crate) interface_control: &'a LMacInterfaceControl<'res>,
}
impl ScanOperation<'_, '_> {
    /// Scan for ESS's, with the specified [ScanConfig].
    ///
    /// If a beacon frame is received, the [BorrowedBuffer] is passed to `beacon_rx_cb`. If that
    /// returns `false`, we end the scan. This can be used to implement searching for a specific
    /// ESS and stopping once it's found.
    pub async fn run(
        self,
        scan_config: Option<ScanConfig<'_>>,
        mut beacon_rx_cb: impl FnMut(ReceivedFrame<'_>) -> bool,
    ) -> Result<(), StaError> {
        // We begin the off channe operation.
        let mut off_channel_operation = self
            .interface_control
            .begin_interface_off_channel_operation()
            .await
            .map_err(StaError::LMacError)?;

        // We get the scan configuration.
        let scan_config = scan_config.unwrap_or_default();
        let channels = match scan_config.strategy {
            ScanStrategy::Single(channel) => &[channel],
            ScanStrategy::CurrentChannel => &[self.interface_control.get_current_channel()],
            ScanStrategy::Linear => [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13].as_slice(),
            ScanStrategy::NonOverlappingFirst => {
                [1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13].as_slice()
            }
            ScanStrategy::Custom(channels) => channels,
        };
        debug!("ESS scan started. Scanning channels: {:?}", channels);
        // Setup scanning mode.
        let router_operation = self
            .rx_router
            .begin_scoped_operation(self.router_queue, Operation::Scanning)
            .await;
        off_channel_operation.set_scanning_mode(ScanningMode::BeaconsOnly);
        self.rx_queue.clear();

        // Loop through channels.
        for channel in channels {
            off_channel_operation
                .set_channel(*channel)
                .map_err(StaError::LMacError)?;
            // Receive on the channel, until the channel remain time is over.
            if with_timeout(scan_config.channel_remain_time, async {
                loop {
                    // If beacon_rx_cb returns false, we break.
                    if !beacon_rx_cb(self.rx_queue.receive().await) {
                        break;
                    }
                }
            })
            .await
            .is_ok()
            {
                break;
            }
        }

        debug!("ESS scan complete.");
        router_operation.complete();

        Ok(())
    }
}
