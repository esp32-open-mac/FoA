use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{with_timeout, Duration};
use foa::{esp_wifi_hal::ScanningMode, LMacInterfaceControl, ReceivedFrame};

use crate::{
    rx_router::{Operation, RxQueue, RxRouter},
    StaError,
};

/// Configuration for a scan.
///
/// This allows configuring how the scan is performed and on which channels.
pub struct ScanConfig<'a> {
    /// The time we should remain on one channel, before jumping to the next one.
    pub channel_remain_time: Duration,
    /// The channels to be scanned. Leaving this set to [None], will mean all channels will be
    /// scanned.
    pub channels: Option<&'a [u8]>,
}
impl ScanConfig<'_> {
    fn get_channels(&self) -> &[u8] {
        self.channels
            .unwrap_or(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13])
    }
}
impl Default for ScanConfig<'_> {
    fn default() -> Self {
        Self {
            channel_remain_time: Duration::from_millis(200),
            channels: None,
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
    pub(crate) router_queue: RxQueue,
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
        debug!(
            "ESS scan started. Scanning channels: {:?}",
            scan_config.get_channels()
        );
        // Setup scanning mode.
        self.rx_router
            .begin_operation(self.router_queue, Operation::Scanning)
            .await;
        off_channel_operation.set_scanning_mode(ScanningMode::BeaconsOnly);
        self.rx_queue.clear();

        // Loop through channels.
        for channel in scan_config.get_channels() {
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

        Ok(())
    }
}
impl Drop for ScanOperation<'_, '_> {
    fn drop(&mut self) {
        self.rx_router.end_operation(self.router_queue);
    }
}
