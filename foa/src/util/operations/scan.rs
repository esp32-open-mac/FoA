use embassy_time::{Duration, WithTimeout};
use esp_wifi_hal::ScanningMode;
use ieee80211::{match_frames, mgmt_frame::BeaconFrame};

use crate::{
    util::rx_router::{HasScanOperation, RxRouterEndpoint, RxRouterScopedOperation},
    LMacError, LMacInterfaceControl, ReceivedFrame,
};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
/// This specifies the order in which channels should be scanned.
pub enum ScanStrategy<'a> {
    /// Only a single channel should be scanned.
    Single(u8),
    /// Only the current channel should be scanned.
    CurrentChannel,
    /// Scan the channels in ascending order (i.e. 1-13).
    Linear,
    #[default]
    /// Scan the channels in groups of three non overlapping channels (i.e. 1, 6, 11, 2, 7, 12 etc.).
    NonOverlappingFirst,
    /// Use a custom channel sequence.
    Custom(&'a [u8]),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// What to do after scanning on a channel.
pub enum PostChannelScanAction<Res> {
    #[default]
    /// Continue scanning the next channel.
    Continue,
    /// Stop scanning.
    Stop(Res),
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
/// Scan on the current channel.
async fn scan_on_channel<'foa, 'params, Operation: HasScanOperation, Res>(
    router_operation: &'params RxRouterScopedOperation<'foa, '_, 'params, Operation>,
    channel: u8,
    rx_cb: &mut impl FnMut(BeaconFrame<'_>, &ReceivedFrame<'foa>, u8) -> PostChannelScanAction<Res>,
) -> PostChannelScanAction<Res> {
    loop {
        let received = router_operation.receive().await;
        let _ = match_frames! {
            received.mpdu_buffer(),
            beacon_frame = BeaconFrame => {
                if let PostChannelScanAction::Stop(res) = (rx_cb)(beacon_frame, &received, channel) {
                    break PostChannelScanAction::Stop(res);
                }
            }
        };
    }
}
/// Search for some kind of BSS, with the specified [ScanConfig].
///
/// If a beacon frame is received, the [BeaconFrame] is passed to `beacon_rx_cb`. If that
/// returns [PostChannelScanAction::Stop], we end the scan. This can be used to enumerate BSS's.
pub async fn scan<'foa, 'vif, 'params, Operation: HasScanOperation, Res>(
    interface_control: &'params LMacInterfaceControl<'foa>,
    rx_router_endpoint: &'params mut RxRouterEndpoint<'foa, 'vif, Operation>,
    mut rx_cb: impl FnMut(BeaconFrame<'_>, &ReceivedFrame<'foa>, u8) -> PostChannelScanAction<Res>,
    scan_config: Option<ScanConfig<'_>>,
) -> Result<Option<Res>, LMacError> {
    // We begin the off channel operation.
    let mut off_channel_operation = interface_control
        .begin_interface_off_channel_operation()
        .await?;

    // We get the scan configuration.
    let scan_config = scan_config.unwrap_or_default();

    // Determine the channels, that should be scanned through the strategy.
    let channels = match scan_config.strategy {
        ScanStrategy::Single(channel) => &[channel],
        ScanStrategy::CurrentChannel => &[interface_control.get_current_channel()],
        ScanStrategy::Linear => [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13].as_slice(),
        ScanStrategy::NonOverlappingFirst => [1, 6, 11, 2, 7, 12, 3, 8, 13, 4, 9, 5, 10].as_slice(),
        ScanStrategy::Custom(channels) => channels,
    };
    debug!("ESS scan started. Scanning channels: {:?}", channels);
    // Setup scanning mode, by configuring the RX router to route beacons to us, setting the scanning mode and clearing
    // the RX queue, so every frame received after this is a beacon.
    let router_operation = rx_router_endpoint
        .start_operation(Operation::SCAN_OPERATION)
        .await;
    off_channel_operation.set_scanning_mode(ScanningMode::BeaconsOnly);

    let mut res = None;

    // Loop through channels.
    for channel in channels {
        off_channel_operation.set_channel(*channel)?;
        let post_channel_scan_action = scan_on_channel(&router_operation, *channel, &mut rx_cb)
            .with_timeout(scan_config.channel_remain_time)
            .await
            .unwrap_or(PostChannelScanAction::Continue);
        if let PostChannelScanAction::Stop(scan_res) = post_channel_scan_action {
            res = Some(scan_res);
            break;
        }
    }

    debug!("ESS scan complete.");
    router_operation.complete();

    Ok(res)
}
