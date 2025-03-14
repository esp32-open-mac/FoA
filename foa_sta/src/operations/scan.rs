use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, DynamicReceiver},
};
use embassy_time::{with_timeout, Duration};
use foa::{esp_wifi_hal::ScanningMode, ReceivedFrame};
use ieee80211::{mgmt_frame::BeaconFrame, scroll::Pread};

use crate::{
    rx_router::{Operation, RouterQueue},
    StaError, StaTxRx, BSS, RX_QUEUE_LEN,
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
pub enum ScanType<'a, const MAX_BSS: usize = 0> {
    Search(&'a str, &'a mut Option<BSS>),
    Enumerate(&'a mut heapless::Vec<BSS, MAX_BSS>),
}
async fn search_for_bss_on_channel(
    rx_queue_receiver: DynamicReceiver<'_, ReceivedFrame<'_>>,
    channel_remain_time: Duration,
    search_ssid: &str,
    bss_dst: &mut Option<BSS>,
) {
    let _ = with_timeout(channel_remain_time, async {
        loop {
            let received = rx_queue_receiver.receive().await;
            let Ok(beacon_frame) = received.mpdu_buffer().pread::<BeaconFrame>(0) else {
                continue;
            };
            let Some(beacon_ssid) = beacon_frame.ssid() else {
                continue;
            };
            if beacon_ssid == search_ssid {
                let Some(bss) = BSS::from_beacon_like(beacon_frame, received.rssi()) else {
                    continue;
                };
                *bss_dst = Some(bss);
                break;
            }
        }
    })
    .await;
}
async fn enumerate_bss_on_channel<const MAX_BSS: usize>(
    rx_queue_receiver: DynamicReceiver<'_, ReceivedFrame<'_>>,
    channel_remain_time: Duration,
    dst: &mut heapless::Vec<BSS, MAX_BSS>,
) {
    let _ = with_timeout(channel_remain_time, async {
        loop {
            let received = rx_queue_receiver.receive().await;
            let Ok(beacon_frame) = received.mpdu_buffer().pread::<BeaconFrame>(0) else {
                continue;
            };
            let Some(beacon_ssid) = beacon_frame.ssid() else {
                continue;
            };
            if let Some(bss) = dst.iter_mut().find(|bss| bss.ssid == beacon_ssid) {
                bss.last_rssi = received.rssi();
            } else {
                let Some(bss) = BSS::from_beacon_like(beacon_frame, received.rssi()) else {
                    continue;
                };
                let _ = dst.push(bss);
            }
        }
    })
    .await;
}
/// Scan for ESS's, with the specified [ScanConfig].
///
/// If a beacon frame is received, the [BorrowedBuffer] is passed to `beacon_rx_cb`. If that
/// returns `false`, we end the scan. This can be used to implement searching for a specific
/// ESS and stopping once it's found.
pub async fn scan<const MAX_BSS: usize>(
    sta_tx_rx: &StaTxRx<'_, '_>,
    rx_queue: &Channel<NoopRawMutex, ReceivedFrame<'_>, RX_QUEUE_LEN>,
    router_queue: RouterQueue,
    scan_config: Option<ScanConfig<'_>>,
    mut scan_type: ScanType<'_, MAX_BSS>,
) -> Result<(), StaError> {
    // We begin the off channel operation.
    let mut off_channel_operation = sta_tx_rx
        .interface_control
        .begin_interface_off_channel_operation()
        .await
        .map_err(StaError::LMacError)?;

    // We get the scan configuration.
    let scan_config = scan_config.unwrap_or_default();

    // Determine the channels, that should be scanned through the strategy.
    let channels = match scan_config.strategy {
        ScanStrategy::Single(channel) => &[channel],
        ScanStrategy::CurrentChannel => &[sta_tx_rx.interface_control.get_current_channel()],
        ScanStrategy::Linear => [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13].as_slice(),
        ScanStrategy::NonOverlappingFirst => [1, 6, 11, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13].as_slice(),
        ScanStrategy::Custom(channels) => channels,
    };
    debug!("ESS scan started. Scanning channels: {:?}", channels);
    // Setup scanning mode, by configuring the RX router to route beacons to us, setting the scanning mode and clearing
    // the RX queue, so every frame received after this is a beacon.
    let router_operation = sta_tx_rx
        .rx_router
        .begin_scoped_operation(router_queue, Operation::Scanning)
        .await;
    off_channel_operation.set_scanning_mode(ScanningMode::BeaconsOnly);
    rx_queue.clear();

    // Loop through channels.
    for channel in channels {
        off_channel_operation
            .set_channel(*channel)
            .map_err(StaError::LMacError)?;
        match &mut scan_type {
            ScanType::Search(search_ssid, bss_dst) => {
                search_for_bss_on_channel(
                    rx_queue.dyn_receiver(),
                    scan_config.channel_remain_time,
                    search_ssid,
                    bss_dst,
                )
                .await;
                if bss_dst.is_some() {
                    break;
                }
            }
            ScanType::Enumerate(dst) => {
                enumerate_bss_on_channel(
                    rx_queue.dyn_receiver(),
                    scan_config.channel_remain_time,
                    dst,
                )
                .await
            }
        }
    }

    debug!("ESS scan complete.");
    router_operation.complete();

    Ok(())
}
