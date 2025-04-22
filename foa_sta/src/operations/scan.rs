use foa::util::operations::{PostChannelScanAction, ScanConfig};

use crate::{rx_router::StaRxRouterEndpoint, StaError, StaTxRx, BSS};
/// Search for a BSS, with the specified [ScanConfig].
///
/// This will return immediately when the first beacon with the specified SSID is received.
pub async fn search_for_bss<'foa, 'vif, 'params>(
    sta_tx_rx: &'params StaTxRx<'foa, 'vif>,
    rx_router_endpoint: &'params mut StaRxRouterEndpoint<'foa, 'vif>,
    scan_config: Option<ScanConfig<'params>>,
    ssid: &str,
) -> Result<BSS, StaError> {
    match foa::util::operations::scan::<_, BSS>(
        sta_tx_rx.interface_control,
        rx_router_endpoint,
        |beacon_frame, received_frame, _channel| {
            if beacon_frame.ssid() != Some(ssid) {
                return PostChannelScanAction::Continue;
            }
            let Some(bss) = BSS::from_beacon_like(beacon_frame, received_frame.rssi()) else {
                return PostChannelScanAction::Continue;
            };
            PostChannelScanAction::Stop(bss)
        },
        scan_config,
    )
    .await
    {
        Ok(Some(bss)) => Ok(bss),
        Ok(None) => Err(StaError::UnableToFindEss),
        Err(lmac_error) => Err(StaError::LMacError(lmac_error)),
    }
}
/// Enumerate all BSS's, with the specified [ScanConfig].
///
/// The key of the `bss_list` is the BSSID ID, of the network.
/// This will run until either all channels have been scanned, or the `bss_list` is full.
pub async fn enumerate_bss<'foa, 'vif, 'params, const MAX_BSS: usize>(
    sta_tx_rx: &'params StaTxRx<'foa, 'vif>,
    rx_router_endpoint: &'params mut StaRxRouterEndpoint<'foa, 'vif>,
    scan_config: Option<ScanConfig<'params>>,
    bss_list: &'params mut heapless::FnvIndexMap<[u8; 6], BSS, MAX_BSS>,
) -> Result<(), StaError> {
    foa::util::operations::scan::<_, ()>(
        sta_tx_rx.interface_control,
        rx_router_endpoint,
        |beacon_frame, received_frame, _channel| {
            if bss_list.contains_key(&*beacon_frame.header.bssid) {
                return PostChannelScanAction::Continue;
            }
            if bss_list.len() == bss_list.capacity() {
                trace!(
                    "Can't add BSS with SSID: {} and BSSID: {} to scan result list. MAX_BSS: {}",
                    beacon_frame.ssid(),
                    beacon_frame.header.bssid,
                    MAX_BSS
                );
                return PostChannelScanAction::Stop(());
            }
            let Some(bss) = BSS::from_beacon_like(beacon_frame, received_frame.rssi()) else {
                return PostChannelScanAction::Continue;
            };
            let _ = bss_list.insert(*beacon_frame.header.bssid, bss);
            PostChannelScanAction::Continue
        },
        scan_config,
    )
    .await
    .map(|_| ())
    .map_err(StaError::LMacError)
}
