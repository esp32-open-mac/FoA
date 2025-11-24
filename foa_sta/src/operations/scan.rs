use foa::util::operations::{PostChannelScanAction, ScanConfig};
use heapless::index_map::FnvIndexMap;
use ieee80211::{
    elements::DSSSParameterSetElement,
    mac_parser::MACAddress,
    mgmt_frame::{body::BeaconLikeBody, ManagementFrame},
};

use crate::{rx_router::StaRxRouterEndpoint, SecurityConfig, StaError, StaTxRx};

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
/// Information about a BSS.
pub struct BSS {
    /// The SSID of the BSS.
    pub ssid: heapless::String<32>,
    /// The channel on which the BSS operates.
    ///
    /// NOTE: This is taken from the DSSS Parameter Set Element and not just the channel on which
    /// we received the beacon.
    pub channel: u8,
    /// The BSSID of the BSS.
    pub bssid: MACAddress,
    /// The RSSI in dBm of the last frame received from this BSS.
    pub last_rssi: i8,
    /// The security configuration of the network.
    pub security_config: SecurityConfig,
}
impl BSS {
    /// Create a [BSS] from the information in a beacon or probe response frame.
    pub fn from_beacon_like<Subtype>(
        frame: ManagementFrame<BeaconLikeBody<'_, Subtype>>,
        rssi: i8,
    ) -> Option<Self> {
        let mut ssid = heapless::String::new();
        let _ = ssid.push_str(frame.ssid()?);
        let channel = frame
            .elements
            .get_first_element::<DSSSParameterSetElement>()?
            .current_channel;
        let bssid = frame.header.bssid;
        let security_config = SecurityConfig::from_beacon_like(&frame);
        Some(Self {
            ssid,
            channel,
            bssid,
            last_rssi: rssi,
            security_config,
        })
    }
}
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
    bss_list: &'params mut FnvIndexMap<[u8; 6], BSS, MAX_BSS>,
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
