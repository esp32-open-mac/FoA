use core::marker::PhantomData;

use foa::{
    esp_wifi_hal::{TxParameters, WiFiRate},
    LMacInterfaceControl,
};
use ieee80211::{
    common::{IEEE80211Reason, SequenceControl},
    element_chain,
    mac_parser::MACAddress,
    mgmt_frame::{body::DeauthenticationBody, DeauthenticationFrame, ManagementFrameHeader},
    scroll::Pwrite,
};

use crate::StaError;

/// This will transmit a deauth frame to the AP, but not unlock the channel.
pub async fn send_deauth(
    interface_control: &LMacInterfaceControl<'_>,
    bssid: MACAddress,
    own_address: MACAddress,
    phy_rate: WiFiRate,
) -> Result<(), StaError> {
    let mut tx_buf = interface_control.alloc_tx_buf().await;
    let written = tx_buf
        .pwrite(
            DeauthenticationFrame {
                header: ManagementFrameHeader {
                    receiver_address: bssid,
                    bssid,
                    transmitter_address: own_address,
                    sequence_control: SequenceControl::new(),
                    ..Default::default()
                },
                body: DeauthenticationBody {
                    reason: IEEE80211Reason::LeavingNetworkDeauth,
                    elements: element_chain! {},
                    _phantom: PhantomData,
                },
            },
            0,
        )
        .map_err(|_| StaError::TxBufferTooSmall)?;
    let _ = interface_control
        .transmit(
            &mut tx_buf[..written],
            &TxParameters {
                rate: phy_rate,
                ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
            },
            true,
        )
        .await;
    Ok(())
}
