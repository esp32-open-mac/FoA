use core::marker::PhantomData;

use crate::{
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

/// This will transmit a deauthentication frame.
///
/// The value of the RA and TA fields are shown in the table below.
///
/// `to_ap` | RA | TA
/// -- | -- | --
/// `true` | BSSID | STA address
/// `false` | STA address | BSSID
pub async fn deauthenticate(
    interface_control: &LMacInterfaceControl<'_>,
    bssid: MACAddress,
    sta_address: MACAddress,
    to_ap: bool,
    phy_rate: WiFiRate,
) {
    let mut tx_buf = interface_control.alloc_tx_buf().await;
    let (receiver_address, transmitter_address) = if to_ap {
        (bssid, sta_address)
    } else {
        (sta_address, bssid)
    };
    let written = tx_buf
        .pwrite(
            DeauthenticationFrame {
                header: ManagementFrameHeader {
                    receiver_address,
                    bssid,
                    transmitter_address,
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
        .unwrap();
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
}
