use core::marker::PhantomData;

use foa::{esp_wifi_hal::TxParameters, LMacInterfaceControl};
use ieee80211::{
    common::{IEEE80211Reason, SequenceControl},
    element_chain,
    mgmt_frame::{body::DeauthenticationBody, DeauthenticationFrame, ManagementFrameHeader},
    scroll::Pwrite,
};

use crate::{ConnectionInfo, DEFAULT_PHY_RATE};

/// This will transmit a deauth frame to the AP, but not unlock the channel.
pub async fn send_deauth(
    interface_control: &LMacInterfaceControl<'_>,
    connection_info: &ConnectionInfo,
) {
    let mut tx_buf = interface_control.alloc_tx_buf().await;
    let written = tx_buf
        .pwrite(
            DeauthenticationFrame {
                header: ManagementFrameHeader {
                    receiver_address: connection_info.bssid,
                    bssid: connection_info.bssid,
                    transmitter_address: connection_info.own_address,
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
                rate: DEFAULT_PHY_RATE,
                ..LMacInterfaceControl::DEFAULT_TX_PARAMETERS
            },
            true,
        )
        .await;
}
