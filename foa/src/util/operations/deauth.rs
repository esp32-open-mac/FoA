use core::marker::PhantomData;

use crate::{PendingTransmission, RetryBehaviour, TxEndpoint};
use esp_wifi_hal::{
    ll::EdcaAccessCategory,
    prelude::{TxMacParameters, TxPhyRate, TxPlcpParameters},
};
use ieee80211::{
    common::{IEEE80211Reason, SequenceControl},
    element_chain,
    mac_parser::MACAddress,
    mgmt_frame::{DeauthenticationFrame, ManagementFrameHeader, body::DeauthenticationBody},
    scroll::Pwrite,
};

/// This will transmit a deauthentication frame.
///
/// While this function is async, it will not wait for the transmission to complete.
/// If you wish to do that, you may use the [PendingTransmission] to wait for it.
///
/// The value of the RA and TA fields are shown in the table below.
///
/// `to_ap` | RA | TA
/// -- | -- | --
/// `true` | BSSID | STA address
/// `false` | STA address | BSSID
pub async fn deauthenticate<'a>(
    tx_endpoint: &'a TxEndpoint<'_>,
    bssid: MACAddress,
    sta_address: MACAddress,
    to_ap: bool,
    rate: TxPhyRate,
) -> PendingTransmission<'a> {
    let mut tx_buf = tx_endpoint.alloc_tx_buf().await;
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
    tx_endpoint.transmit_edca(
        EdcaAccessCategory::default(),
        tx_buf,
        written,
        TxPlcpParameters {
            rate,
            ..Default::default()
        },
        TxMacParameters {
            wait_for_ack: true,
            override_seq_num: true,
            ..Default::default()
        },
        RetryBehaviour::RetryUntil(7),
    )
}
