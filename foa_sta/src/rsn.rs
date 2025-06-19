use core::sync::atomic::Ordering;

use foa::{
    esp_wifi_hal::{AesCipherParameters, CipherParameters, KeyType, MultiLengthKey},
    KeySlot,
};
use ieee80211::{
    crypto::{map_passphrase_to_psk, partition_ptk},
    elements::rsn::{
        IEEE80211AkmType, IEEE80211CipherSuiteSelector, OptionalFeatureConfig, RsnElement,
    },
    mgmt_frame::{body::BeaconLikeBody, ManagementFrame},
};
use portable_atomic::AtomicU64;

/// The length of a Pairwise Master Key.
///
/// This is different for two AKMs, which we're luckily very far from implementing.
pub const PMK_LENGTH: usize = 32;
pub const WPA2_PSK_AKM: IEEE80211AkmType = IEEE80211AkmType::Psk;
pub const PTK_LENGTH: usize = WPA2_PSK_AKM.kck_len().unwrap()
    + WPA2_PSK_AKM.kek_len().unwrap()
    + IEEE80211CipherSuiteSelector::Ccmp128.tk_len().unwrap();
pub const GTK_LENGTH: usize = IEEE80211CipherSuiteSelector::Ccmp128.tk_len().unwrap();

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// The security configuration of a BSS.
///
/// This is derived from the RSN Element or lack thereof, and represents the equivalent security
/// specification.
pub enum SecurityConfig {
    /// Unsecured
    Open,
    /// Wired Equivalent Privacy
    ///
    /// WARNING: Broken beyond repair.
    Wep,
    /// Wi-Fi Protected Access
    ///
    /// WARNING: Broken beyond repair.
    Wpa,
    /// Wi-Fi Protected Access 2 Transition Mode
    ///
    /// WARNING: In terms of group addressed traffic, this is just as broken as WPA.
    Wpa2TransitionMode,
    #[default]
    /// Wi-Fi Protected Access 2
    ///
    /// WPA2 to this day holds all it's security guarantess, which however do **NOT** include
    /// forward or backward secrecy. This means that if an attacker knows the passphrase, they can
    /// passively decrypt all traffic, just by sniffing the 4WHS. WPA3 fixes these issues.
    ///
    /// Frostie314159: This always causes me massive facepalms, when there's a public Wi-Fi
    /// network, that's supposedly secure, because it uses WPA2-PSK, but the key is plastered
    /// everywhere. This "attack" is in fact so simple, that it's a builtin feature in wireshark.
    Wpa2,
    /// Wi-Fi Protected Access 3 Transition Mode
    ///
    /// This allows WPA2 and WPA3 capable devices to connect, but comes with the drawback, that if
    /// an attack knows the key and was able to capture a 4WHS of a WPA2 device, they can still
    /// decrypt and forge group addressed frames of WPA2 and WPA3 devices, as well as pairwise
    /// frames of WPA2 devices.
    Wpa3TransitionMode,
    /// Wi-Fi Protected Access 3
    ///
    /// This is the most secure configuration, since there are currently no attacks, that would
    /// allow any kind of forgery or decryption, by an attack with knowledge of the key.
    Wpa3,
    /// Opportunistic Wireless Encryption Transition Mode
    ///
    /// This is the middle ground between an open and an OWE network. Group addressed frames are
    /// still vulnerable.
    OweTransitionMode,
    /// Opportunistic Wireless Encryption
    ///
    /// This is like an open network, but the traffic is still encrypted. It does not provide any
    /// authentication. Using this instead of WPA2 for a public network is the sensible approach,
    /// if absolutely no access control is required.
    Owe,
    /// No equivalent was found, or the configuration is invalid.
    Invalid,
}
impl SecurityConfig {
    pub fn from_beacon_like<Subtype>(frame: &ManagementFrame<BeaconLikeBody<'_, Subtype>>) -> Self {
        let Some(rsn) = frame.elements.get_first_element::<RsnElement>() else {
            return Self::Open;
        };
        if rsn.pairwise_cipher_suite_list.iter().len() == 0
            || rsn
                .rsn_capbilities
                .map(|rsn_capabilities| {
                    rsn_capabilities.mfp_config() == OptionalFeatureConfig::Invalid
                })
                .unwrap_or(false)
        {
            return Self::Invalid;
        }
        // TODO: Implement this properly. Since I'm on a tight schedule and we'll only support
        // WPA2-PSK for now I'll bodge this a little. Detecting the security config used is an
        // absolute freaking nightmare.
        let is_group_cipher_ccmp = matches!(
            rsn.group_data_cipher_suite,
            None | Some(IEEE80211CipherSuiteSelector::Ccmp128)
        );
        let is_pairwise_cipher_ccmp = rsn
            .pairwise_cipher_suite_list
            .map(|mut pairwise_cipher_suites| {
                pairwise_cipher_suites
                    .any(|cipher_suite| cipher_suite == IEEE80211CipherSuiteSelector::Ccmp128)
            })
            .unwrap_or(true);
        let Some(mut akm_list) = rsn.akm_list else {
            return Self::Invalid;
        };
        let is_akm_psk = akm_list.any(|akm_suite| akm_suite == IEEE80211AkmType::Psk);
        if is_group_cipher_ccmp && is_pairwise_cipher_ccmp && is_akm_psk {
            return Self::Wpa2;
        }
        Self::Invalid
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Credentials<'a> {
    PreSharedKey(&'a [u8; PMK_LENGTH]),
    Passphrase(&'a str),
}
impl Credentials<'_> {
    pub(crate) fn pmk(&self, output: &mut [u8; PMK_LENGTH], ssid: &str) {
        match self {
            Self::PreSharedKey(psk) => output.copy_from_slice(psk.as_slice()),
            Self::Passphrase(passphrase) => map_passphrase_to_psk(passphrase, ssid, output),
        }
    }
}
#[derive(Debug)]
pub(crate) struct TransientKeySecurityAssociation<const N: usize, const IS_PAIRWISE: bool> {
    pub key: [u8; N],
    pub key_id: u8,
    replay_counter: AtomicU64,
    packet_number: AtomicU64,
}
impl<const N: usize, const IS_PAIRWISE: bool> TransientKeySecurityAssociation<N, IS_PAIRWISE> {
    pub const fn new(key: [u8; N], key_id: u8) -> Self {
        Self {
            key,
            key_id,
            replay_counter: AtomicU64::new(0),
            packet_number: AtomicU64::new(0)
        }
    }
    pub fn tk(&self) -> &[u8; 16] {
        if IS_PAIRWISE {
            partition_ptk(
                &self.key,
                WPA2_PSK_AKM,
                IEEE80211CipherSuiteSelector::Ccmp128,
            )
            .unwrap()
            .2
        } else {
            self.key.as_slice()
        }
        .try_into()
        .unwrap()
    }
    pub fn next_packet_number(&self) -> u64 {
        self.packet_number.fetch_add(1, Ordering::Relaxed)
    }
    pub fn update_and_validate_replay_counter(&self, packet_number: u64) -> bool {
        let valid = self.replay_counter.load(Ordering::Relaxed) < packet_number;
        if valid {
            self.replay_counter.store(packet_number, Ordering::Relaxed);
        }
        valid
    }
}
pub enum TksaType {
    Ptksa,
    Gtksa,
}
#[derive(Debug)]
pub(crate) struct SecurityAssociations {
    pub pmksa: [u8; PMK_LENGTH],
    pub ptksa: TransientKeySecurityAssociation<PTK_LENGTH, true>,
    pub gtksa: TransientKeySecurityAssociation<GTK_LENGTH, false>,
}
pub(crate) struct CryptoState<'foa> {
    pub gtk_key_slot: KeySlot<'foa>,
    pub ptk_key_slot: KeySlot<'foa>,
    pub security_associations: SecurityAssociations,
}
impl<'foa> CryptoState<'foa> {
    pub fn new(
        mut gtk_key_slot: KeySlot<'foa>,
        mut ptk_key_slot: KeySlot<'foa>,
        bssid: [u8; 6],
        keys: SecurityAssociations,
    ) -> Self {
        Self::init_key_slot_with_tk(&mut gtk_key_slot, &keys.gtksa, bssid);
        Self::init_key_slot_with_tk(&mut ptk_key_slot, &keys.ptksa, bssid);
        Self {
            gtk_key_slot,
            ptk_key_slot,
            security_associations: keys,
        }
    }
    fn init_key_slot_with_tk<const N: usize, const IS_PAIRWISE: bool>(
        key_slot: &mut KeySlot<'foa>,
        tk: &TransientKeySecurityAssociation<N, IS_PAIRWISE>,
        bssid: [u8; 6],
    ) {
        key_slot
            .set_key(
                tk.key_id,
                bssid,
                CipherParameters::Ccmp(AesCipherParameters {
                    key: MultiLengthKey::Short(tk.tk()),
                    key_type: if IS_PAIRWISE {
                        KeyType::Pairwise
                    } else {
                        KeyType::Group
                    },
                    mfp_enabled: false,
                    spp_enabled: false,
                }),
            )
            .unwrap();
    }
}
