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
pub struct PskLengthMismatchError;
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
/// Network credentials.
pub enum Credentials<'a> {
    /// A pre shared key (PSK).
    PreSharedKey(&'a [u8]),
    /// A passphrase.
    ///
    /// We internally convert this to a PSK for further processing.
    Passphrase(&'a str),
}
impl Credentials<'_> {
    /// Get the PMK for the provided credentials.
    ///
    /// If this is a PSK and the length doesn't match the provided output slice,
    /// [PskLengthMismatchError] is returned.
    pub(crate) fn pmk(&self, output: &mut [u8], ssid: &str) -> Result<(), PskLengthMismatchError> {
        match self {
            Self::PreSharedKey(psk) => {
                if output.len() == psk.len() {
                    output.copy_from_slice(psk);
                } else {
                    return Err(PskLengthMismatchError);
                }
            }
            Self::Passphrase(passphrase) => map_passphrase_to_psk(passphrase, ssid, output),
        }
        Ok(())
    }
}
#[derive(Debug)]
/// A transient key security association.
///
/// Currently this is only meant for GTKSA's and PTKSA's
pub(crate) struct TransientKeySecurityAssociation<const N: usize, const IS_PAIRWISE: bool> {
    /// The cryptographic key of this TKSA.
    pub key: [u8; N],
    /// The ID of the key.
    pub key_id: u8,
    /// The replay counter tracks the last packet number received from the STA, with which this
    /// TKSA is established. It's updated with the PN of any received frame, where the PN is larger
    /// than the current value.
    replay_counter: AtomicU64,
    /// The TX packet number is incremented by one for every transmitted encapsulated MPDU.
    packet_number: AtomicU64,
}
impl<const N: usize, const IS_PAIRWISE: bool> TransientKeySecurityAssociation<N, IS_PAIRWISE> {
    /// Create a new TKSA with all counters initialized to zero.
    pub const fn new(key: [u8; N], key_id: u8) -> Self {
        Self {
            key,
            key_id,
            replay_counter: AtomicU64::new(0),
            packet_number: AtomicU64::new(1),
        }
    }
    /// Get the temporal key for this TKSA.
    pub fn tk(
        &self,
        akm_suite: IEEE80211AkmType,
        cipher_suite: IEEE80211CipherSuiteSelector,
    ) -> &[u8] {
        if IS_PAIRWISE {
            partition_ptk(&self.key, akm_suite, cipher_suite).unwrap().2
        } else {
            self.key.as_slice()
        }
    }
    /// Get the next TX packet number for this TKSA.
    pub fn next_packet_number(&self) -> u64 {
        self.packet_number.fetch_add(1, Ordering::Relaxed)
    }
    /// Check if the PN is valid and update the replay counter.
    ///
    /// This will only update the replay counter, if the PN is larger than the current value.
    pub fn update_and_validate_replay_counter(&self, packet_number: u64) -> bool {
        let replay_counter = self.replay_counter.load(Ordering::Relaxed);
        let valid = replay_counter <= packet_number;
        if valid {
            self.replay_counter.store(packet_number, Ordering::Relaxed);
        } else {
            debug!(
                "Packet number not greater than replay counter. {} <= {}",
                replay_counter, packet_number
            );
        }
        valid
    }
}
#[derive(Debug)]
/// All security associations used in a WPA2 network.
pub(crate) struct SecurityAssociations {
    /// The pairwise transient key.
    pub ptksa: TransientKeySecurityAssociation<PTK_LENGTH, true>,
    /// The group transient key.
    pub gtksa: TransientKeySecurityAssociation<GTK_LENGTH, false>,
    /// The Authentication and Key Management Suite.
    pub akm_suite: IEEE80211AkmType,
    /// The cipher suite.
    pub cipher_suite: IEEE80211CipherSuiteSelector,
}
impl SecurityAssociations {
    pub fn pairwise_temporal_key(&self) -> &[u8] {
        self.ptksa.tk(self.akm_suite, self.cipher_suite)
    }
    pub fn group_temporal_key(&self) -> &[u8] {
        self.gtksa.tk(self.akm_suite, self.cipher_suite)
    }
    /*
    pub fn kck(&self) -> &[u8] {
        partition_ptk(&self.ptksa.key, self.akm_suite, self.cipher_suite)
            .unwrap()
            .0
    }
    pub fn kek(&self) -> &[u8] {
        partition_ptk(&self.ptksa.key, self.akm_suite, self.cipher_suite)
            .unwrap()
            .1
    }
    */
}
/// State of cryptographic management.
pub(crate) struct CryptoState<'foa> {
    /// Key slot used for the GTK.
    pub gtk_key_slot: KeySlot<'foa>,
    /// Key slot used for the PTK.
    pub ptk_key_slot: KeySlot<'foa>,
    /// All security associations.
    pub security_associations: SecurityAssociations,
}
impl<'foa> CryptoState<'foa> {
    pub fn new(
        gtk_key_slot: KeySlot<'foa>,
        ptk_key_slot: KeySlot<'foa>,
        bssid: [u8; 6],
        security_associations: SecurityAssociations,
    ) -> Self {
        let mut temp = Self {
            gtk_key_slot,
            ptk_key_slot,
            security_associations,
        };
        temp.update_key_slot(false, bssid);
        temp.update_key_slot(true, bssid);
        temp
    }
    /*
    pub fn update_gtksa(&mut self, gtk: &[u8; 16], gtk_key_id: u8, bssid: [u8; 6]) {
        self.security_associations.gtksa.key.copy_from_slice(gtk);
        self.security_associations.gtksa.key_id = gtk_key_id;
        self.update_key_slot(true, bssid);
    }
    */
    fn update_key_slot(&mut self, group: bool, bssid: [u8; 6]) {
        let (key_slot, tk, key_id, key_type) = if group {
            (
                &mut self.gtk_key_slot,
                self.security_associations.group_temporal_key(),
                self.security_associations.gtksa.key_id,
                KeyType::Group,
            )
        } else {
            (
                &mut self.ptk_key_slot,
                self.security_associations.pairwise_temporal_key(),
                self.security_associations.ptksa.key_id,
                KeyType::Pairwise,
            )
        };
        key_slot
            .set_key(
                key_id,
                bssid,
                CipherParameters::Ccmp(AesCipherParameters {
                    key: MultiLengthKey::Short(tk.try_into().unwrap()),
                    key_type,
                    mfp_enabled: false,
                    spp_enabled: false,
                }),
            )
            .unwrap();
    }
}
