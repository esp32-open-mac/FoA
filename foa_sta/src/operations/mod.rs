use ieee80211::{
    elements::rates::{EncodedRate, ExtendedSupportedRatesElement, SupportedRatesElement},
    extended_supported_rates, supported_rates,
};

pub mod connect;
pub mod deauth;
pub mod scan;

const DEFAULT_SUPPORTED_RATES: SupportedRatesElement<[EncodedRate; 8]> = supported_rates![
    5.5 B,
    11 B,
    1 B,
    2 B,
    6,
    12,
    24,
    48
];
const DEFAULT_XRATES: ExtendedSupportedRatesElement<[EncodedRate; 4]> =
    extended_supported_rates![54, 9, 18, 36];
