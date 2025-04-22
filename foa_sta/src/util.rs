use core::fmt::Display;

/// A wrapper to log hex bytes with log or defmt.
pub struct HexWrapper<'a>(pub &'a [u8]);
impl Display for HexWrapper<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("{:#02x?}", self.0))
    }
}
#[cfg(feature = "defmt")]
impl defmt::Format for HexWrapper<'_> {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{:02x}", self.0);
    }
}
