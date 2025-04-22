#![no_std]
use foa_sta::Credentials;

pub fn get_credentials() -> Option<Credentials<'static>> {
    option_env!("PASSWORD").map(Credentials::Passphrase)
}
#[macro_export]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.init_with(|| ($val));
        x
    }};
}
