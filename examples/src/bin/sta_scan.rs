#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join3;

use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println as _;

use foa::FoAResources;
use foa_sta::StaResources;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let stack_resources = mk_static!(FoAResources, FoAResources::new());
    let ([mut sta_vif, ..], mut foa_runner) = foa::init(
        stack_resources,
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
    );
    let mut sta_resources = mk_static!(StaResources, StaResources::default());
    let (mut sta_control, mut sta_runner, _net_device) =
        foa_sta::new_sta_interface(&mut sta_vif, &mut sta_resources, Rng::new(peripherals.RNG));
    join3(foa_runner.run(), sta_runner.run(), async {
        let mut found_bss = heapless::Vec::new();
        let _ = sta_control.scan::<32>(None, &mut found_bss).await;
        for bss in found_bss {
            info!(
                "Found BSS, with SSID: \"{}\", BSSID: {}, channel: {}, last RSSI: {}.",
                bss.ssid, bss.bssid, bss.channel, bss.last_rssi
            );
        }
    })
    .await;
}
