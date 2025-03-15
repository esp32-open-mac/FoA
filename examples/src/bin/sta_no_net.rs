#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;

use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println as _;

use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_sta::{StaNetDevice, StaResources, StaRunner};

use rand_core::RngCore;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SSID: &str = env!("SSID");

#[embassy_executor::task]
async fn foa_task(mut foa_runner: FoARunner<'static>) -> ! {
    foa_runner.run().await
}
#[embassy_executor::task]
async fn sta_task(mut sta_runner: StaRunner<'static, 'static>) -> ! {
    sta_runner.run().await
}
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let stack_resources = mk_static!(FoAResources, FoAResources::new());
    let ([sta_vif, ..], foa_runner) = foa::init(
        stack_resources,
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
    );
    spawner.spawn(foa_task(foa_runner)).unwrap();

    let sta_resources = mk_static!(StaResources<'static>, StaResources::default());
    let (mut sta_control, sta_runner, _net_device) = foa_sta::new_sta_interface(
        mk_static!(VirtualInterface<'static>, sta_vif),
        sta_resources,
        Rng::new(peripherals.RNG),
    );
    spawner.spawn(sta_task(sta_runner)).unwrap();

    let mut mac_address = [0u8; 6];
    Rng::new(peripherals.RNG).fill_bytes(mac_address.as_mut_slice());
    mac_address[0] &= !(1);
    sta_control.set_mac_address(mac_address).await.unwrap();
    info!("Using MAC address: {:#x}", mac_address);

    defmt::unwrap!(sta_control.connect_by_ssid(SSID, None).await);
    info!(
        "Connected to {} with AID: {:?}",
        SSID,
        sta_control.get_aid().unwrap()
    );
    let _ = sta_control.disconnect().await;
    info!("Disconnected again.")
}
