#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;

use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

use examples::mk_static;
use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_sta::{StaResources, StaRunner};

#[embassy_executor::task]
async fn foa_task(mut runner: FoARunner<'static>) {
    runner.run().await
}
#[embassy_executor::task]
async fn sta_task(mut runner: StaRunner<'static, 'static>) {
    runner.run().await
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_bootloader_esp_idf::esp_app_desc!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let stack_resources = mk_static!(FoAResources, FoAResources::new());
    let ([sta_vif, ..], foa_runner) = foa::init(stack_resources, peripherals.WIFI);
    spawner.must_spawn(foa_task(foa_runner));
    let sta_resources = mk_static!(StaResources, StaResources::default());
    let (mut sta_control, sta_runner, _net_device) = foa_sta::new_sta_interface(
        mk_static!(VirtualInterface<'static>, sta_vif),
        sta_resources,
    );
    spawner.must_spawn(sta_task(sta_runner));
    info!("Starting scan.");
    let mut found_bss = heapless::index_map::FnvIndexMap::new();
    let _ = sta_control.scan::<32>(None, &mut found_bss).await;
    for (_, bss) in found_bss {
        info!(
            "Found BSS, with SSID: \"{}\", BSSID: {}, channel: {}, last RSSI: {} Security: {:?}.",
            bss.ssid, bss.bssid, bss.channel, bss.last_rssi, bss.security_config
        );
    }
}
