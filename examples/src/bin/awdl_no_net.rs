#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use examples::mk_static;
use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_awdl::{AwdlResources, AwdlRunner};

#[embassy_executor::task]
async fn foa_task(mut runner: FoARunner<'static>) {
    runner.run().await
}
#[embassy_executor::task]
async fn awdl_task(mut runner: AwdlRunner<'static, 'static>) -> ! {
    runner.run().await
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_bootloader_esp_idf::esp_app_desc!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let foa_resources = mk_static!(FoAResources, FoAResources::new());
    let ([awdl_vif, ..], foa_runner) = foa::init(foa_resources, peripherals.WIFI);
    spawner.spawn(foa_task(foa_runner)).unwrap();
    let awdl_vif = mk_static!(VirtualInterface<'static>, awdl_vif);
    let awdl_resources = mk_static!(AwdlResources, AwdlResources::new());
    let (mut awdl_control, awdl_runner, _awdl_net_device, _awdl_event_queue_rx) =
        foa_awdl::new_awdl_interface(awdl_vif, awdl_resources);
    spawner.spawn(awdl_task(awdl_runner)).unwrap();
    awdl_control
        .start()
        .await
        .expect("Failed to start AWDL interface.");
}
