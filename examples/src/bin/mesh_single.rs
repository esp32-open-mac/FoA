#![no_std]
#![no_main]

use defmt::{debug, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println as _;
use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_mesh::MeshRunner;
use foa_mesh::state::MeshResources;
use heapless::String;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[embassy_executor::task]
async fn foa_task(mut runner: FoARunner<'static>) -> ! {
    runner.run().await;
}
#[embassy_executor::task]
async fn mesh_task(mut runner: MeshRunner<'static, 'static, Rng>) -> ! {
    runner.run().await
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    info!("Welcome!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    info!("Embassy!");
    esp_hal_embassy::init(timg0.timer0);
    info!("After embassy!");

    let foa_resources = mk_static!(FoAResources, FoAResources::new());
    let ([mesh_vif, ..], foa_runner) = foa::init(foa_resources, peripherals.WIFI, peripherals.ADC2);
    spawner.spawn(foa_task(foa_runner)).unwrap();
    let mesh_vif = mk_static!(VirtualInterface<'static>, mesh_vif);
    let mesh_resources = mk_static!(MeshResources, MeshResources::new());

    let (mut mesh_control, mesh_runner, _mesh_net_device) = foa_mesh::new_mesh_interface(
        mesh_resources,
        mesh_vif,
        2,
        String::try_from("meshtest").unwrap(),
        Rng::new(peripherals.RNG),
    );
    info!("Before spawn!");
    spawner.spawn(mesh_task(mesh_runner)).unwrap();
    info!("Starting!");

    mesh_control
        .start()
        .await
        .expect("Failed to start Mesh interface.");
    loop {
        Timer::after_millis(1000).await;
        debug!("still alive!");
    }
}
