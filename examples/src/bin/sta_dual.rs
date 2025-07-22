#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;
use embassy_futures::join::join_array;
use embassy_net::{
    dns::DnsSocket,
    udp::{PacketMetadata, UdpSocket},
    DhcpConfig, Runner as NetRunner, StackResources as NetStackResources,
};
use embassy_time::Timer;

use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println as _;

use foa::{FoAResources, FoARunner};
use foa_sta::{StaNetDevice, StaResources, StaRunner};

extern crate alloc;
use alloc::boxed::Box;

const SSID: &str = env!("SSID");
#[embassy_executor::task]
async fn foa_task(mut foa_runner: FoARunner<'static>) -> ! {
    foa_runner.run().await
}
#[embassy_executor::task(pool_size = 2)]
async fn sta_task(mut sta_runner: StaRunner<'static, 'static>) -> ! {
    sta_runner.run().await
}
#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut net_runner: NetRunner<'static, StaNetDevice<'static>>) -> ! {
    net_runner.run().await
}
async fn run_net_stack(spawner: &Spawner, net_device: StaNetDevice<'static>) {
    esp_bootloader_esp_idf::esp_app_desc!();
    let net_stack_resources = Box::new(NetStackResources::<3>::new());
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        Box::leak(net_stack_resources),
        1234,
    );
    spawner.spawn(net_task(net_runner)).unwrap();
    info!("waiting for DHCP...");
    net_stack.wait_config_up().await;
    info!(
        "DHCP is now up! Got address {}",
        net_stack.config_v4().unwrap().address
    );
    let dns_socket = DnsSocket::new(net_stack);
    let tcp_bin = dns_socket
        .query("tcpbin.org", embassy_net::dns::DnsQueryType::A)
        .await
        .expect("DNS lookup failure")[0];
    info!("Queried tcpbin.com: {:?}", tcp_bin);
    let endpoint = embassy_net::IpEndpoint {
        addr: tcp_bin,
        port: 4242,
    };
    let mut rx_buffer = Box::new([0u8; 1600]);
    let mut tx_buffer = Box::new([0u8; 1600]);
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut udp_socket = UdpSocket::new(
        net_stack,
        &mut rx_meta,
        rx_buffer.as_mut(),
        &mut tx_meta,
        tx_buffer.as_mut(),
    );
    info!("Connected to tcpbin.com");
    udp_socket.bind(endpoint).unwrap();
    loop {
        udp_socket
            .send_to([0xff; 1].as_slice(), endpoint)
            .await
            .unwrap();
        Timer::after_secs(1).await;
    }
}
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals =
        esp_hal::init(esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::_240MHz));

    heap_allocator!(size: 100 * 1024);
    info!("Initialized FoA with two interfaces.");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let foa_resources = Box::new(FoAResources::new());
    let ([vif_0, vif_1, ..], foa_runner) = foa::init(
        Box::leak(foa_resources),
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
    );
    spawner.spawn(foa_task(foa_runner)).unwrap();

    let rng = Rng::new(peripherals.RNG);

    let mut stas = [vif_0, vif_1].map(|vif| {
        let sta_resources = Box::new(StaResources::new());

        let vif = Box::new(vif);

        let (sta_control, sta_runner, sta_net_device) =
            foa_sta::new_sta_interface(Box::leak(vif), Box::leak(sta_resources), rng);
        spawner.spawn(sta_task(sta_runner)).unwrap();
        (sta_control, sta_net_device)
    });

    let bss = stas[0].0.find_ess(None, SSID).await.unwrap();
    join_array(stas.map(|(mut sta_control, sta_net_device)| {
        let bss = bss.clone();
        async move {
            sta_control.connect(bss, None, None).await.unwrap();
            run_net_stack(&spawner, sta_net_device).await;
        }
    }))
    .await;
}
