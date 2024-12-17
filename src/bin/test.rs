#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::{
    dns::{DnsQueryType, DnsSocket},
    udp::{PacketMetadata, UdpSocket},
    DhcpConfig, Runner as NetRunner, StackResources as NetStackResources,
};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use foa::{
    bg_task::SingleInterfaceRunner,
    sta::{control::BSS, StaInitInfo, StaInterface, StaNetDevice, StaSharedResources},
    FoAStackResources,
};
use log::info;
use rand_core::RngCore;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SSID: &str = "Freifunk"; // My test router.

#[embassy_executor::task]
async fn wifi_task(mut wifi_runner: SingleInterfaceRunner<'static, StaInterface>) -> ! {
    wifi_runner.run().await
}
#[embassy_executor::task]
async fn net_task(mut net_runner: NetRunner<'static, StaNetDevice<'static>>) -> ! {
    net_runner.run().await
}
fn print_bss(bss: BSS) {
    let BSS {
        ssid,
        bssid,
        channel,
        last_rssi,
    } = bss;
    info!("Found ESS with SSID: \"{ssid}\" and BSSID: {bssid} on channel {channel}. Last RSSI was {last_rssi}dBm.");
}
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let stack_resources = mk_static!(
        FoAStackResources<StaSharedResources>,
        FoAStackResources::new()
    );
    let ((mut sta_control, net_device), runner) = foa::new_with_single_interface::<StaInterface>(
        stack_resources,
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        StaInitInfo,
    )
    .await;
    spawner.spawn(wifi_task(runner)).unwrap();

    let mut known_bss = heapless::Vec::new();
    let _ = sta_control.scan::<16>(None, &mut known_bss).await;
    known_bss.into_iter().for_each(print_bss);

    let net_stack_resources = mk_static!(NetStackResources<3>, NetStackResources::new());
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        net_stack_resources,
        1234,
    );
    spawner.spawn(net_task(net_runner)).unwrap();

    let bss = sta_control.find_ess(None, SSID).await.unwrap();
    let mut mac_address = [0u8; 6];
    Rng::new(peripherals.RNG).fill_bytes(mac_address.as_mut_slice());

    // sta_control.set_mac_address(mac_address).await.unwrap();
    sta_control.connect(&bss, None).await.unwrap();

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    while !net_stack.is_config_up() {
        Timer::after_millis(100).await;
    }
    info!(
        "DHCP is now up! Got address {}",
        net_stack.config_v4().unwrap().address
    );
    let dns_socket = DnsSocket::new(net_stack);
    let tcp_bin = dns_socket
        .query("tcpbin.org", DnsQueryType::A)
        .await
        .expect("DNS lookup failure")[0];
    info!("Queried tcpbin.com: {tcp_bin:?}");
    let endpoint = embassy_net::IpEndpoint {
        addr: tcp_bin,
        port: 4242,
    };
    let rx_buffer = mk_static!([u8; 1600], [0u8; 1600]);
    let tx_buffer = mk_static!([u8; 1600], [0u8; 1600]);
    let rx_meta = mk_static!([PacketMetadata; 16], [PacketMetadata::EMPTY; 16]);
    let tx_meta = mk_static!([PacketMetadata; 16], [PacketMetadata::EMPTY; 16]);
    let mut udp_socket = UdpSocket::new(net_stack, rx_meta, rx_buffer, tx_meta, tx_buffer);
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
