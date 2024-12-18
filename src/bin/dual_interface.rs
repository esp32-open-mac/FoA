#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::dns::DnsSocket;
use embassy_net::udp::PacketMetadata;
use embassy_net::udp::UdpSocket;
use embassy_net::DhcpConfig;
use embassy_net::Runner as NetRunner;
use embassy_net::StackResources as NetStackResources;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use foa::{
    bg_task::MultiInterfaceRunner,
    sta::{StaInitInfo, StaInterface, StaNetDevice, StaSharedResources},
    FoAStackResources,
};
use log::info;
use rand_core::RngCore;

const SSID_ZERO: &str = "Freifunk";
const SSID_ONE: &str = "OpenWrt";

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
#[embassy_executor::task]
async fn wifi_task(
    mut wifi_runner: MultiInterfaceRunner<'static, StaInterface, StaInterface>,
) -> ! {
    wifi_runner.run().await
}
#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut net_runner: NetRunner<'static, StaNetDevice<'static>>) -> ! {
    net_runner.run().await
}
async fn run_net_stack(
    spawner: &Spawner,
    net_stack_resources: &'static mut NetStackResources<3>,
    net_device: StaNetDevice<'static>,
) {
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        net_stack_resources,
        1234,
    );
    spawner.spawn(net_task(net_runner)).unwrap();
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
        .query("tcpbin.org", embassy_net::dns::DnsQueryType::A)
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
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let stack_resources = mk_static!(FoAStackResources<StaSharedResources, StaSharedResources>, FoAStackResources::new());
    let (
        (mut sta_control_zero, net_device_zero),
        (mut sta_control_one, net_device_one),
        wifi_runner,
    ) = foa::new_with_multiple_interfaces::<StaInterface, StaInterface>(
        stack_resources,
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        StaInitInfo,
        StaInitInfo,
    )
    .await;
    spawner.spawn(wifi_task(wifi_runner)).unwrap();

    let mut mac_address = [0u8; 6];
    let mut rng = Rng::new(peripherals.RNG);
    rng.fill_bytes(mac_address.as_mut_slice());
    sta_control_zero.set_mac_address(mac_address).await.unwrap();
    rng.fill_bytes(mac_address.as_mut_slice());
    sta_control_one.set_mac_address(mac_address).await.unwrap();

    let mut known_bss = heapless::Vec::new();
    sta_control_zero
        .scan::<16>(None, &mut known_bss)
        .await
        .unwrap();

    let (zero, one) = join(
        sta_control_zero.connect(
            known_bss.iter().find(|bss| bss.ssid == SSID_ZERO).unwrap(),
            None,
        ),
        sta_control_one.connect(
            known_bss.iter().find(|bss| bss.ssid == SSID_ONE).unwrap(),
            None,
        ),
    )
    .await;
    zero.unwrap();
    one.unwrap();
    join(
        run_net_stack(
            &spawner,
            mk_static!(NetStackResources<3>, NetStackResources::new()),
            net_device_zero,
        ),
        run_net_stack(
            &spawner,
            mk_static!(NetStackResources<3>, NetStackResources::new()),
            net_device_one,
        ),
    )
    .await;
}
