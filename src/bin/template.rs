#![no_std]
#![no_main]
use core::mem::MaybeUninit;

use embassy_executor::Spawner;
use embassy_net::{
    dns::{DnsQueryType, DnsSocket},
    udp::{PacketMetadata, UdpSocket},
    DhcpConfig, StackResources as NetResources,
};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};

use esp_alloc as _;
use esp_println::println;
use foa::{ScanConfig, ScanMode, StackResources as WiFiResources};
use ieee80211::mac_parser::MACAddress;
use rand_core::RngCore;

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            HEAP.as_mut_ptr() as *mut u8,
            HEAP_SIZE,
            esp_alloc::MemoryCapability::Internal.into(),
        ));
    }
}
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
#[embassy_executor::task]
async fn wifi_task(wifi_runner: foa::Runner<'static>) -> ! {
    wifi_runner.run().await
}
#[embassy_executor::task]
async fn net_task(mut net_runner: embassy_net::Runner<'static, foa::NetDriver<'static>>) -> ! {
    net_runner.run().await
}
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut rng = Rng::new(peripherals.RNG);
    let mut mac_address = [0u8; 6];
    rng.fill_bytes(mac_address.as_mut_slice());
    mac_address[0] &= !(1);

    let state = mk_static!(WiFiResources, WiFiResources::new());
    let (mut sta_control, runner, net_device) = foa::new(
        state,
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
        Some(MACAddress::new(mac_address)),
    );
    spawner.spawn(wifi_task(runner)).unwrap();

    let dhcp_config = DhcpConfig::default();
    let config = embassy_net::Config::dhcpv4(dhcp_config);
    let seed = 1234;

    let stack_resources = mk_static!(NetResources<5>, NetResources::new());
    let (stack, runner) = embassy_net::new(net_device, config, stack_resources, seed);

    spawner.spawn(net_task(runner)).unwrap();

    let ess = sta_control
        .find_ess(
            "OpenWrt",
            ScanConfig {
                scan_mode: ScanMode::Sweep,
                ..Default::default()
            },
            false,
        )
        .await
        .unwrap();
    sta_control.connect(ess).await.unwrap();
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    let dns_socket = DnsSocket::new(stack);
    let tcp_bin = dns_socket
        .query("tcpbin.org", DnsQueryType::A)
        .await
        .expect("DNS lookup failure")[0];
    println!("Queried tcpbin.com: {tcp_bin:?}");
    let endpoint = embassy_net::IpEndpoint {
        addr: tcp_bin,
        port: 4242,
    };
    let rx_buffer = mk_static!([u8; 1600], [0u8; 1600]);
    let tx_buffer = mk_static!([u8; 1600], [0u8; 1600]);
    let rx_meta = mk_static!([PacketMetadata; 16], [PacketMetadata::EMPTY; 16]);
    let tx_meta = mk_static!([PacketMetadata; 16], [PacketMetadata::EMPTY; 16]);
    let mut udp_socket = UdpSocket::new(stack, rx_meta, rx_buffer, tx_meta, tx_buffer);
    println!("Connected to tcpbin.com");
    udp_socket.bind(endpoint).unwrap();
    loop {
        udp_socket
            .send_to([0xff; 1].as_slice(), endpoint)
            .await
            .unwrap();
    }
}
