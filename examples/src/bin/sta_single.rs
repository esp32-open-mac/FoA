#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;
use embassy_net::{
    dns::{DnsQueryType, DnsSocket},
    udp::{PacketMetadata, UdpSocket},
    DhcpConfig, Runner as NetRunner, StackResources as NetStackResources,
};
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
#[embassy_executor::task]
async fn net_task(mut net_runner: NetRunner<'static, StaNetDevice<'static>>) -> ! {
    net_runner.run().await
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
    let (mut sta_control, sta_runner, net_device) = foa_sta::new_sta_interface(
        mk_static!(VirtualInterface<'static>, sta_vif),
        sta_resources,
        None,
    );
    spawner.spawn(sta_task(sta_runner)).unwrap();

    let mut mac_address = [0u8; 6];
    Rng::new(peripherals.RNG).fill_bytes(mac_address.as_mut_slice());
    sta_control.set_mac_address(mac_address).await.unwrap();

    let net_stack_resources = mk_static!(NetStackResources<3>, NetStackResources::new());
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        net_stack_resources,
        1234,
    );
    spawner.spawn(net_task(net_runner)).unwrap();

    let bss = sta_control.find_ess(None, SSID).await.unwrap();

    // sta_control.set_mac_address(mac_address).await.unwrap();
    sta_control.connect(&bss, None).await.unwrap();

    // Wait for DHCP, not necessary when using static IP
    info!("waiting for DHCP...");
    net_stack.wait_config_up().await;
    info!(
        "DHCP is now up! Got address {:?}",
        net_stack.config_v4().unwrap().address
    );
    let dns_socket = DnsSocket::new(net_stack);
    let tcp_bin = dns_socket
        .query("tcpbin.org", DnsQueryType::A)
        .await
        .expect("DNS lookup failure")[0];
    info!("Queried tcpbin.com: {:?}", tcp_bin);
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
