#![no_std]
#![no_main]

extern crate alloc;

use alloc::{format, string::String};
use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Ipv6Cidr, Runner as NetRunner, StackResources as NetStackResources, StaticConfigV6,
};
use embassy_time::Timer;
use esp_alloc::heap_allocator;
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println as _;
use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_awdl::{AwdlEvent, AwdlNetDevice, AwdlResources, AwdlRunner};
use reqwless::{client::HttpClient, request::Method};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}
#[embassy_executor::task]
async fn foa_task(mut foa_runner: FoARunner<'static>) -> ! {
    foa_runner.run().await
}
#[embassy_executor::task]
async fn awdl_task(mut awdl_runner: AwdlRunner<'static, 'static>) -> ! {
    awdl_runner.run().await
}
#[embassy_executor::task]
async fn net_task(mut net_runner: NetRunner<'static, AwdlNetDevice<'static>>) -> ! {
    net_runner.run().await
}
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    heap_allocator!(size: 10 * 1024);
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let stack_resources = mk_static!(FoAResources, FoAResources::new());
    let ([awdl_vif, ..], foa_runner) = foa::init(
        stack_resources,
        peripherals.WIFI,
        peripherals.RADIO_CLK,
        peripherals.ADC2,
    );
    spawner.spawn(foa_task(foa_runner)).unwrap();

    let awdl_resources = mk_static!(AwdlResources, AwdlResources::new());
    let (mut awdl_control, awdl_runner, net_device, awdl_event_queue_rx) =
        foa_awdl::new_awdl_interface(
            mk_static!(VirtualInterface<'static>, awdl_vif),
            awdl_resources,
            Rng::new(peripherals.RNG),
        );
    spawner.spawn(awdl_task(awdl_runner)).unwrap();
    awdl_control.randomize_mac_address();
    awdl_control.start().await.unwrap();

    let (airplay_peer, airplay_port) = loop {
        if let AwdlEvent::DiscoveredServiceForPeer {
            address,
            airplay_port: Some(airplay_port),
            ..
        } = awdl_event_queue_rx.receive().await
        {
            break (foa_awdl::hw_address_to_ipv6(&address), airplay_port);
        }
    };
    let net_stack_resources = mk_static!(NetStackResources<3>, NetStackResources::new());
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::ipv6_static(StaticConfigV6 {
            address: Ipv6Cidr::new(awdl_control.own_ipv6_addr(), 10),
            gateway: None,
            dns_servers: heapless::Vec::new(),
        }),
        net_stack_resources,
        1234,
    );
    spawner.spawn(net_task(net_runner)).unwrap();
    let tcp_client_state = mk_static!(TcpClientState<2, 1400, 1400>, TcpClientState::new());
    let tcp_client = TcpClient::new(net_stack, tcp_client_state);
    /*
        loop {
            let _ = tcp_socket.write("FCK-APL".as_bytes()).await;
            Timer::after_secs(1).await;
        }
    */

    let dns_client = DnsSocket::new(net_stack);
    let mut http_client = HttpClient::new(&tcp_client, &dns_client);

    let rx_buf = mk_static!([u8; 8192], [0; 8192]);

    let url = format!("http://[{airplay_peer}]:{airplay_port}/info");
    loop {
        let mut request = http_client
            .request(Method::GET, url.as_str())
            .await
            .unwrap();
        let response = request.send(rx_buf).await.unwrap();
        let response_body = response.body().read_to_end().await.unwrap();
        info!("{}", String::from_utf8_lossy(response_body).as_ref());
        Timer::after_secs(5).await;
    }
}
