#![no_std]
#![no_main]

use core::ffi::CStr;

use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    Ipv6Cidr, Runner as NetRunner, StackResources as NetStackResources, StaticConfigV6,
};
use esp_backtrace as _;
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println as _;
use foa::{FoAResources, FoARunner, VirtualInterface};
use foa_awdl::{AwdlEvent, AwdlNetDevice, AwdlResources, AwdlRunner};
use reqwless::client::{HttpClient, TlsConfig, TlsVerify};

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

    let rng = Rng::new(peripherals.RNG);

    let awdl_resources = mk_static!(AwdlResources, AwdlResources::new());
    let (mut awdl_control, awdl_runner, net_device, awdl_event_queue_rx) =
        foa_awdl::new_awdl_interface(
            mk_static!(VirtualInterface<'static>, awdl_vif),
            awdl_resources,
            rng,
        );
    spawner.spawn(awdl_task(awdl_runner)).unwrap();
    awdl_control.randomize_mac_address();
    awdl_control.start().await.unwrap();

    // We wait for an AirDrop peer to be discovered.
    let (airdrop_peer, airdrop_port) = loop {
        if let AwdlEvent::DiscoveredServiceForPeer {
            address,
            airdrop_port: Some(airdrop_port),
            ..
        } = awdl_event_queue_rx.receive().await
        {
            break (foa_awdl::hw_address_to_ipv6(&address), airdrop_port);
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
    let tcp_client_state = mk_static!(TcpClientState<2, 2000, 2000>, TcpClientState::new());
    let tcp_client = TcpClient::new(net_stack, tcp_client_state);

    let dns_client = DnsSocket::new(net_stack);

    const TLS_BUFFER_SIZE: usize = 16640;

    let (tls_read_buf, tls_write_buf) = (
        mk_static!([u8; TLS_BUFFER_SIZE], [0u8; TLS_BUFFER_SIZE]),
        mk_static!([u8; TLS_BUFFER_SIZE], [0u8; TLS_BUFFER_SIZE]),
    );
    let mut seed = [0u8; 8];
    rng.clone().read(seed.as_mut_slice());
    let tls_config = TlsConfig::new(
        u64::from_le_bytes(seed),
        tls_read_buf.as_mut_slice(),
        tls_write_buf.as_mut_slice(),
        TlsVerify::None,
    );
    let mut buf = [0u8; 100];

    use embedded_io::Write;
    core::write!(
        buf.as_mut_slice(),
        "https://[{airdrop_peer}]:{airdrop_port}/"
    )
    .unwrap();
    let url = CStr::from_bytes_until_nul(buf.as_slice()).unwrap();
    let url = url.to_str().unwrap();
    let mut http_client = HttpClient::new_with_tls(&tcp_client, &dns_client, tls_config);

    let _resource = http_client.resource(url).await;
}
