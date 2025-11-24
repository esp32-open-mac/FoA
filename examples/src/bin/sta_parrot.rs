#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    DhcpConfig, Runner as NetRunner, StackResources as NetStackResources,
};
use embassy_time::Timer;
use embedded_io_async::Read;
use esp_backtrace as _;
use esp_hal::{
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{self, Uart},
    Async,
};
use esp_println as _;
use foa::{
    FoAResources, FoARunner, VirtualInterface,
};
use foa_sta::{Credentials, StaNetDevice, StaResources, StaRunner};
use reqwless::{client::HttpClient, request::Method, response::BodyReader};

const SSID: &str = env!("SSID");

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
async fn sta_task(mut sta_runner: StaRunner<'static, 'static>) -> ! {
    sta_runner.run().await
}
#[embassy_executor::task]
async fn net_task(mut net_runner: NetRunner<'static, StaNetDevice<'static>>) -> ! {
    net_runner.run().await
}
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_bootloader_esp_idf::esp_app_desc!();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let stack_resources = mk_static!(FoAResources, FoAResources::new());
    let ([sta_vif, ..], foa_runner) = foa::init(
        stack_resources,
        peripherals.WIFI,
        peripherals.ADC2,
    );
    spawner.spawn(foa_task(foa_runner)).unwrap();

    let sta_resources = mk_static!(StaResources<'static>, StaResources::default());
    let (mut sta_control, sta_runner, net_device) = foa_sta::new_sta_interface(
        mk_static!(VirtualInterface<'static>, sta_vif),
        sta_resources,
        Rng::new(),
    );
    spawner.spawn(sta_task(sta_runner)).unwrap();

    let _ = sta_control.randomize_mac_address();

    let net_stack_resources = mk_static!(NetStackResources<3>, NetStackResources::new());
    let (net_stack, net_runner) = embassy_net::new(
        net_device,
        embassy_net::Config::dhcpv4(DhcpConfig::default()),
        net_stack_resources,
        1234,
    );
    spawner.spawn(net_task(net_runner)).unwrap();

    defmt::unwrap!(
        sta_control
            .connect_by_ssid(SSID, None, Some(Credentials::Passphrase(env!("PASSWORD"))))
            .await
    );

    info!("Connected to {}.", SSID);

    net_stack.wait_config_up().await;
    info!(
        "DHCP: Got address {}.",
        net_stack.config_v4().unwrap().address
    );

    let client_state = mk_static!(TcpClientState<4, 1500, 1500>, TcpClientState::new());
    let tcp_client = TcpClient::new(net_stack, client_state);
    let dns_client = DnsSocket::new(net_stack);
    let mut http_client = HttpClient::new(&tcp_client, &dns_client);

    let rx_buf = mk_static!([u8; 8192], [0; 8192]);

    let parrot_buffer = mk_static!([u8; 1119], [0u8; 1119]);
    #[cfg(feature = "esp32")]
    let (rx_pin, tx_pin) = (peripherals.GPIO3, peripherals.GPIO1);
    #[cfg(feature = "esp32s2")]
    let (rx_pin, tx_pin) = (peripherals.GPIO44, peripherals.GPIO43);
    let mut uart = Uart::new(peripherals.UART0, uart::Config::default())
        .unwrap()
        .with_rx(rx_pin)
        .with_tx(tx_pin)
        .into_async();
    defmt::flush();
    loop {
        let mut request = http_client
            .request(Method::GET, "http://parrot.live/")
            .await
            .unwrap();
        let response = request.send(rx_buf).await.unwrap();
        let BodyReader::Chunked(mut chunked_reader) = response.body().reader() else {
            panic!()
        };

        loop {
            let Ok(_) = chunked_reader.read_exact(parrot_buffer).await else {
                break;
            };
            let _ = <Uart<'static, Async> as embedded_io_async::Write>::write_all(
                &mut uart,
                parrot_buffer,
            )
            .await;
            let _ = uart.flush();
        }
    }
}
