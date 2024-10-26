#![no_std]
#![no_main]
use core::{mem::MaybeUninit, str::from_utf8};

use embassy_executor::Spawner;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    DhcpConfig, Stack, StackResources,
};
use embassy_time::{Duration, Timer};
use embedded_nal_async::TcpConnect;
use esp_backtrace as _;
use esp_hal::timer::timg::TimerGroup;

use esp_alloc as _;
use esp_println::println;
use foa::{new, NetDriver, Runner, ScanConfig, ScanMode, State};
use ieee80211::mac_parser::MACAddress;
use log::{error, info};
use reqwless::{
    client::{HttpClient, TlsConfig, TlsVerify},
    request::Method,
};
use serde::Deserialize;

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
async fn wifi_task(wifi_runner: Runner<'static>) -> ! {
    wifi_runner.run().await
}
#[embassy_executor::task]
async fn net_task(mut stack: embassy_net::Runner<'static, NetDriver<'static>>) -> ! {
    stack.run().await
}
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    init_heap();
    esp_println::logger::init_logger_from_env();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    let state = mk_static!(
        State,
        State::new(
            peripherals.WIFI,
            peripherals.RADIO_CLK,
            peripherals.ADC2,
            Some(MACAddress::new([0x00, 0x80, 0x41, 0x13, 0x37, 0x41])),
        )
    );
    let (mut sta_control, runner, net_device) = new(state);
    spawner.spawn(wifi_task(runner)).unwrap();

    let mut hostname = heapless::String::new();
    let _ = hostname.push_str("FOA");
    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(hostname);
    let config = embassy_net::Config::dhcpv4(dhcp_config);
    let seed = 1234;

    let stack_resources = mk_static!(StackResources<5>, StackResources::new());
    let (stack, runner) = embassy_net::new(net_device, config, stack_resources, seed);

    spawner.spawn(net_task(runner)).unwrap();

    let freifunk = sta_control
        .find_ess(
            "OpenWrt",
            ScanConfig {
                scan_mode: ScanMode::Sweep,
                ..Default::default()
            },
            true,
        )
        .await
        .unwrap();
    sta_control.connect(freifunk).await.unwrap();
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
    loop {
        let mut rx_buffer = [0; 8192];

        let client_state = TcpClientState::<1, 1024, 1024>::new();
        let tcp_client = TcpClient::new(stack, &client_state);
        let dns_client = DnsSocket::new(stack);

        let mut http_client = HttpClient::new(&tcp_client, &dns_client);
        let url = "http://worldtimeapi.org/api/timezone/Europe/Berlin";

        info!("connecting to {}", &url);

        let mut request = match http_client.request(Method::GET, &url).await {
            Ok(req) => req,
            Err(e) => {
                error!("Failed to make HTTP request: {:?}", e);
                return; // handle the error
            }
        };

        let response = match request.send(&mut rx_buffer).await {
            Ok(resp) => resp,
            Err(_e) => {
                error!("Failed to send HTTP request");
                return; // handle the error;
            }
        };

        let body = match from_utf8(response.body().read_to_end().await.unwrap()) {
            Ok(b) => b,
            Err(_e) => {
                error!("Failed to read response body");
                return; // handle the error
            }
        };
        info!("Response body: {:?}", &body);

        // parse the response body and update the RTC

        #[derive(Deserialize)]
        struct ApiResponse<'a> {
            datetime: &'a str,
            // other fields as needed
        }

        let bytes = body.as_bytes();
        match serde_json_core::de::from_slice::<ApiResponse>(bytes) {
            Ok((output, _used)) => {
                info!("Datetime: {:?}", output.datetime);
            }
            Err(_e) => {
                error!("Failed to parse response body");
                return; // handle the error
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}
