#![no_std]
#![deny(missing_docs)]
//! # Ferris-on-Air (FoA)
//! Ferris-on-Air is an asynchronous IEEE 802.11 MAC stack for the ESP32 series of chips. It is
//! build on top of [esp_wifi_hal], which is the driver for the Wi-Fi peripheral, and is based on
//! the reverse engineering efforts of the [ESP32-Open-MAC project](https://esp32-open-mac.be/).
//! ## Note:
//! This project is neither maintained by nor in anyway affiliated with Espressif. You're using
//! this at your own risk!
//! ## Structure
//! The `foa` crate is the central element of the stack. It doesn't implement any specific
//! operating modes, but implements the APIs to do so. When calling [init], you get a
//! [FoARunner] and a set of [VirtualInterfaces](VirtualInterface), which can then be passed on to
//! interface implementations.
//!
//! Currently, the following operating modes have an interface implemented for them.
//!
//! Operating Mode | Implementation | Description | Maintainer
//! -- | -- | -- | --
//! STA | [foa_sta](https://github.com/esp32-open-mac/FoA/tree/main/foa_sta) | A simple client implementation. | `Frostie314159`
//! Nintendo DS Pictochat | [foa_dswifi](https://github.com/mjwells2002/foa_dswifi) | An implementation of the Nintendo DS Pictochat protocol. | `mjwells2002`
//! AWDL | [foa_awdl](https://github.com/esp32-open-mac/FoA/tree/main/foa_awdl) | An implementation of the Apple Wireless Direct Link protocol | `Frostie314159`
//!
//!
//! ### Station (STA)
//! A simple STA mode interface is implemented in [foa_sta](https://github.com/esp32-open-mac/FoA/tree/main/foa_sta).
//! For details on the supported features, please check the documentation of `foa_sta`.

use core::{array, mem, sync::atomic::Ordering};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, ReceiveFuture},
    mutex::Mutex,
};
use esp_config::esp_config_int;
use esp_hal::peripherals::WIFI;
use esp_wifi_hal::prelude::*;
use lmac::SharedLMacState;

#[macro_use]
extern crate defmt_or_log;

mod bg_task;
mod lmac;
#[cfg(feature = "arc_buffers")]
mod rx_arc_pool;
mod tx_buffer_management;
mod tx_queue;

pub use bg_task::FoARunner;
pub use lmac::*;
use portable_atomic::AtomicBool;
#[cfg(feature = "arc_buffers")]
pub use rx_arc_pool::RxArcBuffer;
pub use tx_buffer_management::TxBuffer;
pub use tx_queue::{
    PendingTransmission, PendingTransmissionStatus, RetryBehaviour, TxEndpoint, TxReturnData,
};

pub use esp_wifi_hal;
#[cfg(feature = "arc_buffers")]
use rx_arc_pool::RxArcPool;
use tx_buffer_management::TxBufferManager;

use crate::{
    bg_task::{RxRunner, TxRunner},
    tx_queue::{TxQueue, TxQueueRunner},
};

pub mod util;

/// Number of buffers used for the hardware RX queue.
pub const RX_BUFFER_COUNT: usize = esp_config_int!(usize, "FOA_CONFIG_RX_BUFFER_COUNT");
/// Length of the interface RX queue.
pub const RX_QUEUE_LEN: usize = esp_config_int!(usize, "FOA_CONFIG_RX_QUEUE_LEN");
/// Number of buffers preallocated for TX.
pub const TX_BUFFER_COUNT: usize = esp_config_int!(usize, "FOA_CONFIG_TX_BUFFER_COUNT");
/// The size of a [TxBuffer].
///
/// This is fixed, so that interfaces can rely on the size of a TX buffer.
pub const TX_BUFFER_SIZE: usize = 1600;

/// A frame received from the driver.
#[cfg(feature = "arc_buffers")]
pub type ReceivedFrame<'res> = RxArcBuffer<'res>;
#[cfg(not(feature = "arc_buffers"))]
/// A frame received from the driver.
pub type ReceivedFrame<'res> = BorrowedBuffer<'res>;

/// An endpoint to an RX queue.
///
/// As long as this is alive, the rx queue will be active.
pub struct RxEndpoint<'res, 'a> {
    rx_queue: &'a (
        Channel<NoopRawMutex, ReceivedFrame<'res>, RX_QUEUE_LEN>,
        AtomicBool,
    ),
}
impl<'res, 'a> RxEndpoint<'res, 'a> {
    fn new(
        rx_queue: &'a (
            Channel<NoopRawMutex, ReceivedFrame<'res>, RX_QUEUE_LEN>,
            AtomicBool,
        ),
    ) -> Self {
        rx_queue.1.store(true, Ordering::Relaxed);
        Self { rx_queue }
    }
    /// Receive a frame.
    pub fn receive(
        &mut self,
    ) -> ReceiveFuture<'_, NoopRawMutex, ReceivedFrame<'res>, RX_QUEUE_LEN> {
        self.rx_queue.0.receive()
    }
}
impl Drop for RxEndpoint<'_, '_> {
    fn drop(&mut self) {
        self.rx_queue.1.store(false, Ordering::Relaxed);
    }
}

/// The resources required by the WiFi stack.
pub struct FoAResources {
    /// Resources for the Wi-Fi driver.
    wifi_resources: WiFiResources<RX_BUFFER_COUNT>,
    /// State for the LMAC.
    shared_lmac_state: Option<SharedLMacState>,

    // RX
    /// RX Queues for all interfaces.
    ///
    /// The [AtomicBool] indicates, whether this queue is active.
    rx_queues: [(
        Channel<NoopRawMutex, ReceivedFrame<'static>, RX_QUEUE_LEN>,
        AtomicBool,
    ); INTERFACE_COUNT],
    #[cfg(feature = "arc_buffers")]
    /// Atomically reference counted buffer pool.
    arc_pool: RxArcPool,

    // TX
    /// All EDCA TX queues.
    edca_tx_queues: Option<[TxQueue; 4]>,
    /// Beacon TX is handled differently, since it's essentially always bound to stricter timing
    /// requirements.
    beacon_tx_queue: Option<Mutex<NoopRawMutex, TxQueueEndpoint<'static>>>,
    /// TX buffers used by the [TxBufferManager].
    tx_buffers: [[u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT],
    /// The aforementioned TX buffer manager.
    tx_buffer_manager: Option<TxBufferManager<'static>>,
}
impl FoAResources {
    /// Create new stack resources.
    ///
    /// This has to be in internal RAM, since the DMA descriptors and buffers for the Wi-Fi driver
    /// have to be in internal RAM.
    pub fn new() -> Self {
        Self {
            wifi_resources: WiFiResources::new(),
            #[cfg(feature = "arc_buffers")]
            arc_pool: RxArcPool::new(),
            rx_queues: [const { (Channel::new(), AtomicBool::new(false)) }; INTERFACE_COUNT],
            shared_lmac_state: None,

            edca_tx_queues: None,
            beacon_tx_queue: None,
            tx_buffers: [[0u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT],
            tx_buffer_manager: None,
        }
    }
}
impl Default for FoAResources {
    fn default() -> Self {
        Self::new()
    }
}

/// A virtual interface (VIF).
///
/// This is intended to be used by interface implementations, which should take in a mutable
/// reference to a VIF.
pub struct VirtualInterface<'res> {
    interface_control: LMacInterfaceControl<'res>,
    rx_queue: &'res (
        Channel<NoopRawMutex, ReceivedFrame<'res>, RX_QUEUE_LEN>,
        AtomicBool,
    ),
    tx_endpoint: TxEndpoint<'res>,
}
impl<'res> VirtualInterface<'res> {
    /// Split the virtual interface into it's components.
    ///
    /// NOTE: This is intended for interface implementations. User code shouldn't call this,
    /// although nothing will happen.
    ///
    /// The RX endpoint is passed by value, as it controls whether a queue is active or not.
    pub fn split<'a>(
        &'a mut self,
    ) -> (
        &'a mut LMacInterfaceControl<'res>,
        RxEndpoint<'res, 'a>,
        &'a mut TxEndpoint<'res>,
    ) {
        (
            &mut self.interface_control,
            RxEndpoint::new(self.rx_queue),
            &mut self.tx_endpoint,
        )
    }
    /// Reset the virtual interface.
    ///
    /// This is releases any prior channel lock, resets all filters and clears the RX queue.
    pub fn reset(&mut self) {
        self.rx_queue.0.clear();
        self.interface_control.unlock_channel();
        self.interface_control
            .set_scanning_mode(ScanningMode::Disabled);
        self.interface_control.clear_filter(RxFilterBank::Bssid);
        self.interface_control
            .clear_filter(RxFilterBank::ReceiverAddress);
    }
}

/// Initialise FoA.
pub fn init<'res>(
    resources: &'res mut FoAResources,
    wifi: WIFI<'res>,
) -> ([VirtualInterface<'res>; INTERFACE_COUNT], FoARunner<'res>) {
    let wifi = WiFi::new(wifi, &mut resources.wifi_resources);

    let SplitDriverComponents {
        rx_interface_controllers,
        rx_endpoint,
        channel_controller,
        crypto_controller,
        tx_queue_endpoints: [beacon_tx_endpoint, edca_tx_endpoints @ ..],
    } = wifi.split();

    // This is for all transmutes here.
    // # SAFETY:
    // We do this only to avoid self referential structs. All of the destination lifetimes are the
    // lifetime of the resources struct and therefore valid.

    // Initialize global state
    let shared_lmac_state = resources
        .shared_lmac_state
        .insert(SharedLMacState::new(channel_controller, crypto_controller));
    let tx_buffer_manager = unsafe {
        core::mem::transmute::<
            &mut Option<TxBufferManager<'static>>,
            &mut Option<TxBufferManager<'res>>,
        >(&mut resources.tx_buffer_manager)
    }
    .insert(TxBufferManager::new(&mut resources.tx_buffers));
    let lmac_interface_controls = shared_lmac_state.split(rx_interface_controllers);
    let rx_queue_senders = array::from_fn(|i| {
        (
            unsafe {
                mem::transmute::<
                    embassy_sync::channel::DynamicSender<
                        'res,
                        esp_wifi_hal::borrowed_buffer::BorrowedBuffer<'static>,
                    >,
                    embassy_sync::channel::DynamicSender<
                        'res,
                        esp_wifi_hal::borrowed_buffer::BorrowedBuffer<'res>,
                    >,
                >(resources.rx_queues[i].0.dyn_sender())
            },
            &resources.rx_queues[i].1,
        )
    });
    // TX queue setup
    let beacon_tx_queue = resources.beacon_tx_queue.insert(Mutex::new(unsafe {
        core::mem::transmute::<
            esp_wifi_hal::async_driver::TxQueueEndpoint<'res>,
            esp_wifi_hal::async_driver::TxQueueEndpoint<'static>,
        >(beacon_tx_endpoint)
    }));
    let edca_tx_queues = resources
        .edca_tx_queues
        .insert([const { TxQueue::new() }; 4]);
    let edca_tx_queues = unsafe {
        core::mem::transmute::<&mut [TxQueue; 4], &'res mut [TxQueue; 4]>(edca_tx_queues)
    };

    let virtual_interfaces = unsafe {
        lmac_interface_controls.map(|lmac_interface_control| VirtualInterface {
            rx_queue: mem::transmute::<
                &'res (
                    embassy_sync::channel::Channel<
                        embassy_sync::blocking_mutex::raw::NoopRawMutex,
                        esp_wifi_hal::borrowed_buffer::BorrowedBuffer<'static>,
                        2,
                    >,
                    portable_atomic::AtomicBool,
                ),
                &'res (
                    embassy_sync::channel::Channel<
                        embassy_sync::blocking_mutex::raw::NoopRawMutex,
                        esp_wifi_hal::borrowed_buffer::BorrowedBuffer<'res>,
                        2,
                    >,
                    portable_atomic::AtomicBool,
                ),
            >(&resources.rx_queues[lmac_interface_control.interface()]),
            tx_endpoint: mem::transmute::<tx_queue::TxEndpoint<'_>, tx_queue::TxEndpoint<'res>>(
                TxEndpoint {
                    beacon_tx_endpoint: beacon_tx_queue,
                    dyn_tx_buffer_manager: tx_buffer_manager,
                    edca_tx_endpoints: edca_tx_queues.each_ref(),
                    interface: lmac_interface_control.interface(),
                },
            ),
            interface_control: lmac_interface_control,
        })
    };

    (
        virtual_interfaces,
        FoARunner {
            tx_runner: TxRunner {
                edca_tx_runners: edca_tx_endpoints.map(|tx_endpoint| TxQueueRunner {
                    tx_queue: &edca_tx_queues[tx_endpoint.hardware_tx_queue().hardware_slot() - 1],
                    tx_endpoint,
                }),
            },
            rx_runner: RxRunner {
                rx_endpoint,
                rx_queue_senders,
                #[cfg(feature = "arc_buffers")]
                rx_arc_pool: &resources.arc_pool,
            },
        },
    )
}
