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

use core::{array, mem};

use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, DynamicReceiver},
};
use esp_config::esp_config_int;
use esp_hal::peripherals::{ADC2, WIFI};
use esp_wifi_hal::{RxFilterBank, ScanningMode, WiFi, WiFiResources};
use lmac::SharedLMacState;

#[cfg(not(feature = "arc_buffers"))]
use esp_wifi_hal::BorrowedBuffer;

#[macro_use]
extern crate defmt_or_log;

mod bg_task;
mod lmac;
#[cfg(feature = "arc_buffers")]
mod rx_arc_pool;
mod tx_buffer_management;

pub use bg_task::FoARunner;
pub use lmac::*;
#[cfg(feature = "arc_buffers")]
pub use rx_arc_pool::RxArcBuffer;
pub use tx_buffer_management::TxBuffer;

pub use esp_wifi_hal;
#[cfg(feature = "arc_buffers")]
use rx_arc_pool::RxArcPool;
use tx_buffer_management::TxBufferManager;

pub mod util;

const RX_BUFFER_COUNT: usize = esp_config_int!(usize, "FOA_CONFIG_RX_BUFFER_COUNT");
const RX_QUEUE_LEN: usize = esp_config_int!(usize, "FOA_CONFIG_RX_QUEUE_LEN");
const TX_BUFFER_COUNT: usize = esp_config_int!(usize, "FOA_CONFIG_TX_BUFFER_COUNT");
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
/// A receiver to the RX queue of an interface.
pub type RxQueueReceiver<'res> = DynamicReceiver<'res, ReceivedFrame<'res>>;

/// The resources required by the WiFi stack.
pub struct FoAResources {
    wifi_resources: WiFiResources<RX_BUFFER_COUNT>,
    #[cfg(feature = "arc_buffers")]
    arc_pool: RxArcPool,
    tx_buffers: [[u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT],
    tx_buffer_manager: Option<TxBufferManager<TX_BUFFER_COUNT>>,
    shared_lmac_state: Option<SharedLMacState>,
    rx_queues: [Channel<NoopRawMutex, ReceivedFrame<'static>, RX_QUEUE_LEN>; WiFi::INTERFACE_COUNT],
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
            tx_buffers: [[0u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT],
            tx_buffer_manager: None,
            shared_lmac_state: None,
            rx_queues: [const { Channel::new() }; WiFi::INTERFACE_COUNT],
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
    rx_queue_receiver: RxQueueReceiver<'res>,
}
impl<'res> VirtualInterface<'res> {
    /// Split the virtual interface into it's components.
    ///
    /// NOTE: This is intended for interface implementations. User code shouldn't call this,
    /// although nothing will happen.
    pub fn split<'a>(
        &'a mut self,
    ) -> (
        &'a mut LMacInterfaceControl<'res>,
        &'a mut RxQueueReceiver<'res>,
    ) {
        (&mut self.interface_control, &mut self.rx_queue_receiver)
    }
    /// Reset the virtual interface.
    ///
    /// This is releases any prior channel lock, resets all filters and clears the RX queue.
    pub fn reset(&mut self) {
        // We can't call clear on a DynamicReceiver, so this is the best we can do for now.
        // This isn't too bad, since this will only be called rarely and the RX queues shouldn't be
        // that long.
        while self.rx_queue_receiver.try_receive().is_ok() {}
        self.interface_control.unlock_channel();
        self.interface_control
            .set_scanning_mode(ScanningMode::Disabled);
        self.interface_control
            .set_filter_status(RxFilterBank::BSSID, false);
        self.interface_control
            .set_filter_status(RxFilterBank::ReceiverAddress, false);
    }
}

/// Initialise FoA.
pub fn init<'res>(
    resources: &'res mut FoAResources,
    wifi: WIFI,
    adc2: ADC2,
) -> (
    [VirtualInterface<'res>; WiFi::INTERFACE_COUNT],
    FoARunner<'res>,
) {
    // This is for all transmutes here.
    // # SAFETY:
    // We do this only to avoid self referential structs. All of the destination lifetimes are the
    // lifetime of the resources struct and therefore valid.
    let wifi = WiFi::new(wifi, adc2, &mut resources.wifi_resources);
    unsafe extern "C" {
        fn phy_set_most_tpw(power: u8);
    }

    unsafe {
        phy_set_most_tpw(84);
    }

    let shared_lmac_state = resources
        .shared_lmac_state
        .insert(SharedLMacState::new(wifi));
    let tx_buffer_manager = resources
        .tx_buffer_manager
        .insert(unsafe { TxBufferManager::new(&mut resources.tx_buffers) });
    let (lmac_receive_endpoint, lmac_interface_controls) =
        shared_lmac_state.split(tx_buffer_manager.dyn_tx_buffer_manager());
    let rx_queue_senders =
        array::from_fn(|i| unsafe { mem::transmute(resources.rx_queues[i].dyn_sender()) });
    let virtual_interfaces =
        lmac_interface_controls.map(|lmac_interface_control| VirtualInterface {
            rx_queue_receiver: unsafe {
                mem::transmute::<
                    DynamicReceiver<'_, ReceivedFrame<'static>>,
                    DynamicReceiver<'_, ReceivedFrame<'_>>,
                >(
                    resources.rx_queues[lmac_interface_control.get_filter_interface()]
                        .dyn_receiver(),
                )
            },
            interface_control: lmac_interface_control,
        });
    (
        virtual_interfaces,
        FoARunner {
            lmac_receive_endpoint,
            rx_queue_senders,
            #[cfg(feature = "arc_buffers")]
            rx_arc_pool: &resources.arc_pool,
        },
    )
}
