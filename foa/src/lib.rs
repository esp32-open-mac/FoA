#![no_std]
#![allow(async_fn_in_trait)]

use core::{array, mem};

use bg_task::FoARunner;
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex,
    channel::{Channel, DynamicReceiver},
};
use esp_config::esp_config_int;
use esp_hal::peripherals::{ADC2, RADIO_CLK, WIFI};
use esp_wifi_hal::{BorrowedBuffer, DMAResources, RxFilterBank, ScanningMode, WiFi};
use lmac::{LMacInterfaceControl, SharedLMacState};

pub mod bg_task;
pub mod lmac;
// mod rx_arc_pool;
pub mod tx_buffer_management;

pub use esp_wifi_hal;
use tx_buffer_management::TxBufferManager;

const RX_BUFFER_COUNT: usize = esp_config_int!(usize, "FOA_CONFIG_RX_BUFFER_COUNT");
const RX_BUFFER_SIZE: usize = esp_config_int!(usize, "FOA_CONFIG_RX_BUFFER_SIZE");
const RX_QUEUE_LEN: usize = esp_config_int!(usize, "FOA_CONFIG_RX_QUEUE_LEN");
const TX_BUFFER_COUNT: usize = esp_config_int!(usize, "FOA_CONFIG_TX_BUFFER_COUNT");
const TX_BUFFER_SIZE: usize = esp_config_int!(usize, "FOA_CONFIG_TX_BUFFER_SIZE");

/// A frame received from the driver.
pub type ReceivedFrame<'res> = BorrowedBuffer<'res>;
/// A receiver to the RX queue of an interface.
pub type RxQueueReceiver<'res> = DynamicReceiver<'res, ReceivedFrame<'res>>;

/// The resources required by the WiFi stack.
pub struct FoAResources {
    dma_resources: DMAResources<RX_BUFFER_SIZE, RX_BUFFER_COUNT>,
    tx_buffers: [[u8; TX_BUFFER_SIZE]; TX_BUFFER_COUNT],
    tx_buffer_manager: Option<TxBufferManager<TX_BUFFER_COUNT>>,
    shared_lmac_state: Option<SharedLMacState>,
    rx_queues: [Channel<NoopRawMutex, ReceivedFrame<'static>, RX_QUEUE_LEN>; WiFi::INTERFACE_COUNT],
}
impl FoAResources {
    pub fn new() -> Self {
        Self {
            dma_resources: DMAResources::new(),
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
pub fn init(
    resources: &mut FoAResources,
    wifi: WIFI,
    radio_clock: RADIO_CLK,
    adc2: ADC2,
) -> ([VirtualInterface<'_>; WiFi::INTERFACE_COUNT], FoARunner<'_>) {
    // This is for all transmutes here.
    // # SAFETY:
    // We do this only to avoid self referential structs. All of the destination lifetimes are the
    // lifetime of the resources struct and therefore valid.
    let wifi = WiFi::new(wifi, radio_clock, adc2, &mut resources.dma_resources);
    resources.shared_lmac_state = Some(SharedLMacState::new(wifi));
    resources.tx_buffer_manager = Some(unsafe { TxBufferManager::new(&mut resources.tx_buffers) });
    let (lmac_receive_endpoint, lmac_interface_controls) =
        resources.shared_lmac_state.as_mut().unwrap().split(
            resources
                .tx_buffer_manager
                .as_ref()
                .unwrap()
                .dyn_tx_buffer_manager(),
        );
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
        },
    )
}
