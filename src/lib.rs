#![doc = include_str!("../README.md")]
#![no_std]
#![allow(async_fn_in_trait)]

use bg_task::{MultiInterfaceRunner, SingleInterfaceRunner};
use esp32_wifi_hal_rs::{DMAResources, WiFi};
use esp_hal::{
    efuse::Efuse,
    peripherals::{ADC2, RADIO_CLK, WIFI},
};
use interface::Interface;
use lmac::SharedLMacState;
use log::debug;
use tx_buffer_management::TxBufferManager;

pub mod bg_task;
pub mod interface;
pub mod lmac;
pub mod sta;
pub mod tx_buffer_management;

/// The resources required by the WiFi stack.
pub struct FoAStackResources<'a, IfZeroResources: Default, IfOneResources: Default = ()> {
    dma_resources: DMAResources<1600, 10>,
    tx_resources: [[u8; 1600]; 10],
    tx_buffer_manager: Option<TxBufferManager<'a, 1600, 10>>,
    shared_lmac_state: Option<SharedLMacState<'a>>,
    if_zero_resources: IfZeroResources,
    if_one_resources: IfOneResources,
}
impl<IfZeroResources: Default, IfOneResources: Default>
    FoAStackResources<'_, IfZeroResources, IfOneResources>
{
    pub fn new() -> Self {
        Self {
            dma_resources: DMAResources::new(),
            tx_resources: [[0u8; 1600]; 10],
            tx_buffer_manager: None,
            shared_lmac_state: None,
            if_zero_resources: IfZeroResources::default(),
            if_one_resources: IfOneResources::default(),
        }
    }
}
impl<IfZeroResources: Default, IfOneResources: Default> Default
    for FoAStackResources<'_, IfZeroResources, IfOneResources>
{
    fn default() -> Self {
        Self::new()
    }
}

/// Initialize FoA with two interfaces.
pub async fn new_with_multiple_interfaces<'res, IfZero: Interface, IfOne: Interface>(
    resources: &'res mut FoAStackResources<
        'res,
        IfZero::SharedResourcesType<'res>,
        IfOne::SharedResourcesType<'res>,
    >,
    wifi: WIFI,
    radio_clock: RADIO_CLK,
    adc2: ADC2,
    if_zero_init_info: IfZero::InitInfo,
    if_one_init_info: IfOne::InitInfo,
) -> (
    IfZero::ControlType<'res>,
    IfOne::ControlType<'res>,
    MultiInterfaceRunner<'res, IfZero, IfOne>,
) {
    resources.shared_lmac_state = Some(SharedLMacState::new(WiFi::new(
        wifi,
        radio_clock,
        adc2,
        &mut resources.dma_resources,
    )));
    resources.tx_buffer_manager = Some(TxBufferManager::new(&mut resources.tx_resources));
    let tx_buffer_manager = resources.tx_buffer_manager.as_mut().unwrap();
    let (receive_endpoint, transmit_endpoint, if_zero_lmac_control, if_one_lmac_control) =
        resources
            .shared_lmac_state
            .as_mut()
            .unwrap()
            .split(tx_buffer_manager.dyn_tx_buffer_manager());
    let mut mac_address = Efuse::read_base_mac_address();
    let (if_zero_control, if_zero_runner, if_zero_input) = IfZero::new(
        &mut resources.if_zero_resources,
        if_zero_init_info,
        transmit_endpoint,
        if_zero_lmac_control,
        mac_address,
    )
    .await;

    // The MAC address for the second interface is the same as the first interface, only with two
    // added to the first octet.
    mac_address[5] += 2;
    let (if_one_control, if_one_runner, if_one_input) = IfOne::new(
        &mut resources.if_one_resources,
        if_one_init_info,
        transmit_endpoint,
        if_one_lmac_control,
        mac_address,
    )
    .await;
    debug!(
        "Initialized Wi-Fi stack with {} and {} interfaces.",
        IfZero::NAME,
        IfOne::NAME
    );
    (
        if_zero_control,
        if_one_control,
        MultiInterfaceRunner {
            receive_endpoint,
            if_zero_runner,
            if_zero_input,
            if_one_runner,
            if_one_input,
        },
    )
}
/// Initialize FoA with a single interface.
pub async fn new_with_single_interface<'res, If: Interface>(
    resources: &'res mut FoAStackResources<'res, If::SharedResourcesType<'res>, ()>,
    wifi: WIFI,
    radio_clock: RADIO_CLK,
    adc2: ADC2,
    if_init_info: If::InitInfo,
) -> (If::ControlType<'res>, SingleInterfaceRunner<'res, If>) {
    resources.shared_lmac_state = Some(SharedLMacState::new(WiFi::new(
        wifi,
        radio_clock,
        adc2,
        &mut resources.dma_resources,
    )));
    resources.tx_buffer_manager = Some(TxBufferManager::new(&mut resources.tx_resources));
    let tx_buffer_manager = resources.tx_buffer_manager.as_mut().unwrap();
    let (receive_endpoint, transmit_endpoint, if_zero_lmac_control, _if_one_lmac_control) =
        resources
            .shared_lmac_state
            .as_mut()
            .unwrap()
            .split(tx_buffer_manager.dyn_tx_buffer_manager());

    let (if_control, if_runner, if_input) = If::new(
        &mut resources.if_zero_resources,
        if_init_info,
        transmit_endpoint,
        if_zero_lmac_control,
        Efuse::read_base_mac_address(),
    )
    .await;
    debug!("Initialized Wi-Fi stack with {} interface.", If::NAME);
    (
        if_control,
        SingleInterfaceRunner {
            receive_endpoint,
            if_runner,
            if_input,
        },
    )
}
