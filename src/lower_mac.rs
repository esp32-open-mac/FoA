use core::{
    cell::UnsafeCell,
    future::Future,
    sync::atomic::{AtomicU16, Ordering},
};

use embassy_futures::{
    select::{select, Either},
    yield_now,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    semaphore::{FairSemaphore, Semaphore, SemaphoreReleaser},
    signal::Signal,
};
use esp32_wifi_hal_rs::{BorrowedBuffer, RxFilterBank, RxFilterInterface, WiFi, WiFiRate};
use esp_hal::{
    efuse::Efuse,
    peripherals::{ADC2, RADIO_CLK, WIFI},
};
use ieee80211::{
    mac_parser::MACAddress,
    scroll::{self, ctx::TryIntoCtx, Pwrite},
};
use log::debug;

use crate::{StaError, StaResult};

pub struct LowerMACTransaction<'a> {
    lower_mac: &'a LowerMAC,
    _permit: SemaphoreReleaser<'a, FairSemaphore<CriticalSectionRawMutex, 2>>,
    previous_channel: u8,
    rollback: bool,
}
impl<'a> LowerMACTransaction<'a> {
    pub fn new(
        lower_mac: &'a LowerMAC,
        _permit: SemaphoreReleaser<'a, FairSemaphore<CriticalSectionRawMutex, 2>>,
    ) -> Self {
        Self {
            lower_mac,
            _permit,
            previous_channel: unsafe { lower_mac.wifi.get().as_ref().unwrap().get_channel() },
            rollback: true,
        }
    }
    pub fn disable_rollback(&mut self) {
        self.rollback = false;
    }
    pub fn get_wifi(&self) -> &WiFi {
        unsafe { self.lower_mac.wifi.get().as_ref().unwrap() }
    }
    pub fn get_wifi_mut(&mut self) -> &mut WiFi {
        unsafe { self.lower_mac.wifi.get().as_mut().unwrap() }
    }
    pub async fn receive(&self) -> BorrowedBuffer {
        self.get_wifi().receive().await
    }
    pub async fn transmit(
        &self,
        frame: impl TryIntoCtx<bool, Error = scroll::Error>,
        buf: &mut [u8],
        rate: WiFiRate,
    ) -> StaResult<()> {
        let written = buf
            .pwrite_with(frame, 0, true)
            .map_err(StaError::SerializationError)?;
        self.get_wifi().transmit(&buf[..written], rate).await;
        Ok(())
    }
    pub fn set_channel(&self, channel: u8) {
        unsafe {
            self.lower_mac
                .wifi
                .get()
                .as_mut()
                .unwrap()
                .set_channel(channel)
                .unwrap()
        }
    }
}
impl Drop for LowerMACTransaction<'_> {
    fn drop(&mut self) {
        self.lower_mac.transaction_pending.signal(false);
        if self.rollback {
            let previous_channel = self.previous_channel;
            let _ = self.get_wifi_mut().set_channel(previous_channel);
            self.get_wifi_mut()
                .set_scanning_mode(RxFilterInterface::Zero, false);
        }
    }
}
/// The lower MAC is concerned with sharing access to the WiFi peripheral,
/// between the background task and the user facing control.
///
/// This works through a system of transactions.
/// In normal operation, the background task will use the `nt_(receive|transmit)` functions.
/// These will wait until no transaction is in progress and grant access to the peripheral.
/// When exclusive access to the peripheral is needed, a transaction can be started with
/// [Self::transaction_begin].
pub struct LowerMAC {
    wifi: UnsafeCell<WiFi>,
    transaction_pending: Signal<CriticalSectionRawMutex, bool>,
    access_semaphore: FairSemaphore<CriticalSectionRawMutex, 2>,
    sequence_number: AtomicU16,
    mac_address: MACAddress,
}
impl LowerMAC {
    pub fn new(
        wifi: WIFI,
        radio_clock: RADIO_CLK,
        adc2: ADC2,
        mac_address: Option<MACAddress>,
    ) -> Self {
        let mut wifi = WiFi::new(wifi, radio_clock, adc2);
        let mac_address = mac_address.unwrap_or(MACAddress::new(Efuse::get_mac_address()));
        wifi.set_filter(
            RxFilterBank::ReceiverAddress,
            RxFilterInterface::Zero,
            *mac_address,
            [0xff; 6],
        );
        wifi.set_filter_status(RxFilterBank::ReceiverAddress, RxFilterInterface::Zero, true);
        debug!("Lower MAC initialised with MAC address: {mac_address}.");
        Self {
            wifi: UnsafeCell::new(wifi),
            transaction_pending: Signal::new(),
            access_semaphore: FairSemaphore::new(1),
            sequence_number: AtomicU16::new(0),
            mac_address,
        }
    }
    pub fn get_and_increase_sequence_number(&self) -> u16 {
        self.sequence_number.fetch_add(1, Ordering::Relaxed)
    }
    pub fn get_mac_address(&self) -> MACAddress {
        self.mac_address
    }
    /// Wait for a frame to arrive outside of a transaction.
    pub async fn receive(&self) -> BorrowedBuffer<'_, '_> {
        loop {
            let _permit = self.access_semaphore.acquire(1).await.unwrap();
            match select(
                unsafe { self.wifi.get().as_mut().unwrap() }.receive(),
                self.transaction_pending.wait(),
            )
            .await
            {
                Either::First(borrowed_buffer) => break borrowed_buffer,
                Either::Second(_) => {}
            }
        }
    }
    /// Transmit a frame outside of a transaction.
    pub async fn transmit(
        &self,
        frame: impl TryIntoCtx<bool, Error = scroll::Error>,
        buf: &mut [u8],
        rate: WiFiRate,
    ) -> StaResult<()> {
        let _permit = self.access_semaphore.acquire(1).await.unwrap();
        let written = buf
            .pwrite_with(frame, 0, true)
            .map_err(StaError::SerializationError)?;
        unsafe { self.wifi.get().as_mut().unwrap() }
            .transmit(&buf[..written], rate)
            .await;
        Ok(())
    }
    /// Begin a transaction.
    pub async fn transaction_begin(&self) -> LowerMACTransaction<'_> {
        self.transaction_pending.signal(true);
        let permit = self.access_semaphore.acquire(1).await.unwrap();
        LowerMACTransaction::new(self, permit)
    }
}
