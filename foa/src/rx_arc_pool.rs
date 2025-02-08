use core::mem::MaybeUninit;

use esp_wifi_hal::BorrowedBuffer;
use portable_atomic::{AtomicUsize, Ordering};

struct RxArcBufferInner<'res> {
    buffer: BorrowedBuffer<'res>,
    refs: AtomicUsize,
}
impl RxArcBufferInner<'_> {
    fn increment_refs(&self) -> usize {
        self.refs.fetch_add(1, Ordering::Relaxed)
    }
    fn decrement_refs(&self) -> usize {
        self.refs.fetch_sub(1, Ordering::Relaxed)
    }
}

struct RxArcPoolState<const RX_BUFFER_COUNT: usize> {
    slab: [(MaybeUninit<RxArcBufferInner<'static>>, bool); RX_BUFFER_COUNT],
}
impl<const RX_BUFFER_COUNT: usize> RxArcPoolState<RX_BUFFER_COUNT> {
    fn try_alloc<'a, 'res>(
        &'a mut self,
        buffer: BorrowedBuffer<'res>,
    ) -> Option<&'a mut RxArcBufferInner<'res>> {
        self.slab
            .iter_mut()
            .filter(|inner| inner.1)
            .next()
            .map(|inner| {
                unsafe {
                    core::mem::transmute::<_, &'a mut MaybeUninit<RxArcBufferInner<'res>>>(
                        &mut inner.0,
                    )
                }
                .write(RxArcBufferInner {
                    buffer,
                    refs: AtomicUsize::default(),
                })
            })
    }
}
pub struct RxArcPool<const RX_BUFFER_COUNT: usize> {}
