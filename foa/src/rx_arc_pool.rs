use core::{cell::RefCell, ops::Deref, ptr::NonNull};

use embassy_sync::blocking_mutex::NoopMutex;
use esp_wifi_hal::BorrowedBuffer;
use portable_atomic::{AtomicUsize, Ordering};

struct RxArcBufferInner<'res> {
    buffer: BorrowedBuffer<'res>,
    refs: AtomicUsize,
    index: usize,
}
impl RxArcBufferInner<'_> {
    fn increment_refs(&self) -> usize {
        self.refs.fetch_add(1, Ordering::Relaxed)
    }
    fn decrement_refs(&self) -> usize {
        self.refs.fetch_sub(1, Ordering::Relaxed)
    }
}

/// An atomically reference counted received buffer, allocated from a static pool.
pub struct RxArcBuffer<'res> {
    inner: NonNull<RxArcBufferInner<'res>>,
    arc_pool: &'res RxArcPool,
}
impl<'res> RxArcBuffer<'res> {
    fn inner_ref(&self) -> &'res RxArcBufferInner<'res> {
        unsafe { self.inner.as_ref() }
    }
}
impl Clone for RxArcBuffer<'_> {
    fn clone(&self) -> Self {
        self.inner_ref().increment_refs();
        Self {
            inner: self.inner,
            arc_pool: self.arc_pool,
        }
    }
}
impl<'res> Deref for RxArcBuffer<'res> {
    type Target = BorrowedBuffer<'res>;
    fn deref(&self) -> &Self::Target {
        &self.inner_ref().buffer
    }
}
impl Drop for RxArcBuffer<'_> {
    fn drop(&mut self) {
        if self.inner_ref().decrement_refs() == 0 {
            unsafe {
                self.arc_pool.free(self.inner);
            }
        }
    }
}

pub(crate) struct RxArcPool {
    state: NoopMutex<RefCell<[Option<RxArcBufferInner<'static>>; super::RX_BUFFER_COUNT]>>,
}
impl RxArcPool {
    pub const fn new() -> Self {
        Self {
            state: NoopMutex::new(RefCell::new([const { None }; super::RX_BUFFER_COUNT])),
        }
    }
    pub(crate) fn try_alloc<'res>(
        &'res self,
        buffer: BorrowedBuffer<'res>,
    ) -> Option<RxArcBuffer<'res>> {
        // SAFETY:
        // We only hand out references to the inside of the Option, which means that the actual
        // state of the option can not be changed. Also the NoopMutex enforces, that this never
        // crosses core boundaries, which means that the transitions between in use and not in use
        // can only happen inside the lock guard, even though the inside of the Option maybe used
        // without a lock.
        //
        // If you have any soundness concerns PLEASE bring them up, since this goes quite a bit
        // over my head. (Frostie314159)
        let inner = self.state.lock(|rc| {
            let Ok(mut state) = rc.try_borrow_mut() else {
                // This should not be reachable, since FoA can only run on one core at a time, so
                // unless, we tried to lock it again in here, this can never fail. I decided to
                // just return None in this case, since I wanted to avoid unnecessary panic
                // machinery being generated.
                return None;
            };
            state
                .iter_mut()
                .enumerate()
                .find(|inner| inner.1.is_none())
                .map(|(i, inner)| {
                    NonNull::from(
                        // SAFETY:
                        // We transmute this to have the lifetime 'res, which is the same lifetime,
                        // that self has.
                        unsafe {
                            core::mem::transmute::<
                                &mut Option<RxArcBufferInner<'static>>,
                                &mut Option<RxArcBufferInner<'res>>,
                            >(inner)
                        }
                        .insert(RxArcBufferInner {
                            buffer,
                            refs: AtomicUsize::default(),
                            index: i,
                        }),
                    )
                })
        });
        inner.map(|inner| RxArcBuffer {
            inner,
            arc_pool: self,
        })
    }
    // SAFETY:
    // inner has to be a pointer to a RxArcBufferInner, that was allocated from this ARC pool.
    unsafe fn free(&self, inner: NonNull<RxArcBufferInner<'_>>) {
        let inner_ref = unsafe { inner.as_ref() };
        // Indicates, that this arc is available again.
        self.state.lock(|rc| {
            let Ok(mut state) = rc.try_borrow_mut() else {
                // See alloc for the explanation, of why this is unreachable.
                return;
            };
            state[inner_ref.index] = None;
        });
    }
}
