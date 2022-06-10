use mutex_trait;
use rp_pico::hal::sio::{SpinlockValid,Spinlock};
use core::cell::RefCell;

pub struct Mutex<T,const N: usize>
where Spinlock<N>: SpinlockValid
{
    data: T,
}

impl<T,const N: usize> Mutex<T,N> 
where Spinlock<N>: SpinlockValid
{
    pub const fn new(data: T) -> Self {
        Self { data}
    }

    fn access(&self) -> &T {
        &self.data
    }
}

    
impl<'a, T, const N: usize> mutex_trait::Mutex for &'a Mutex<RefCell<T>,N> 
where Spinlock<N>: SpinlockValid
{
    type Data = T;

    fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
        let _lock= Spinlock::<N>::claim();
        f(&mut *self.access().borrow_mut())
    }
}

unsafe impl< T, const N: usize> Sync for Mutex<T,N>
where Spinlock<N>: SpinlockValid {}