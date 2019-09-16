#![no_std]

use core::mem::MaybeUninit;
use svisual::{NAME_SZ, SV};
pub mod prelude;

use crate::hal::{
    device::{USART1, USART2, USART3},
    dma::Transmit,
    serial::{TxDma1, TxDma2, TxDma3},
};
use byteorder::{ByteOrder, LE};
use stm32f1xx_hal as hal;

pub trait SendPackageDma<N, P>: Transmit
where
    N: heapless::ArrayLength<(&'static [u8], svisual::ValueRec<P>)>,
    P: generic_array::ArrayLength<i32> + typenum::marker_traits::Unsigned + core::ops::Mul<U4>,
    <P as core::ops::Mul<U4>>::Output: generic_array::ArrayLength<u8>,
{
    fn send_package_dma(&mut self, module: &'static [u8], values: &SV<N, P>);
}

type GA<P> = GenericArray<u8, Prod<P, U4>>;
use generic_array::typenum::{consts::U4, Prod};
use generic_array::GenericArray;

macro_rules! impl_send_package_dma {
    ($(
        $USARTX:ident: (
            $txdma:ident,
        ),
    )+) => {
        $(
impl<N, P> SendPackageDma<N, P> for $txdma
where
    N: heapless::ArrayLength<(&'static [u8], svisual::ValueRec<P>)>,
    P: generic_array::ArrayLength<i32> + typenum::marker_traits::Unsigned + core::ops::Mul<U4>,
    <P as core::ops::Mul<U4>>::Output : generic_array::ArrayLength<u8>
{
    fn send_package_dma(
        &mut self,
        module: &'static [u8],
        values: &SV<N, P>)
    {
        //if values.map.is_empty() {
        //    return Err(Error::EmptyPackage);
        //}

        self.write_and_wait(b"=begin=");

        let packet_size = P::to_usize();
        let vl_size : usize = NAME_SZ+4+packet_size*4;
        static mut NDATA : MaybeUninit<[u8; NAME_SZ+4]> = MaybeUninit::uninit();
        let mut val_data = unsafe { MaybeUninit::<GA<P>>::uninit().assume_init() };// should be static
        unsafe {
            // Full package size
            let x_arr = &mut *NDATA.as_mut_ptr();
            x_arr[0..4].copy_from_slice(&((NAME_SZ + vl_size * values.map.len()) as u32).to_le_bytes());
            // Identifier (name) of the module
            let len = module.len();
            x_arr[4..len+4].copy_from_slice(module);
            if NAME_SZ > len {
                x_arr[len+4] = 0;
            }

            self.write_and_wait(&(*NDATA.as_ptr()));
        }

        for (k, v) in values.map.iter() {
            unsafe {
                // Data for single variable
                let x_arr = &mut *NDATA.as_mut_ptr();
                x_arr[0..k.len()].copy_from_slice(k);
                if NAME_SZ > k.len() {
                    x_arr[k.len()] = 0;
                }
                x_arr[NAME_SZ..].copy_from_slice(&(v.vtype as i32).to_le_bytes());

                self.write_and_wait(&(*NDATA.as_ptr()));
            }

            LE::write_i32_into(&v.vals, &mut val_data.as_mut_slice());
            self.write_and_wait(&val_data);
        }
        self.write_and_wait(b"=end=");
    }
}
        )+
    }
}

impl_send_package_dma! {
    USART1: (
        TxDma1,
    ),
    USART2: (
        TxDma2,
    ),
    USART3: (
        TxDma3,
    ),
}

use as_slice::AsSlice;
use stm32f1xx_hal::dma::TransferPayload;

use core::sync::atomic::{self, Ordering};
extern crate cast;

pub trait BlockingWriteDma<B>: hal::dma::Transmit
where
    B: AsSlice<Element = u8>,
{
    fn write_and_wait(&mut self, buffer: &B);
}

macro_rules! write_dma_wait {
    ($(
        $USARTX:ident: (
            $txdma:ident,
        ),
    )+) => {
        $(
            impl<B> BlockingWriteDma<B> for $txdma
            where
                B: AsSlice<Element = u8>
            {
                fn write_and_wait(&mut self, buffer: &B) {
                    self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);

                    self.channel.set_memory_address(buffer.as_slice().as_ptr() as u32, true);
                    self.channel.set_transfer_length(buffer.as_slice().len());

                    atomic::compiler_fence(Ordering::Release);

                    self.channel.ch().cr.modify(|_, w| { w
                        .mem2mem() .clear_bit()
                        .pl()      .medium()
                        .msize()   .bits8()
                        .psize()   .bits8()
                        .circ()    .clear_bit()
                        .dir()     .set_bit()
                    });
                    self.start();

                    while self.channel.in_progress() {}

                    atomic::compiler_fence(Ordering::Acquire);

                    self.stop();

                    // we need a read here to make the Acquire fence effective
                    // we do *not* need this if `dma.stop` does a RMW operation
                    unsafe { core::ptr::read_volatile(&0); }

                    // we need a fence here for the same reason we need one in `Transfer.wait`
                    atomic::compiler_fence(Ordering::Acquire);
                }
            }
        )+
    }
}

write_dma_wait! {
    USART1: (
        TxDma1,
    ),
    USART2: (
        TxDma2,
    ),
    USART3: (
        TxDma3,
    ),
}
