#![no_std]

use svisual::{SV, NAME_SZ};
pub mod prelude;

use byteorder::{LE,ByteOrder};
use stm32f1xx_hal as hal;
use crate::hal::{
    device::{USART1,USART2,USART3},
    serial::{TxDma1, TxDma2, TxDma3},
    dma::Transmit,
};

fn copy_slice(dst: &mut [u8], src: &[u8]) {
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s;
    }
}


pub trait SendPackageDma<N, P> : Transmit
where
    N: heapless::ArrayLength<(&'static [u8], svisual::ValueRec<P>)>,
    P: generic_array::ArrayLength<i32> + typenum::marker_traits::Unsigned + core::ops::Mul<U4>,
    <P as core::ops::Mul<U4>>::Output : generic_array::ArrayLength<u8>
{
    fn send_package_dma(
        &mut self,
        module: &'static [u8],
        values: &SV<N, P>);
}

type GA<P> = GenericArray<u8, Prod<P, U4>>;
use generic_array::typenum::{Prod, consts::U4};
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
        static mut NDATA : [u8; NAME_SZ+4] = [0u8; NAME_SZ+4];
        let mut val_data : GA<P> = GenericArray::default();// should be static
        unsafe {
            // Full package size
            LE::write_i32(&mut NDATA[0..4], (NAME_SZ + vl_size * values.map.len()) as i32);
            // Identifier (name) of the module
            copy_slice(&mut NDATA[4..], module);
        }
        unsafe { self.write_and_wait(&NDATA) };

        for (k, v) in values.map.iter() {
            // Data for single variable
            unsafe {
                NDATA = [0u8; NAME_SZ+4];
                copy_slice(&mut NDATA[0..NAME_SZ], k);
                LE::write_i32(&mut NDATA[NAME_SZ..], v.vtype as i32);
            }
            unsafe { self.write_and_wait(&NDATA) };
            
            LE::write_i32_into(&v.vals, val_data.as_mut_slice());
            
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


use stable_deref_trait::StableDeref;
use as_slice::AsSlice;

use core::sync::atomic::{self, Ordering};
extern crate cast;


pub trait WriteDmaWait<B>: hal::dma::Transmit
where B: StableDeref + AsSlice<Element = u8>/* + 'static*/ {
    fn write_and_wait(&mut self, buffer: B) -> B;
}

macro_rules! write_dma_wait {
    ($(
        $USARTX:ident: (
            $txdma:ident,
        ),
    )+) => {
        $(
            impl<B> WriteDmaWait<B> for $txdma
            where
                B: StableDeref + AsSlice<Element = u8>/* + 'static*/
            {
                fn write_and_wait(&mut self, buffer: B) -> B {
                    self.channel.set_peripheral_address(unsafe{ &(*$USARTX::ptr()).dr as *const _ as u32 }, false);

                    self.channel.set_memory_address(buffer.as_slice().as_ptr() as u32, true);
                    self.channel.set_transfer_length(buffer.as_slice().len());

                    atomic::compiler_fence(Ordering::Release);

                    self.channel.ch().cr.modify(|_, w| { w
                        .mem2mem() .clear_bit()
                        .pl()      .medium()
                        .msize()   .bit8()
                        .psize()   .bit8()
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

                    buffer
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

