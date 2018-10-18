#![no_std]
#![allow(patterns_in_fns_without_body)]

use svisual::{NAME_SZ, PACKET_SZ, VL_SZ};
pub mod prelude;

use heapless::{
    consts::U1
};
use byteorder::{LE,ByteOrder};
use stm32f103xx_hal as hal;
use crate::hal::stm32f103xx as device;
use crate::hal::{serial::Tx,dma::dma1};
use crate::device::{USART1,USART2,USART3};

fn copy_slice(dst: &mut [u8], src: &[u8]) {
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s;
    }
}


pub trait SendPackageDma {
    type Chan;
    fn send_package_dma(
        self,
        module: &'static [u8],
        mut c: Self::Chan,
        values: &heapless::FnvIndexMap<&[u8], svisual::ValueRec, U1>)
    -> (Self::Chan, Self);
}


macro_rules! impl_send_package_dma {
    ($(
        $USARTX:ident: (
            tx: $tx_chan:path
        ),
    )+) => {
        $(
impl SendPackageDma for Tx<$USARTX> {
    type Chan = $tx_chan;
    fn send_package_dma(
        self,
        module: &'static [u8],
        c: Self::Chan,
        values: &heapless::FnvIndexMap<&[u8], svisual::ValueRec, U1>)
    -> (Self::Chan, Self) {
        //if values.is_empty() {
        //    return Err(Error::EmptyPackage);
        //}

        let (_, c, tx) = self.write_all(c, b"=begin=").wait();

        static mut NDATA : [u8; NAME_SZ+4] = [0u8; NAME_SZ+4];
        unsafe {
            // Общий размер пакета
            LE::write_i32(&mut NDATA[0..4], (NAME_SZ + VL_SZ * values.len()) as i32);
            // Идентификатор (название) модуля
            copy_slice(&mut NDATA[4..], module);
        }
        let (_, c, tx) = unsafe { tx.write_all(c, &NDATA).wait() };

        let mut tx = Some(tx);
        let mut c = Some(c);
        for (k, v) in values {
            let c_back = c.take().unwrap();
            let tx_back = tx.take().unwrap();
            unsafe {
                NDATA = [0u8; NAME_SZ+4];
                copy_slice(&mut NDATA[0..NAME_SZ], k);
                LE::write_i32(&mut NDATA[NAME_SZ..], v.vtype as i32);
            }
            let (_, c_back, tx_back) = unsafe { tx_back.write_all(c_back, &NDATA).wait() };
            
            unsafe { LE::write_i32_into(&v.vals, &mut MYDATA.0); }
            
            // Значения для одной переменной
            static mut MYDATA : MyData = MyData([0u8; PACKET_SZ*4]);
            let (_, c_back, tx_back) = unsafe { tx_back.write_all(c_back, &MYDATA).wait() };
            tx = Some(tx_back);
            c = Some(c_back);
        }
        let tx = tx.unwrap();
        let c = c.unwrap();
        let (_, c, tx) = tx.write_all(c, b"=end=").wait();
        (c, tx)
    }
}
        )+
    }
}

struct MyData([u8; PACKET_SZ*4]);
impl core::convert::AsRef<[u8]> for MyData {
    #[inline]
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl_send_package_dma! {
    USART1: (
        tx: dma1::C4
    ),
    USART2: (
        tx: dma1::C7
    ),
    USART3: (
        tx: dma1::C2
    ),
}
