#![no_std]

use svisual::{SV, NAME_SZ, PACKET_SZ, VL_SZ};
pub mod prelude;

//use heapless::consts::U2;

use byteorder::{LE,ByteOrder};
use stm32f103xx_hal as hal;
use crate::hal::{
    stm32f103xx as device,
    serial::{Tx, WriteDma},
    dma::DmaChannel
};
use crate::device::{USART1,USART2,USART3};

fn copy_slice(dst: &mut [u8], src: &[u8]) {
    for (d, s) in dst.iter_mut().zip(src.iter()) {
        *d = *s;
    }
}


pub trait SendPackageDma<N> : DmaChannel
    where N: heapless::ArrayLength<(&'static [u8], svisual::ValueRec)> {
    fn send_package_dma(
        self,
        module: &'static [u8],
        c: Self::Dma,
        values: &SV<N>)
    -> (Self::Dma, Self);
}

macro_rules! impl_send_package_dma {
    ($USARTX:ident) => {
impl<N> SendPackageDma<N> for Tx<$USARTX>
    where N: heapless::ArrayLength<(&'static [u8], svisual::ValueRec)> {
    fn send_package_dma(
        self,
        module: &'static [u8],
        c: Self::Dma,
        values: &SV<N>)
    -> (Self::Dma, Self) {
        //if values.map.is_empty() {
        //    return Err(Error::EmptyPackage);
        //}

        let (_, c, tx) = self.write_all(c, b"=begin=").wait();

        static mut NDATA : [u8; NAME_SZ+4] = [0u8; NAME_SZ+4];
        unsafe {
            // Full package size
            LE::write_i32(&mut NDATA[0..4], (NAME_SZ + VL_SZ * values.map.len()) as i32);
            // Identifier (name) of the module
            copy_slice(&mut NDATA[4..], module);
        }
        let (_, c, tx) = unsafe { tx.write_all(c, &NDATA).wait() };

        let mut tx = Some(tx);
        let mut c = Some(c);
        for (k, v) in values.map.iter() {
            let c_back = c.take().unwrap();
            let tx_back = tx.take().unwrap();
            // Data for single variable
            unsafe {
                NDATA = [0u8; NAME_SZ+4];
                copy_slice(&mut NDATA[0..NAME_SZ], k);
                LE::write_i32(&mut NDATA[NAME_SZ..], v.vtype as i32);
            }
            let (_, c_back, tx_back) = unsafe { tx_back.write_all(c_back, &NDATA).wait() };
            
            unsafe { LE::write_i32_into(&v.vals, &mut MYDATA.0); }
            
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
    }
}

struct MyData([u8; PACKET_SZ*4]);
impl core::convert::AsRef<[u8]> for MyData {
    #[inline]
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl_send_package_dma!(USART1);
impl_send_package_dma!(USART2);
impl_send_package_dma!(USART3);
