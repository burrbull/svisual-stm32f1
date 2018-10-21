#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use stm32f103xx_hal as hal;
use crate::hal::stm32f103xx as device;
use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::serial::Serial;
use cortex_m_rt::{entry};

use svisual_stm32f1::prelude::*;
use heapless::consts::{U1};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = device::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Try a different clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let channels = dp.DMA1.split(&mut rcc.ahb);

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    
    let mut delay = Delay::new(cp.SYST, clocks);
    
    // USART1
    let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pa10 = gpioa.pa10;
    let serial = Serial::usart1(
        dp.USART1,
        (pa9, pa10),
        &mut afio.mapr,
        9_600.bps(),
        clocks,
        &mut rcc.apb2,
    );
    let tx = serial.split().0;
    let c = channels.4;
    
    let mut sv = svisual::SV::<U1>::new();
    
    let mut tx = Some(tx);
    let mut c = Some(c);
    
    loop {
        for i in 0..30 {
            sv.add_float_value(b"temp", 15.+(i as f32)).ok();
            sv.next(|s| {
                let c_back = c.take().unwrap();
                let tx_back = tx.take().unwrap();
                let (c_back, tx_back) = tx_back.send_package_dma(b"TempMod", c_back, s);
                tx = Some(tx_back);
                c = Some(c_back);
            });
            delay.delay_ms(100u16);
        }
    }
}
