#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate panic_halt;

use crate::hal::delay::Delay;
use crate::hal::prelude::*;
use crate::hal::serial::Serial;
use crate::hal::stm32f103xx as device;
use cortex_m_rt::entry;
use stm32f103xx_hal as hal;

use heapless::consts::U1;
use svisual_stm32f1::prelude::*;

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
    let mut tx = serial.split().0;
    let mut c = channels.4;

    let mut sv = svisual::SV::<U1>::new();

    loop {
        for i in 0..30 {
            sv.add_float_value(b"temp", 15. + (i as f32)).ok();
            sv.next(|s| {
                tx_back.send_package_dma(b"TempMod", &mut c, s);
            });
            delay.delay_ms(100u16);
        }
    }
}
