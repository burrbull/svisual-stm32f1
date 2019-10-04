#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]

use panic_halt as _;

use stm32f1xx_hal::{
    prelude::*,
    pac,
    delay::Delay,
    serial::{Config, Serial},
};
use cortex_m_rt::entry;

use heapless::consts::{U1, U20};
use svisual_stm32f1::prelude::*;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let p = pac::Peripherals::take().unwrap();

    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();

    // Try a different clock configuration
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = p.AFIO.constrain(&mut rcc.apb2);
    let _channels = p.DMA1.split(&mut rcc.ahb);

    let mut gpioa = p.GPIOA.split(&mut rcc.apb2);

    let mut delay = Delay::new(cp.SYST, clocks);

    // USART1
    let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pa10 = gpioa.pa10;
    let serial = Serial::usart1(
        p.USART1,
        (pa9, pa10),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    );
    let mut tx = serial.split().0; //.with_dma(channels.4);

    let mut sv = svisual::SV::<U1, U20>::new();

    loop {
        for i in 0..30 {
            sv.add_float_value(b"temp", 15. + (i as f32)).ok();
            sv.next(|s| {
                tx.send_package(b"TempMod", s);
            });
            delay.delay_ms(100u16);
        }
    }
}
