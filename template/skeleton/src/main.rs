#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use __HAL_MOD__::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        __CLOCK_SETUP__
        .freeze(&mut flash.acr);

    let mut gpio = dp.__GPIO_PORT__.split();
    let mut led = gpio.__LED_PIN__.__LED_MODE__;
    let delay_cycles = clocks.sysclk().raw() / __BLINK_DIVISOR__;

    loop {
        __LED_ON__
        cortex_m::asm::delay(delay_cycles);
        __LED_OFF__
        cortex_m::asm::delay(delay_cycles);
    }
}
