#![no_main]
#![no_std]

use panic_semihosting as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    adc,
    pac::Peripherals,
    prelude::*,
    timer::{Tim2NoRemap, Timer},
};

use cortex_m::asm::wfi;
use cortex_m_rt::entry;

use eurorack_oxide_utils::voct::MvOct;
use stm32f1xx_hal::pwm::Channel;

// macro_rules! get_peripheral_ref {
//     ($lref:ident, $gref:ident) => {
//         $lref.get_or_insert_with(|| {
//             cortex_m::interrupt::free(|cs| $gref.borrow(cs).replace(None).unwrap())
//         })
//     };
// }

#[entry]
fn main() -> ! {
    init_peripherals();

    loop {
        wfi();
    }
}

fn init_peripherals() {
    if let Some(dp) = Peripherals::take() {
        // cortex_m::interrupt::free(move |cs| {
        let mut rcc = dp.RCC.constrain();
        let mut flash = dp.FLASH.constrain();
        let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

        // Init clocks
        let clocks = rcc
            .cfgr
            .adcclk(2.mhz())
            .sysclk(8.mhz())
            .pclk1(8.mhz())
            .freeze(&mut flash.acr);

        // Init timer
        let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
        let pins = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
        );

        let mut pwm = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1).pwm::<Tim2NoRemap, _, _, _>(
            pins,
            &mut afio.mapr,
            1.hz(),
        );
        pwm.set_duty(Channel::C1, pwm.get_max_duty() / 2);
        pwm.enable(Channel::C1);

        // Init ADC
        let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
        let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
        let mut ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
        //
        // *G_ADC1.borrow(cs).borrow_mut() = Some(adc1);
        // *G_CH0.borrow(cs).borrow_mut() = Some(ch0);
        loop {
            let measure: u16 = adc1.read(&mut ch0).unwrap();
            let freq = (MvOct(measure as f32 * 0.806).us()) as u32;
            pwm.set_period(freq.us());
            pwm.set_duty(Channel::C1, pwm.get_max_duty() / 2);
        }
    } else if cfg!(debug_assertions) {
        panic!("Failed to initialize timer: can't take Peripherals");
    }
}
