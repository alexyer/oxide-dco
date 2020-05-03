#![no_main]
#![no_std]

// TODO(alexyer): Update to conditionally compile to halt for release.
use panic_semihosting as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    adc, gpio,
    gpio::{gpioa, gpiob, PushPull},
    pac::{interrupt, Interrupt, Peripherals, ADC1, TIM3},
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
};

use cortex_m_rt::entry;

use core::cell::RefCell;
use cortex_m::asm::wfi;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::Peripherals as c_m_Peripherals;
use eurorack_oxide_utils::voct::MvOct;
use stm32f1xx_hal::gpio::Output;

type AdcCh = gpiob::PB0<gpio::Analog>;
type OUTPIN = gpioa::PA0<Output<PushPull>>;

static G_ADC1: Mutex<RefCell<Option<adc::Adc<ADC1>>>> = Mutex::new(RefCell::new(None));
static G_CH0: Mutex<RefCell<Option<AdcCh>>> = Mutex::new(RefCell::new(None));
static G_TIM: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_OUT: Mutex<RefCell<Option<OUTPIN>>> = Mutex::new(RefCell::new(None));

macro_rules! get_peripheral_ref {
    ($lref:ident, $gref:ident) => {
        $lref.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| $gref.borrow(cs).replace(None).unwrap())
        })
    };
}

#[interrupt]
fn TIM3() {
    static mut ADC: Option<adc::Adc<ADC1>> = None;
    static mut CH0: Option<AdcCh> = None;
    static mut TIM: Option<CountDownTimer<TIM3>> = None;
    static mut OUT: Option<OUTPIN> = None;

    static mut COUNTER: usize = 0;
    static mut PERIOD: usize = 0;

    let adc1 = get_peripheral_ref!(ADC, G_ADC1);
    let ch0 = get_peripheral_ref!(CH0, G_CH0);
    let tim = get_peripheral_ref!(TIM, G_TIM);
    let out = get_peripheral_ref!(OUT, G_OUT);

    *COUNTER += 1;
    if *COUNTER == 1 {
        out.toggle().ok();
        let measure: u16 = adc1.read(ch0).unwrap();
        *PERIOD = MvOct(measure as f32 * 1.209 + 1000.0).us() as usize;
    } else if *COUNTER == *PERIOD / 2 {
        out.toggle().ok();
    } else if *COUNTER == *PERIOD {
        *COUNTER = 0;
    }

    tim.wait().ok();
}

#[entry]
fn main() -> ! {
    init_peripherals();

    loop {
        wfi();
    }
}

fn init_peripherals() {
    if let (Some(dp), Some(cp)) = (Peripherals::take(), c_m_Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();

            // Init clocks
            let clocks = rcc
                .cfgr
                .adcclk(100.khz())
                .sysclk(72.mhz())
                .pclk1(30.mhz())
                .freeze(&mut flash.acr);

            // Init ADC
            let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
            adc1.set_sample_time(adc::SampleTime::T_239);
            let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
            let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

            *G_ADC1.borrow(cs).borrow_mut() = Some(adc1);
            *G_CH0.borrow(cs).borrow_mut() = Some(ch0);

            // Init timer
            let mut tim =
                Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(8.mhz());
            tim.listen(Event::Update);

            *G_TIM.borrow(cs).borrow_mut() = Some(tim);

            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::TIM3, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::TIM3);

            // Configure out pin
            let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
            let out = gpioa.pa0.into_push_pull_output(&mut gpioa.crl);

            *G_OUT.borrow(cs).borrow_mut() = Some(out);
        });
    } else if cfg!(debug_assertions) {
        panic!("Failed to initialize timer: can't take Peripherals");
    }
}
