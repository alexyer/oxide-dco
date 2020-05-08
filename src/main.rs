#![no_main]
#![no_std]

use core::cell::RefCell;
// TODO(alexyer): Update to conditionally compile to halt for release.
use panic_semihosting as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    adc, gpio,
    gpio::{gpiob, Output, PushPull},
    pac::{interrupt, Interrupt, Peripherals, ADC1, GPIOA, TIM3},
    prelude::*,
    rcc::Enable,
    timer::{CountDownTimer, Event, Timer},
};

use cortex_m::{asm::wfi, interrupt::Mutex};

use cortex_m::peripheral::Peripherals as c_m_Peripherals;
use cortex_m_rt::entry;

use eurorack_oxide_utils::voct::MvOct;
use eurorack_oxide_utils::voct::Voltage;

type AdcCh = gpiob::PB0<gpio::Analog>;
type OUTPIN = gpiob::PB1<Output<PushPull>>;

static G_ADC1: Mutex<RefCell<Option<adc::Adc<ADC1>>>> = Mutex::new(RefCell::new(None));
static G_CH0: Mutex<RefCell<Option<AdcCh>>> = Mutex::new(RefCell::new(None));
static G_DAC: Mutex<RefCell<Option<GPIOA>>> = Mutex::new(RefCell::new(None));
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
    static mut DAC: Option<GPIOA> = None;
    static mut TIM: Option<CountDownTimer<TIM3>> = None;
    static mut OUT: Option<OUTPIN> = None;

    static mut COUNTER: usize = 0;
    static mut PERIOD: usize = 0;

    let adc1 = get_peripheral_ref!(ADC, G_ADC1);
    let ch0 = get_peripheral_ref!(CH0, G_CH0);
    let dac = get_peripheral_ref!(DAC, G_DAC);
    let out = get_peripheral_ref!(OUT, G_OUT);
    let tim = get_peripheral_ref!(TIM, G_TIM);

    *COUNTER += 1;
    if *COUNTER == 1 {
        out.toggle().ok();
        let oct = 0000.0;
        let measure: u16 = adc1.read(ch0).unwrap();
        let mv = MvOct(measure as f32 * 1.3333333 + oct + 1000.0);
        *PERIOD = mv.us() as usize;
        // dac.odr.write(|w| unsafe { w.bits((measure / 32) as u32) });
        dac.odr
            .write(|w| unsafe { w.bits((mv.hz() / 32.0) as u32) });
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
                .sysclk(70.mhz())
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
            let mut tim = Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(8.mhz());
            tim.listen(Event::Update);

            *G_TIM.borrow(cs).borrow_mut() = Some(tim);

            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::TIM3, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::TIM3);

            // Configure out pin
            let out = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);
            *G_OUT.borrow(cs).borrow_mut() = Some(out);

            // Init DAC port
            let dac = dp.GPIOA;
            GPIOA::enable(&mut rcc.apb2);

            dac.crl.write(|w| unsafe { w.bits(0x33333333) });
            *G_DAC.borrow(cs).borrow_mut() = Some(dac);
        });
    } else if cfg!(debug_assertions) {
        panic!("Initialization failed: can't take Peripherals");
    }
}
