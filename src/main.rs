#![no_main]
#![no_std]

use core::cell::RefCell;
// TODO(alexyer): Update to conditionally compile to halt for release.
use panic_semihosting as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    adc, gpio,
    gpio::{gpiob, Output, PushPull},
    pac::{interrupt, Interrupt, Peripherals, ADC1, GPIOA, TIM2, TIM3},
    prelude::*,
    rcc::Enable,
    timer::{CountDownTimer, Event, Timer},
};

use core::sync::atomic::{AtomicU32, Ordering};

use cortex_m::peripheral::Peripherals as c_m_Peripherals;
use cortex_m::{asm::wfi, interrupt::Mutex};
use cortex_m_rt::entry;

use eurorack_oxide_utils::voct::MvOct;
use eurorack_oxide_utils::voct::Voltage;

type AdcCh = gpiob::PB0<gpio::Analog>;
type OUTPIN = gpiob::PB1<Output<PushPull>>;
decreconst AVG_BUF_SIZE: usize = 32;
const TIM3_FREQ_HZ: u32 = 200000;
const SEC_IN_US: u32 = 1000000;

static G_ADC1: Mutex<RefCell<Option<adc::Adc<ADC1>>>> = Mutex::new(RefCell::new(None));
static G_CH0: Mutex<RefCell<Option<AdcCh>>> = Mutex::new(RefCell::new(None));
static G_DAC: Mutex<RefCell<Option<GPIOA>>> = Mutex::new(RefCell::new(None));
static G_TIM2: Mutex<RefCell<Option<CountDownTimer<TIM2>>>> = Mutex::new(RefCell::new(None));
static G_TIM3: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));
static G_OUT: Mutex<RefCell<Option<OUTPIN>>> = Mutex::new(RefCell::new(None));
static PERIOD: AtomicU32 = AtomicU32::new(0);

macro_rules! get_peripheral_ref {
    ($lref:ident, $gref:ident) => {
        $lref.get_or_insert_with(|| {
            cortex_m::interrupt::free(|cs| $gref.borrow(cs).replace(None).unwrap())
        })
    };
}

const fn cirle_time() -> u32 {
    SEC_IN_US / TIM3_FREQ_HZ
}

fn us_to_period(us: u32) -> u32 {
    us / cirle_time() / 2
}

#[interrupt]
fn TIM2() {
    static mut ADC: Option<adc::Adc<ADC1>> = None;
    static mut AVG_BUF: [u16; AVG_BUF_SIZE] = [0; AVG_BUF_SIZE];
    static mut AVG_COUNTER: usize = 0;
    static mut CH0: Option<AdcCh> = None;
    static mut DAC: Option<GPIOA> = None;
    static mut TIM: Option<CountDownTimer<TIM2>> = None;

    let adc1 = get_peripheral_ref!(ADC, G_ADC1);
    let dac = get_peripheral_ref!(DAC, G_DAC);
    let ch0 = get_peripheral_ref!(CH0, G_CH0);
    let tim = get_peripheral_ref!(TIM, G_TIM2);

    AVG_BUF[*AVG_COUNTER % AVG_BUF_SIZE] = adc1.read(ch0).unwrap();
    *AVG_COUNTER += 1;

    if *AVG_COUNTER % AVG_BUF_SIZE == 0 {
        let avg = avg(AVG_BUF);
        let voltage = avg * 1200 / adc1.read_vref() as u32;
        let mv = MvOct(voltage as f32 * 1.5015 as f32);

        PERIOD.store(us_to_period(mv.us()), Ordering::Relaxed);
        dac.odr
            .write(|w| unsafe { w.bits((mv.hz() / 32.0) as u32) });
    }

    tim.wait().ok();
}

#[interrupt]
fn TIM3() {
    static mut TIM: Option<CountDownTimer<TIM3>> = None;
    static mut OUT: Option<OUTPIN> = None;

    static mut LOCAL_PERIOD: u32 = 0;
    static mut COUNTER: u32 = 0;

    let out = get_peripheral_ref!(OUT, G_OUT);
    let tim = get_peripheral_ref!(TIM, G_TIM3);

    if *COUNTER >= *LOCAL_PERIOD {
        out.toggle().ok();
        *COUNTER = 0;
        *LOCAL_PERIOD = PERIOD.load(Ordering::Relaxed);
    }
    *COUNTER += 1;

    tim.wait().ok();
}

fn avg(buf: &mut [u16; AVG_BUF_SIZE]) -> u32 {
    let mut acc: u32 = 0;
    for i in 0..buf.len() {
        acc += buf[i] as u32;
    }

    acc / AVG_BUF_SIZE as u32
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
                .adcclk(10.mhz())
                .sysclk(30.mhz())
                .pclk1(15.mhz())
                .freeze(&mut flash.acr);

            // Init ADC
            let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
            adc1.set_sample_time(adc::SampleTime::T_239);
            let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
            let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

            *G_ADC1.borrow(cs).borrow_mut() = Some(adc1);
            *G_CH0.borrow(cs).borrow_mut() = Some(ch0);

            // Init timer
            let mut tim2 = Timer::tim2(dp.TIM2, &clocks, &mut rcc.apb1)
                .start_count_down((TIM3_FREQ_HZ / 2).hz());
            tim2.listen(Event::Update);

            let mut tim3 =
                Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(TIM3_FREQ_HZ.hz());
            tim3.listen(Event::Update);

            *G_TIM2.borrow(cs).borrow_mut() = Some(tim2);
            *G_TIM3.borrow(cs).borrow_mut() = Some(tim3);

            let mut nvic = cp.NVIC;
            unsafe {
                nvic.set_priority(Interrupt::TIM3, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);

                nvic.set_priority(Interrupt::TIM2, 16);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
            }
            cortex_m::peripheral::NVIC::unpend(Interrupt::TIM2);
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
