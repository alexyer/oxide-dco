#![no_main]
#![no_std]

use panic_semihosting as _;

use stm32f1xx_hal as hal;

use crate::hal::{
    adc, gpio,
    gpio::{gpioa, gpiob, Alternate, PushPull},
    pac::{interrupt, Interrupt, Peripherals, ADC1, TIM2, TIM3},
    prelude::*,
    pwm::{Channel, Pwm, C1, C2, C3, C4},
    timer::{CountDownTimer, Event, Tim2NoRemap, Timer},
};

use cortex_m_rt::entry;

use core::cell::RefCell;
use cortex_m::asm::wfi;
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::Peripherals as c_m_Peripherals;
use eurorack_oxide_utils::voct::MvOct;

type AdcCh = gpiob::PB0<gpio::Analog>;
type Pwm1 = Pwm<
    TIM2,
    Tim2NoRemap,
    (C1, C2, C3, C4),
    (
        gpioa::PA0<Alternate<PushPull>>,
        gpio::gpioa::PA1<Alternate<PushPull>>,
        gpio::gpioa::PA2<Alternate<PushPull>>,
        gpio::gpioa::PA3<Alternate<PushPull>>,
    ),
>;

const TIM_ADC_FREQ: u32 = 40000; // in Hz
static mut AVG_BUF: [u32; 20] = [0; 20];

static G_ADC1: Mutex<RefCell<Option<adc::Adc<ADC1>>>> = Mutex::new(RefCell::new(None));
static G_CH0: Mutex<RefCell<Option<AdcCh>>> = Mutex::new(RefCell::new(None));
static G_PWM: Mutex<RefCell<Option<Pwm1>>> = Mutex::new(RefCell::new(None));
static G_TIM_ADC: Mutex<RefCell<Option<CountDownTimer<TIM3>>>> = Mutex::new(RefCell::new(None));

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
    static mut PWM: Option<Pwm1> = None;
    static mut TIM_ADC: Option<CountDownTimer<TIM3>> = None;

    static mut AVG_BUF_INDEX: u16 = 0;
    static mut PREV_MEASURE: i16 = 0;

    let adc1 = get_peripheral_ref!(ADC, G_ADC1);
    let ch0 = get_peripheral_ref!(CH0, G_CH0);
    let pwm = get_peripheral_ref!(PWM, G_PWM);
    let tim_adc = get_peripheral_ref!(TIM_ADC, G_TIM_ADC);

    let measure: u32 = adc1.read(ch0).unwrap();

    if (measure as i16 - *PREV_MEASURE).abs() > 60 {
        unsafe {
            AVG_BUF[*AVG_BUF_INDEX as usize] = measure;
            *AVG_BUF_INDEX = (*AVG_BUF_INDEX + 1) % AVG_BUF.len() as u16;

            let avg: u32 = AVG_BUF.iter().sum::<u32>() / AVG_BUF.len() as u32;

            let freq = (MvOct(avg as f32 * 1.209 + 1000.0).us()) as u32;
            pwm.set_period(freq.us());
            pwm.set_duty(Channel::C1, pwm.get_max_duty() / 2);
        }
    }

    // *PREV_MEASURE = measure as i16;
    tim_adc.wait().ok();
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
            let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

            // Init clocks
            let clocks = rcc
                .cfgr
                .adcclk(50.khz())
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
                1.ms(),
            );
            pwm.set_duty(Channel::C1, pwm.get_max_duty() / 2);
            pwm.enable(Channel::C1);

            // Init ADC
            let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
            adc1.set_sample_time(adc::SampleTime::T_239);
            let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
            let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

            *G_ADC1.borrow(cs).borrow_mut() = Some(adc1);
            *G_CH0.borrow(cs).borrow_mut() = Some(ch0);
            *G_PWM.borrow(cs).borrow_mut() = Some(pwm);

            // Init ADC timer
            // Set up a timer expiring after 1s
            let mut tim_adc =
                Timer::tim3(dp.TIM3, &clocks, &mut rcc.apb1).start_count_down(TIM_ADC_FREQ.hz());

            // Generate an interrupt when the timer expires
            tim_adc.listen(Event::Update);

            // Move the timer into our global storage
            *G_TIM_ADC.borrow(cs).borrow_mut() = Some(tim_adc);

            // Enable TIM2 IRQ, set prio 1 and clear any pending IRQs
            let mut nvic = cp.NVIC;
            // Calling 'set_priority()' and 'unmask()' requires 'unsafe {}'
            // - https://docs.rs/stm32f1xx-hal/0.5.3/stm32f1xx_hal/stm32/struct.NVIC.html#method.set_priority
            unsafe {
                nvic.set_priority(Interrupt::TIM3, 1);
                cortex_m::peripheral::NVIC::unmask(Interrupt::TIM3);
            }
            // Clear the interrupt state
            cortex_m::peripheral::NVIC::unpend(Interrupt::TIM3);
        });
    } else if cfg!(debug_assertions) {
        panic!("Failed to initialize timer: can't take Peripherals");
    }
}
