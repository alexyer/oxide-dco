#![no_main]
#![no_std]

// TODO(alexyer): Update to conditionally compile to halt for release.
use panic_semihosting as _;

use rtfm::app;

use stm32f1xx_hal as hal;

use crate::hal::{
    adc, gpio, pac,
    prelude::*,
    rcc::Enable,
    timer::{CountDownTimer, Event, Timer},
};

use core::sync::atomic::{AtomicU32, Ordering};

use eurorack_oxide_utils::voct::MvOct;
use eurorack_oxide_utils::voct::Voltage;

const AVG_BUF_SIZE: usize = 32;
const TIM3_FREQ_HZ: u32 = 200000;
const SEC_IN_US: u32 = 1000000;

const fn circle_time() -> u32 {
    SEC_IN_US / TIM3_FREQ_HZ
}

fn us_to_period(us: u32) -> u32 {
    us / circle_time() / 2
}

fn avg(buf: &mut [u16; AVG_BUF_SIZE]) -> u32 {
    let mut acc: u32 = 0;
    for i in 0..buf.len() {
        acc += buf[i] as u32;
    }

    acc / AVG_BUF_SIZE as u32
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        adc1: adc::Adc<pac::ADC1>,
        ch0: gpio::gpiob::PB0<gpio::Analog>,
        dac: pac::GPIOA,
        tim2: CountDownTimer<pac::TIM2>,
        tim3: CountDownTimer<pac::TIM3>,
        out: gpio::gpiob::PB1<gpio::Output<gpio::PushPull>>,

        #[init([0; AVG_BUF_SIZE])]
        avg_buf: [u16; AVG_BUF_SIZE],

        #[init(AtomicU32::new(0))]
        period: AtomicU32,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        // Init clocks
        let clocks = rcc
            .cfgr
            .adcclk(10.mhz())
            .sysclk(30.mhz())
            .pclk1(15.mhz())
            .freeze(&mut flash.acr);

        // Init ADC
        let mut adc1 = adc::Adc::adc1(cx.device.ADC1, &mut rcc.apb2, clocks);
        adc1.set_sample_time(adc::SampleTime::T_239);
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);

        // Init timers
        let mut tim2 = Timer::tim2(cx.device.TIM2, &clocks, &mut rcc.apb1)
            .start_count_down((TIM3_FREQ_HZ / 2).hz());
        tim2.listen(Event::Update);

        let mut tim3 =
            Timer::tim3(cx.device.TIM3, &clocks, &mut rcc.apb1).start_count_down(TIM3_FREQ_HZ.hz());
        tim3.listen(Event::Update);

        // Configure out pin
        let out = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

        // Init DAC port
        let dac = cx.device.GPIOA;
        pac::GPIOA::enable(&mut rcc.apb2);
        dac.crl.write(|w| unsafe { w.bits(0x33333333) });

        init::LateResources {
            adc1,
            ch0,
            tim2,
            tim3,
            out,
            dac,
        }
    }

    #[task(binds = TIM3, priority = 2, resources = [out, tim3, &period])]
    fn tick(cx: tick::Context) {
        static mut COUNTER: u32 = 0;

        if *COUNTER >= cx.resources.period.load(Ordering::Relaxed) {
            cx.resources.out.toggle().ok();
            *COUNTER = 0;
        }
        *COUNTER += 1;

        cx.resources.tim3.clear_update_interrupt_flag();
    }

    #[task(binds = TIM2, priority = 1, resources = [adc1, avg_buf, ch0, dac, &period, tim2])]
    fn measure(cx: measure::Context) {
        static mut AVG_COUNTER: usize = 0;

        cx.resources.avg_buf[*AVG_COUNTER % AVG_BUF_SIZE] =
            cx.resources.adc1.read(cx.resources.ch0).unwrap();
        *AVG_COUNTER += 1;

        if *AVG_COUNTER % AVG_BUF_SIZE == 0 {
            let avg = avg(cx.resources.avg_buf);
            let voltage = avg as f32 * 1191.55555 / cx.resources.adc1.read_vref() as f32;
            // let voltage = avg as f32 * 1.237740204;
            let mv = MvOct(6000.0 - 2.0 * voltage);
            // let mv = MvOct(voltage as f32 * 1.5015 as f32);

            cx.resources
                .period
                .store(us_to_period(mv.us()), Ordering::Relaxed);

            cx.resources
                .dac
                .odr
                .write(|w| unsafe { w.bits((mv.hz() / 16.0) as u32) });
        }

        cx.resources.tim2.clear_update_interrupt_flag();
    }
};
