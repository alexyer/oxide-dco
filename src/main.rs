#![no_main]
#![no_std]

// TODO(alexyer): Update to conditionally compile to halt for release.
use panic_semihosting as _;

use rtfm::app;

use cortex_m::peripheral::DWT;
use embedded_graphics::{
    egcircle,
    fonts::{Font6x8, Text},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::Circle,
    primitive_style,
    style::{TextStyleBuilder, PrimitiveStyle, Styled},
};
use embedded_hal::digital::v2::OutputPin;
use ssd1306::{prelude::*, Builder};
use stm32f0xx_hal as hal;

use crate::hal::{
    adc, gpio,
    // gpio::ExtiPin,
    // i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
    // rcc::Enable,
    timers::{Event, Timer},
};

use embedded_graphics::style::TextStyle;
use eurorack_oxide_utils::voct::MvOct;
use eurorack_oxide_utils::voct::Voltage;
// use rtfm::cyccnt::{U32Ext, Instant};
use stm32f0xx_hal::delay::Delay;
use core::fmt::Binary;
use cortex_m_semihosting::hprintln;
use core::ptr;
use cortex_m::peripheral::syst::SystClkSource::Core;
use stm32f0xx_hal::pac::tim1::smcr::MSM_A::SYNC;

const AVG_BUF_SIZE: usize = 32;
const TIM14_FREQ_HZ: u32 = 200000;
const SEC_IN_US: u32 = 1000000;
const FINE_TUNE_STEP: i16 = 2;
const SYSCLK_FREQ_HZ: u32 = 48_000_000;

static mut COUNTER: u32 = 0;
static mut FINE_TUNE: u32 = 0;
static mut PERIOD: u32 = 0;

const fn circle_time() -> u32 {
    SEC_IN_US / TIM14_FREQ_HZ
}

// type I2C = BlockingI2c<
//     pac::I2C1,
//     (
//         gpio::gpiob::PB8<gpio::Alternate<gpio::OpenDrain>>,
//         gpio::gpiob::PB9<gpio::Alternate<gpio::OpenDrain>>,
//     ),
// >;

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

// fn gui_intro(display: &mut GraphicsMode<I2cInterface<I2C>>) {
//     let text_style = TextStyleBuilder::new(Font6x8)
//             .text_color(BinaryColor::On)
//             .build();
//
//     let size = display.size();
//
//     Text::new("RUDEBOY", Point::new((size.width / 2 - 21) as i32, (size.height / 2) as i32))
//         .into_styled(text_style)
//         .draw(display)
//         .unwrap();
//
//     display.flush().unwrap();
// }

// fn gui_show(display: &mut GraphicsMode<I2cInterface<I2C>>) {
//     display.clear();
//
//     let text_style = TextStyleBuilder::new(Font6x8)
//         .text_color(BinaryColor::On)
//         .build();
//
//     let size = display.size();
//
//     let circle_1: Styled<Circle, PrimitiveStyle<BinaryColor>> = egcircle!(
//     center = Point::new((size.width / 2) as i32, size.height as i32),
//     radius = size.width / 3,
//     style = primitive_style!(
//         stroke_color = BinaryColor::On,
//         stroke_width = 2
//        )
//     );
//
//     circle_1.draw(display).unwrap();
//
//     Text::new("-12", Point::new(1_i32, (size.height - 8) as i32))
//         .into_styled(text_style)
//         .draw(display)
//         .unwrap();
//
//     Text::new("+12", Point::new((size.width - 18) as i32, (size.height - 8) as i32))
//         .into_styled(text_style)
//         .draw(display)
//         .unwrap();
//
//     display.flush().unwrap();
// }

#[app(device = stm32f0xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        adc: adc::Adc,
        ch0: gpio::gpiob::PB0<gpio::Analog>,
        // delay: Delay,
        // display: GraphicsMode<I2cInterface<I2C>>,
        // exti: pac::EXTI,
        gpioa: pac::GPIOA,
        // hard_sync: gpio::gpiob::PB5<gpio::Input<gpio::Floating>>,
        out: gpio::gpiob::PB1<gpio::Output<gpio::PushPull>>,
        tim3: Timer<pac::TIM3>,
        tim14: Timer<pac::TIM14>,

        #[init([0; AVG_BUF_SIZE])]
        avg_buf: [u16; AVG_BUF_SIZE],
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut flash = cx.device.FLASH;
        let mut rcc = cx.device.RCC;
        // let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        // Init clocks
        let mut clocks = rcc
            .configure()
            .sysclk(SYSCLK_FREQ_HZ.hz())
            .hclk(SYSCLK_FREQ_HZ.hz())
            .pclk(SYSCLK_FREQ_HZ.hz())
            .freeze(&mut flash);

        // let mut syst = cx.core.SYST;
        // syst.set_clock_source(Core);
        // syst.set_reload(SYSCLK_FREQ_HZ - 1);
        // syst.enable_counter();
        // syst.enable_interrupt();

        // Init ADC and out pin
        let mut adc = adc::Adc::new(cx.device.ADC, &mut clocks);
        adc.set_sample_time(adc::AdcSampleTime::T_55);
        let mut gpiob = cx.device.GPIOB.split(&mut clocks);

        let (mut out, mut ch0) = cortex_m::interrupt::free(move |cs| {
            (
                gpiob.pb1.into_push_pull_output(cs),
                gpiob.pb0.into_analog(cs)
            )
        });

        // Init timers
        let mut tim3 = Timer::tim3(cx.device.TIM3, (TIM14_FREQ_HZ / 2).hz(), &mut clocks);
        tim3.listen(Event::TimeOut);

        let mut tim14 =
            Timer::tim14(cx.device.TIM14, TIM14_FREQ_HZ.hz(), &mut clocks);
        tim14.listen(Event::TimeOut);

        // Init DAC port
        // fixme(alexyer): unsafe port initialization.
        let gpioa = cx.device.GPIOA;

        unsafe {
            let ahbenr = ptr::read(0x40021014 as *const u32);
            ptr::write(0x40021014 as *mut u32, ahbenr | 1 << 17);
            ptr::write(0x48000000 as *mut u16, 0x5555);
        }

        // // Init Hard Sync pin
        // let mut hard_sync = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        // hard_sync.make_interrupt_source(&mut afio);
        // hard_sync.trigger_on_edge(&cx.device.EXTI, gpio::Edge::RISING);
        // hard_sync.enable_interrupt(&cx.device.EXTI);

        // Init Encoder
        // Into pull up input
        // gpioa.crh.write(|w| unsafe { w.bits(0x8800) });
        // gpioa.bsrr.write(|w| unsafe { w.bits(1 << 10) });
        // gpioa.bsrr.write(|w| unsafe { w.bits(1 << 11) });
        //
        // // Make interrupt source
        // afio.exticr3
        //     .exticr3()
        //     .modify(|r, w| unsafe { w.bits((r.bits() & !(0xf << 10)) | (0 << 10)) });
        //
        // // Trigger on Falling edge
        // cx.device
        //     .EXTI
        //     .ftsr
        //     .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 10)) });
        // cx.device
        //     .EXTI
        //     .rtsr
        //     .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 10)) });
        //
        // // Enable EXTI interrupt
        // cx.device
        //     .EXTI
        //     .imr
        //     .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 10)) });
        // let exti = cx.device.EXTI;

        // Init display
        // let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        // let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);
        //
        // let mut i2c = BlockingI2c::i2c1(
        //     cx.device.I2C1,
        //     (scl, sda),
        //     &mut afio.mapr,
        //     Mode::Fast {
        //         frequency: 400_000.hz(),
        //         duty_cycle: DutyCycle::Ratio2to1,
        //     },
        //     clocks,
        //     &mut rcc.apb1,
        //     1000,
        //     10,
        //     1000,
        //     1000,
        // );

        // let cp = cortex_m::Peripherals::take().unwrap();
        // let delay = Delay::new(cp.SYST, clocks);

        // let mut display: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
        // display.init().unwrap();

        init::LateResources {
            adc,
            ch0,
            // delay,
            // display,
            // exti,
            gpioa,
            // hard_sync,
            out,
            tim3,
            tim14,
        }
    }

    // #[idle(resources = [delay, display])]
    #[idle(resources = [])]
    fn idle(cx: idle::Context) -> ! {
        // gui_intro(cx.resources.display);
        // cx.resources.delay.delay_ms(500_u16);
        // gui_show(cx.resources.display);

        loop {
            cortex_m::asm::wfi();
        }
}

    // #[task(binds = EXTI15_10, priority = 1, resources = [exti, &fine_tune, gpioa])]
    // #[task(binds = EXTI15_10, priority = 1, resources = [])]
    // fn encoder_handler(mut cx: encoder_handler::Context) {
    //     let bits = cx.resources.gpioa.lock(|gpioa| gpioa.idr.read().bits());
    //
    //     let state = (bits & (1 << 11)) == 0;
    //
    //     match state {
    //         true => cx
    //             .resources
    //             .fine_tune
    //             .fetch_add(FINE_TUNE_STEP, Ordering::Relaxed),
    //         false => cx
    //             .resources
    //             .fine_tune
    //             .fetch_add(-FINE_TUNE_STEP, Ordering::Relaxed),
    //     };
    //
    //     cx.resources.exti.pr.write(|w| unsafe { w.bits(1 << 10) });
    // }

    // #[task(binds = EXTI9_5, priority = 3, resources = [&counter, hard_sync])]
    // #[task(binds = EXTI9_5, priority = 3, resources = [])]
    // fn hard_sync(cx: hard_sync::Context) {
    //     cx.resources.counter.store(0, Ordering::Relaxed);
    //     cx.resources.hard_sync.clear_interrupt_pending_bit();
    // }

    #[task(binds = TIM14, priority = 4, resources = [out, tim14])]
    fn tick(cx: tick::Context) {
        unsafe {
            if COUNTER == 0 {
                cx.resources.out.set_low().ok();
            } else if COUNTER >= PERIOD {
                cx.resources.out.toggle().ok();
                COUNTER = 0;
            }

            COUNTER += 1;
        }
        cx.resources.tim14.wait().ok();
    }

    #[task(binds = TIM3, priority = 2, resources = [adc, avg_buf, ch0, gpioa, tim3])]
    fn measure(cx: measure::Context) {
        static mut AVG_COUNTER: usize = 0;
        let vrefint_cal = f32::from(unsafe { ptr::read(0x1FFF_F7BA as *const u16) });
        let r: u16 = cx.resources.adc.read(cx.resources.ch0).unwrap();
        // hprintln!("{}", adc::VRef::read_vdda(cx.resources.adc));

        cx.resources.avg_buf[*AVG_COUNTER % AVG_BUF_SIZE] = r;
        // cx.resources.avg_buf[*AVG_COUNTER % AVG_BUF_SIZE] = r * 3300;

        *AVG_COUNTER += 1;

        if *AVG_COUNTER % AVG_BUF_SIZE == 0 {
            let avg = avg(cx.resources.avg_buf);
            let voltage = avg * u32::from(adc::VRef::read_vdda(cx.resources.adc)) / 4095;
            // let voltage = (adc::VRef::read_vdda(cx.resources.adc) as f32 * avg) / (4095.0 * 0.6);
            // let voltage = (3300.0 * vrefint_cal * avg) / (adc::VRef::read_vdda(cx.resources.adc) as f32 * 4095.0);
            // let voltage = avg as f32 * 1.237740204;
            // hprintln!("{}", 6000.0 - 2.0 * voltage);
            let mv = MvOct(6000.0 - 2.0 * voltage as f32) + unsafe { FINE_TUNE } as f32;
            // let mv = MvOct(voltage as f32 * 1.5015 as f32);

            unsafe { PERIOD = us_to_period(mv.us()); }

            cx.resources.gpioa.odr.modify(|r, w| unsafe {
                w.bits((r.bits() & (0xff << 8)) | (mv.hz() / 16.0) as u32 & 0xff)
            });

            cx.resources.tim3.wait().ok();
        }

    }
};
