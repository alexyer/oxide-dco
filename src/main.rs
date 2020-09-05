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
use stm32f1xx_hal as hal;

use crate::hal::{
    adc, gpio,
    gpio::ExtiPin,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac,
    prelude::*,
    rcc::Enable,
    timer::{CountDownTimer, Event, Timer},
};

use core::sync::atomic::{AtomicI16, AtomicU32, Ordering};

use embedded_graphics::style::TextStyle;
use eurorack_oxide_utils::voct::MvOct;
use eurorack_oxide_utils::voct::Voltage;
use rtfm::cyccnt::{U32Ext, Instant};
use stm32f1xx_hal::delay::Delay;
use core::fmt::Binary;

const AVG_BUF_SIZE: usize = 32;
const TIM3_FREQ_HZ: u32 = 200000;
const SEC_IN_US: u32 = 1000000;
const FINE_TUNE_STEP: i16 = 2;
const SYSCLK_FREQ_HZ: u32 = 30_000_000;

const fn circle_time() -> u32 {
    SEC_IN_US / TIM3_FREQ_HZ
}

type I2C = BlockingI2c<
    pac::I2C1,
    (
        gpio::gpiob::PB8<gpio::Alternate<gpio::OpenDrain>>,
        gpio::gpiob::PB9<gpio::Alternate<gpio::OpenDrain>>,
    ),
>;

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

fn gui_intro(display: &mut GraphicsMode<I2cInterface<I2C>>) {
    let text_style = TextStyleBuilder::new(Font6x8)
            .text_color(BinaryColor::On)
            .build();

    let size = display.size();

    Text::new("RUDEBOY", Point::new((size.width / 2 - 21) as i32, (size.height / 2) as i32))
        .into_styled(text_style)
        .draw(display)
        .unwrap();

    display.flush().unwrap();
}

fn gui_show(display: &mut GraphicsMode<I2cInterface<I2C>>) {
    display.clear();

    let text_style = TextStyleBuilder::new(Font6x8)
        .text_color(BinaryColor::On)
        .build();

    let size = display.size();

    let circle_1: Styled<Circle, PrimitiveStyle<BinaryColor>> = egcircle!(
    center = Point::new((size.width / 2) as i32, size.height as i32),
    radius = size.width / 3,
    style = primitive_style!(
        stroke_color = BinaryColor::On,
        stroke_width = 2
       )
    );

    circle_1.draw(display).unwrap();

    Text::new("-12", Point::new(1_i32, (size.height - 8) as i32))
        .into_styled(text_style)
        .draw(display)
        .unwrap();

    Text::new("+12", Point::new((size.width - 18) as i32, (size.height - 8) as i32))
        .into_styled(text_style)
        .draw(display)
        .unwrap();

    display.flush().unwrap();
}

#[app(device = stm32f1xx_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        adc1: adc::Adc<pac::ADC1>,
        ch0: gpio::gpiob::PB0<gpio::Analog>,
        delay: Delay,
        display: GraphicsMode<I2cInterface<I2C>>,
        exti: pac::EXTI,
        gpioa: pac::GPIOA,
        hard_sync: gpio::gpiob::PB5<gpio::Input<gpio::Floating>>,
        out: gpio::gpiob::PB1<gpio::Output<gpio::PushPull>>,
        tim2: CountDownTimer<pac::TIM2>,
        tim3: CountDownTimer<pac::TIM3>,

        #[init([0; AVG_BUF_SIZE])]
        avg_buf: [u16; AVG_BUF_SIZE],

        #[init(AtomicU32::new(0))]
        counter: AtomicU32,

        #[init(AtomicI16::new(0))]
        fine_tune: AtomicI16,

        #[init(AtomicU32::new(0))]
        period: AtomicU32,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();
        let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);

        // Init clocks
        let clocks = rcc
            .cfgr
            .adcclk(10.mhz())
            .sysclk(SYSCLK_FREQ_HZ.hz())
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

        // Init out pin
        let out = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

        // Init DAC port
        let gpioa = cx.device.GPIOA;
        pac::GPIOA::enable(&mut rcc.apb2);
        gpioa.crl.write(|w| unsafe { w.bits(0x33333333) });

        // Init Hard Sync pin
        let mut hard_sync = gpiob.pb5.into_floating_input(&mut gpiob.crl);
        hard_sync.make_interrupt_source(&mut afio);
        hard_sync.trigger_on_edge(&cx.device.EXTI, gpio::Edge::RISING);
        hard_sync.enable_interrupt(&cx.device.EXTI);

        // Init Encoder
        // Into pull up input
        gpioa.crh.write(|w| unsafe { w.bits(0x8800) });
        gpioa.bsrr.write(|w| unsafe { w.bits(1 << 10) });
        gpioa.bsrr.write(|w| unsafe { w.bits(1 << 11) });

        // Make interrupt source
        afio.exticr3
            .exticr3()
            .modify(|r, w| unsafe { w.bits((r.bits() & !(0xf << 10)) | (0 << 10)) });

        // Trigger on Falling edge
        cx.device
            .EXTI
            .ftsr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 10)) });
        cx.device
            .EXTI
            .rtsr
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 10)) });

        // Enable EXTI interrupt
        cx.device
            .EXTI
            .imr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 10)) });
        let exti = cx.device.EXTI;

        // Init display
        let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
        let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

        let mut i2c = BlockingI2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Fast {
                frequency: 400_000.hz(),
                duty_cycle: DutyCycle::Ratio2to1,
            },
            clocks,
            &mut rcc.apb1,
            1000,
            10,
            1000,
            1000,
        );

        let cp = cortex_m::Peripherals::take().unwrap();
        let delay = Delay::new(cp.SYST, clocks);

        let mut display: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
        display.init().unwrap();

        init::LateResources {
            adc1,
            ch0,
            delay,
            display,
            exti,
            gpioa,
            hard_sync,
            out,
            tim2,
            tim3,
        }
    }

    #[idle(resources = [delay, display])]
    fn idle(cx: idle::Context) -> ! {
        gui_intro(cx.resources.display);
        cx.resources.delay.delay_ms(500_u16);
        gui_show(cx.resources.display);

        loop {
            cortex_m::asm::wfi();
        }
}

    #[task(binds = EXTI15_10, priority = 1, resources = [exti, &fine_tune, gpioa])]
    fn encoder_handler(mut cx: encoder_handler::Context) {
        let bits = cx.resources.gpioa.lock(|gpioa| gpioa.idr.read().bits());

        let state = (bits & (1 << 11)) == 0;

        match state {
            true => cx
                .resources
                .fine_tune
                .fetch_add(FINE_TUNE_STEP, Ordering::Relaxed),
            false => cx
                .resources
                .fine_tune
                .fetch_add(-FINE_TUNE_STEP, Ordering::Relaxed),
        };

        cx.resources.exti.pr.write(|w| unsafe { w.bits(1 << 10) });
    }

    #[task(binds = EXTI9_5, priority = 3, resources = [&counter, hard_sync])]
    fn hard_sync(cx: hard_sync::Context) {
        cx.resources.counter.store(0, Ordering::Relaxed);
        cx.resources.hard_sync.clear_interrupt_pending_bit();
    }

    #[task(binds = TIM3, priority = 4, resources = [&counter, out, tim3, &period])]
    fn tick(cx: tick::Context) {
        let c = cx.resources.counter.load(Ordering::Relaxed);

        if c == 0 {
            cx.resources.out.set_low().ok();
        } else if c >= cx.resources.period.load(Ordering::Relaxed) {
            cx.resources.out.toggle().ok();
            cx.resources.counter.store(0, Ordering::Relaxed);
        }

        cx.resources.counter.fetch_add(1, Ordering::Relaxed);

        cx.resources.tim3.clear_update_interrupt_flag();
    }

    #[task(binds = TIM2, priority = 2, resources = [adc1, avg_buf, ch0, gpioa, &fine_tune, &period, tim2])]
    fn measure(cx: measure::Context) {
        static mut AVG_COUNTER: usize = 0;

        cx.resources.avg_buf[*AVG_COUNTER % AVG_BUF_SIZE] =
            cx.resources.adc1.read(cx.resources.ch0).unwrap();
        *AVG_COUNTER += 1;

        if *AVG_COUNTER % AVG_BUF_SIZE == 0 {
            let avg = avg(cx.resources.avg_buf);
            let voltage = avg as f32 * 1191.55555 / cx.resources.adc1.read_vref() as f32;
            // let voltage = avg as f32 * 1.237740204;
            let mv = MvOct(6000.0 - 2.0 * voltage)
                + cx.resources.fine_tune.load(Ordering::Relaxed) as f32;
            // let mv = MvOct(voltage as f32 * 1.5015 as f32);

            cx.resources
                .period
                .store(us_to_period(mv.us()), Ordering::Relaxed);

            cx.resources.gpioa.odr.modify(|r, w| unsafe {
                w.bits((r.bits() & (0xff << 8)) | (mv.hz() / 16.0) as u32 & 0xff)
            });
        }

        cx.resources.tim2.clear_update_interrupt_flag();
    }
};
