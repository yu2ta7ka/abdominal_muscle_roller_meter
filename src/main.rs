#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m;
use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::entry;

use crate::hal::{
    gpio::*,
    i2c::I2c,
    prelude::*,
    serial::config::Config,
    serial::Serial,
    spi::*,
    stm32,
    stm32::interrupt,
    timer::{Event, Timer},
};
use core::fmt::Write; // for pretty formatting of the serial output
use stm32f4xx_hal as hal;

use embedded_graphics::egtext;
use embedded_graphics::fonts::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::rectangle::Rectangle;
use embedded_graphics::primitives::Circle;
use embedded_graphics::style::PrimitiveStyle;
use embedded_graphics::style::PrimitiveStyleBuilder;
use embedded_graphics::text_style;
use heapless::String;
use st7735_lcd;
use st7735_lcd::Orientation;
use vl53l0x::VL53L0x;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::sync::atomic::{AtomicUsize, Ordering};

static TIMER_TIM2: Mutex<RefCell<Option<Timer<stm32::TIM2>>>> = Mutex::new(RefCell::new(None));
static COUNTER: AtomicUsize = AtomicUsize::new(0);
#[interrupt]
fn TIM2() {
    free(|cs| {
        if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
            // Clears interrupt associated with event.
            tim2.clear_interrupt(Event::TimeOut);
        }
        COUNTER.fetch_add(1, Ordering::Relaxed);
    });
}

#[derive(Copy, Clone)]
enum Direction {
    Init,
    Push,
    Pull,
    Rest,
}

impl core::fmt::Display for Direction {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Direction::Init => write!(f, "Init"),
            Direction::Push => write!(f, "Push"),
            Direction::Pull => write!(f, "Pull"),
            Direction::Rest => write!(f, "Rest"),
        }
    }
}

const DIST_SIZE: usize = 10;

pub struct AbMuCalc {
    distance: [u16; DIST_SIZE],
    past_direct: Direction,
    corrent_direct: Direction,
    shrink_dist: u16,
    extend_dist: u16,
}

impl AbMuCalc {
    pub fn new() -> AbMuCalc {
        AbMuCalc {
            distance: [0; DIST_SIZE],
            past_direct: Direction::Init,
            corrent_direct: Direction::Init,
            shrink_dist: core::u16::MAX,
            extend_dist: 0,
        }
    }

    pub fn calc_repetition(
        &mut self,
        dist: u16,
        tx: &mut stm32f4xx_hal::serial::Tx<stm32f4xx_hal::stm32::USART2>,
    ) -> bool {
        // calc direction
        for i in 0..self.distance.len() {
            if i < self.distance.len() - 1 {
                self.distance[i] = self.distance[i + 1];
            } else {
                self.distance[i] = dist;
            }
        }

        // difference
        let mut push_count = 0;
        let mut pull_count = 0;
        for i in 0..(self.distance.len() - 1) {
            if self.distance[i] > self.distance[i + 1] {
                push_count += 1;
            } else {
                pull_count += 1;
            }
        }

        // if direct is push, update shrink_dist
        if push_count == self.distance.len() - 1 {
            if self.distance[0] < self.shrink_dist {
                self.shrink_dist = self.distance[0];
            }
        }

        // if direct is push, update shrink_dist
        if pull_count == self.distance.len() - 1 {
            if self.distance[self.distance.len() - 1] > self.extend_dist {
                self.extend_dist = self.distance[self.distance.len() - 1];
            }
        }

        // I want to measure strength
        let diff_dist = if self.extend_dist > self.shrink_dist {
            self.extend_dist - self.shrink_dist
        } else {
            0
        };

        // judge
        if push_count == self.distance.len() - 1 {
            self.corrent_direct = Direction::Push;
        } else if pull_count == self.distance.len() - 1 {
            self.corrent_direct = Direction::Pull;
        }
        let ret = match (self.past_direct, self.corrent_direct) {
            (Direction::Push, Direction::Pull) => true,
            _ => false,
        };

        // Debug
        /*        writeln!(
                    tx,
                    "s_d:{} e_d:{} diff:{} distance: {:?} mm push_count:{} pull_count:{} past_direct:{} corrent_direct:{} \r",
                    self.shrink_dist, self.extend_dist, diff_dist, self.distance, push_count, pull_count, self.past_direct, self.corrent_direct
                )
                .unwrap();
        */
        self.past_direct = self.corrent_direct;
        ret
    }
}

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).pclk1(42.mhz()).freeze();
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    // Set up the interrupt timer
    // Generates an interrupt at 1 milli second intervals.
    let mut timer = Timer::tim2(dp.TIM2, 1000.hz(), clocks);
    timer.listen(Event::TimeOut);
    // Move the ownership of the period_timer to global.
    free(|cs| {
        TIMER_TIM2.borrow(cs).replace(Some(timer));
    });
    // Enable interrupt
    stm32::NVIC::unpend(stm32::Interrupt::TIM2);
    unsafe {
        stm32::NVIC::unmask(stm32::Interrupt::TIM2);
    }

    // LCD setting
    // for SPI communication
    let sck = gpioa.pa5.into_alternate_af5();
    let miso = gpioa.pa6.into_alternate_af5();
    let mosi = gpioa.pa7.into_alternate_af5();
    let rst = gpioc.pc0.into_push_pull_output();
    let dc = gpiob.pb0.into_push_pull_output();
    // GND connects to CS. Therefore, no code is reuired.
    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        16_000_000.hz(),
        clocks,
    );

    // display setting
    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 160, 128);
    disp.init(&mut delay).unwrap();
    disp.set_orientation(&Orientation::Landscape).unwrap();
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    let black_backdrop = Rectangle::new(Point::new(0, 0), Point::new(160, 128)).into_styled(style);
    black_backdrop.draw(&mut disp).unwrap();
    let black_count_area =
        Rectangle::new(Point::new(96, 0), Point::new(160, 32)).into_styled(style);

    let dist_black_count_area =
        Rectangle::new(Point::new(0, 40), Point::new(160, 72)).into_styled(style);

    // ToF sensor setting
    let scl = gpiob.pb6.into_alternate_af4_open_drain();
    let sda = gpiob.pb7.into_alternate_af4_open_drain();
    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);
    let mut tof = VL53L0x::new(i2c).unwrap();

    // Serial communication setting
    let tx_pin = gpioa.pa2.into_alternate_af7();
    let rx_pin = gpioa.pa3.into_alternate_af7();
    let serial = Serial::usart2(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(9600.bps()),
        clocks,
    )
    .unwrap();
    let (mut tx, mut _rx) = serial.split();

    writeln!(&mut tx, "{}!", "Abdominal muscle roller meter").unwrap();
    // draw to LCD
    let mut textbuffer: String<8> = String::new();
    write!(&mut textbuffer, "rep:0").unwrap();
    egtext!(
        text = textbuffer.as_str(),
        top_left = (0, 0),
        style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
    )
    .draw(&mut disp)
    .unwrap();

    let mut abmucalc = AbMuCalc::new();
    let mut repetition = 0;

    loop {
        let start_time = COUNTER.load(Ordering::Relaxed);
        // get dist data
        let dist = VL53L0x::read_range_single_millimeters_blocking(&mut tof).unwrap();
        let get_dist_time = COUNTER.load(Ordering::Relaxed);

        // calc repetition
        match abmucalc.calc_repetition(dist, &mut tx) {
            true => {
                repetition += 1;
                black_count_area.draw(&mut disp).unwrap();
                // draw to LCD
                let mut textbuffer: String<8> = String::new();
                write!(&mut textbuffer, "rep:{}", repetition).unwrap();
                egtext!(
                    text = textbuffer.as_str(),
                    top_left = (0, 0),
                    style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
                )
                .draw(&mut disp)
                .unwrap();
            }
            false => (),
        };
        let calc_rep_time = COUNTER.load(Ordering::Relaxed);

        // calc radius
        let mut radius: f32 = 48.0;
        if dist < 1000 {
            let rate = dist as f32 / 1000.0;
            radius *= rate;
        }

        // draw red circle
        Circle::new(Point::new(80, 80), 48)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::BLACK,48 - radius as u32))
            .draw(&mut disp)
            .unwrap();
        Circle::new(Point::new(80, 80), radius as u32)
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
            .draw(&mut disp)
            .unwrap();

        /*
        // Debug
        // draw dist
        dist_black_count_area.draw(&mut disp).unwrap();
        let mut textbuffer: String<8> = String::new();
        write!(&mut textbuffer, "{}", dist).unwrap();
        egtext!(
            text = textbuffer.as_str(),
            top_left = (0, 40),
            style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
        )
        .draw(&mut disp)
        .unwrap();

        // draw radius
        dist_black_count_area.draw(&mut disp).unwrap();
        let mut textbuffer: String<8> = String::new();
        write!(&mut textbuffer, "{}", radius as u32).unwrap();
        egtext!(
            text = textbuffer.as_str(),
            top_left = (0, 40),
            style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
        )
        .draw(&mut disp)
        .unwrap();
        */

        let draw_lcd_time = COUNTER.load(Ordering::Relaxed);

        // Debug
        //writeln!(tx, "loop time     {}ms\r", draw_lcd_time - start_time).unwrap();
        //writeln!(tx, "get dist time {}ms\r", get_dist_time - start_time).unwrap();
        //writeln!(tx, "calc rep time {}ms\r", calc_rep_time - get_dist_time).unwrap();
        //writeln!(tx, "draw lcd time {}ms\r", draw_lcd_time - calc_rep_time).unwrap();
    }
}
