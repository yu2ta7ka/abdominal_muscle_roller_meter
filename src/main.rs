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

/*
static TIMER_TIM5: Mutex<RefCell<Option<Timer<stm32::TIM5>>>> = Mutex::new(RefCell::new(None));
#[interrupt]
fn TIM5() {
    free(|cs| {
        if let Some(ref mut tim5) = TIMER_TIM5.borrow(cs).borrow_mut().deref_mut() {
            // Clears interrupt associated with event.
            tim5.clear_interrupt(Event::TimeOut);
        }

    });
}
*/

#[derive(Copy, Clone)]
enum Direction {
    Init,
    Ready,
    Push,
    Pull,
}

impl core::fmt::Display for Direction {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Direction::Init => write!(f, "Init"),
            Direction::Ready => write!(f, "Ready"),
            Direction::Push => write!(f, "Push"),
            Direction::Pull => write!(f, "Pull"),
        }
    }
}

impl core::fmt::Debug for Direction {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Direction::Init => write!(f, "Init"),
            Direction::Ready => write!(f, "Ready"),
            Direction::Push => write!(f, "Push"),
            Direction::Pull => write!(f, "Pull"),
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
enum StrengthJudgment {
    Weak,
    Medium,
    Strong,
    None,
}

impl core::fmt::Display for StrengthJudgment {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            StrengthJudgment::Weak => write!(f, "Weak"),
            StrengthJudgment::Medium => write!(f, "Medium"),
            StrengthJudgment::Strong => write!(f, "Strong"),
            StrengthJudgment::None => write!(f, "None"),
        }
    }
}

impl core::fmt::Debug for StrengthJudgment {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            StrengthJudgment::Weak => write!(f, "Weak"),
            StrengthJudgment::Medium => write!(f, "Medium"),
            StrengthJudgment::Strong => write!(f, "Strong"),
            StrengthJudgment::None => write!(f, "None"),
        }
    }
}

const LCD_WIDTH: u32 = 160;
const LCD_HIGHT: u32 = 128;

const DIST_SIZE: usize = 5;
const START_DIST_MAX: u16 = 1300; // Distance with the abdominal muscles shrinked. Maximum starting position.
const START_DIST_MIN: u16 = 1000; // Distance with the abdominal muscles shrinked. Minimum starting position.
const END_DIST_MIN: u16 = 0; // Distance with the abdominal muscles extended. End position.

#[derive(Debug)]
pub struct AbMuCalc {
    distance: [u16; DIST_SIZE],
    past_direct: Direction,
    current_direct: Direction,
    start_dist: u16,
    end_dist: u16,
    start_time: usize,
    strength: StrengthJudgment,
    half_rep: bool,
}

impl AbMuCalc {
    pub fn new() -> AbMuCalc {
        AbMuCalc {
            distance: [0; DIST_SIZE],
            past_direct: Direction::Init,
            current_direct: Direction::Init,
            start_dist: START_DIST_MAX,
            end_dist: END_DIST_MIN,
            start_time: 0,
            strength: StrengthJudgment::None,
            half_rep: false,
        }
    }

    pub fn calc_repetition(
        &mut self,
        dist: u16,
        _tx: &mut stm32f4xx_hal::serial::Tx<stm32f4xx_hal::stm32::USART2>,
    ) {
        for i in 0..self.distance.len() {
            // deleted oldest distance[0], added latest distance[9]
            if i < self.distance.len() - 1 {
                self.distance[i] = self.distance[i + 1];
            } else {
                self.distance[i] = dist;
            }
        }
        // If it is outside the measurement distance range, repeticion is not calculated.
        if self.distance[DIST_SIZE - 1] > START_DIST_MAX {
            self.start_dist = START_DIST_MAX;
            self.end_dist = END_DIST_MIN;
            self.past_direct = Direction::Init;
            self.current_direct = Direction::Init;
            self.half_rep = false;
            return;
        } else if START_DIST_MIN < self.distance[DIST_SIZE - 1]
            && self.distance[DIST_SIZE - 1] < START_DIST_MAX
        {
            self.current_direct = Direction::Ready;
        }

        let mut push_count = 0;
        let mut pull_count = 0;
        for i in 0..(self.distance.len() - 1) {
            // The direction is counted from the difference in distance.

            if self.distance[i] > self.distance[i + 1] {
                push_count += 1;
            } else if self.distance[i] <= self.distance[i + 1] {
                pull_count += 1;
            }

            self.start_time = match self.past_direct {
                Direction::Ready => COUNTER.load(Ordering::Relaxed),
                _ => self.start_time,
            };
        }

        // Judgment of direction
        match self.current_direct {
            Direction::Ready => {
                if push_count == self.distance.len() - 1 {
                    self.current_direct = Direction::Push;
                }
            }
            Direction::Push => {
                if push_count == self.distance.len() - 1 {
                    self.current_direct = Direction::Push;
                } else if pull_count == self.distance.len() - 1 {
                    self.current_direct = Direction::Pull;
                }
            }
            Direction::Pull => {
                if push_count == self.distance.len() - 1 {
                    self.current_direct = Direction::Push;
                } else if pull_count == self.distance.len() - 1 {
                    self.current_direct = Direction::Pull;
                }
            }
            Direction::Init => (),
        };

        // Judgment of REP
        match (self.past_direct, self.current_direct) {
            (Direction::Ready, Direction::Push) => {
                self.start_dist = self.distance[0];
            }
            (Direction::Push, Direction::Pull) => {
                self.end_dist = self.distance[0];
                self.half_rep = true;
            }
            _ => (),
        };
        let end_time = if let Direction::Ready = self.current_direct {
            if let Direction::Pull = self.past_direct {
                COUNTER.load(Ordering::Relaxed)
            } else {
                core::usize::MAX
            }
        } else {
            core::usize::MAX
        };

        // Strength judgment information (moving distance)
        let diff_dist = if self.end_dist < self.start_dist {
            self.start_dist - self.end_dist
        } else {
            core::u16::MAX
        };

        // Judgment of strength
        if self.half_rep == true
            && START_DIST_MIN < self.distance[DIST_SIZE - 1]
            && self.distance[DIST_SIZE - 1] < START_DIST_MAX
        {
            // kokoni jikan no kyoudo jyouken mo jissou suru
            if diff_dist <= 400 {
                self.strength = StrengthJudgment::Weak;
            } else if 400 < diff_dist && diff_dist <= 800 {
                self.strength = StrengthJudgment::Medium;
            } else if 800 < diff_dist && diff_dist <= START_DIST_MAX {
                self.strength = StrengthJudgment::Strong;
            }
            self.half_rep = false;
        } else {
            self.strength = StrengthJudgment::None;
        }

        // Debug
        writeln!(
            _tx,
            "s_t:{} e_t:{} time:{} s_d:{} e_d:{} diff:{} dist: {:?} mm ps_cnt:{} pl_cnt:{} past_dire:{} corrent_dire:{} \r",
            self.start_time, end_time,end_time - self.start_time, self.start_dist, self.end_dist, diff_dist, self.distance, push_count, pull_count, self.past_direct, self.current_direct
        ).unwrap();

        self.past_direct = self.current_direct;
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

    /*
     TODO: Use interrupt prcoessing from sensing ToF to calc REP.
    // Set up the interrupt timer
     // Generates an interrupt at 5 milli second intervals.
     let mut timer5 = Timer::tim5(dp.TIM5, 500.hz(), clocks);
     timer5.listen(Event::TimeOut);
     // Move the ownership of the period_timer to global.
     free(|cs| {
         TIMER_TIM5.borrow(cs).replace(Some(timer5));
     });
     // Enable interrupt
     stm32::NVIC::unpend(stm32::Interrupt::TIM5);
     unsafe {
         stm32::NVIC::unmask(stm32::Interrupt::TIM5);
     }
     */

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
    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, LCD_WIDTH, LCD_HIGHT);
    disp.init(&mut delay).unwrap();
    disp.set_orientation(&Orientation::Landscape).unwrap();
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();
    let black_backdrop = Rectangle::new(
        Point::new(0, 0),
        Point::new(LCD_WIDTH as i32, LCD_HIGHT as i32),
    )
    .into_styled(style);
    black_backdrop.draw(&mut disp).unwrap();

    let black_count_area =
        Rectangle::new(Point::new(0, 0), Point::new(LCD_WIDTH as i32, 32)).into_styled(style);

    // Debug
    let _dist_black_count_area =
        Rectangle::new(Point::new(0, 40), Point::new(LCD_WIDTH as i32, 72)).into_styled(style);

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
    write!(&mut textbuffer, "S0M0W0").unwrap();
    egtext!(
        text = textbuffer.as_str(),
        top_left = (0, 0),
        style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
    )
    .draw(&mut disp)
    .unwrap();

    let mut abmucalc = AbMuCalc::new();
    let mut s_repetition = 0;
    let mut m_repetition = 0;
    let mut w_repetition = 0;
    let mut circle_color = Rgb565::BLACK;

    loop {
        let _start_time = COUNTER.load(Ordering::Relaxed);
        // get dist data
        let dist = VL53L0x::read_range_single_millimeters_blocking(&mut tof).unwrap();
        let _get_dist_time = COUNTER.load(Ordering::Relaxed);

        // calc repetition
        abmucalc.calc_repetition(dist, &mut tx);

        match abmucalc.strength {
            StrengthJudgment::None => (),
            StrengthJudgment::Strong => s_repetition += 1,

            StrengthJudgment::Medium => m_repetition += 1,

            StrengthJudgment::Weak => w_repetition += 1,
        };

        match abmucalc.strength {
            StrengthJudgment::None => (),
            _ => {
                black_count_area.draw(&mut disp).unwrap();
                // draw to LCD
                let mut textbuffer: String<8> = String::new();
                write!(
                    &mut textbuffer,
                    "S{}M{}W{}",
                    s_repetition, m_repetition, w_repetition
                )
                .unwrap();
                egtext!(
                    text = textbuffer.as_str(),
                    top_left = (0, 0),
                    style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
                )
                .draw(&mut disp)
                .unwrap();
            }
        };

        let _calc_rep_time = COUNTER.load(Ordering::Relaxed);

        // calc radius
        let mut radius: f32 = 48.0;
        if dist < START_DIST_MIN {
            let rate = dist as f32 / START_DIST_MIN as f32;
            radius *= rate;
        }

        // draw circle
        let update_color = match abmucalc.current_direct {
            Direction::Init => Rgb565::YELLOW,
            Direction::Ready => Rgb565::WHITE,
            Direction::Push => Rgb565::BLUE,
            Direction::Pull => Rgb565::RED,
        };

        if circle_color != update_color {
            circle_color = update_color;
            Circle::new(Point::new(80, 80), radius as u32)
                .into_styled(PrimitiveStyle::with_fill(circle_color))
                .draw(&mut disp)
                .unwrap();
            Circle::new(Point::new(80, 80), 48)
                .into_styled(PrimitiveStyle::with_stroke(
                    Rgb565::BLACK,
                    48 - radius as u32,
                ))
                .draw(&mut disp)
                .unwrap();

            if circle_color == Rgb565::YELLOW || circle_color == Rgb565::WHITE {
                // draw dist
                _dist_black_count_area.draw(&mut disp).unwrap();
                let mut textbuffer: String<8> = String::new();
                write!(&mut textbuffer, "{}", dist).unwrap();
                egtext!(
                    text = textbuffer.as_str(),
                    top_left = (0, 40),
                    style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
                )
                .draw(&mut disp)
                .unwrap();
            }
        }

        let _draw_lcd_time = COUNTER.load(Ordering::Relaxed);

        // Debug
        //writeln!(tx, "loop time     {}ms\r", draw_lcd_time - start_time).unwrap();
        //writeln!(tx, "get dist time {}ms\r", get_dist_time - start_time).unwrap();
        //writeln!(tx, "calc rep time {}ms\r", calc_rep_time - get_dist_time).unwrap();
        //writeln!(tx, "draw lcd time {}ms\r", _draw_lcd_time - _calc_rep_time).unwrap();
    }
}
