#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{
    delay::Delay, gpio::*, i2c::I2c, prelude::*, serial::config::Config, serial::Serial, spi::*,
    stm32,
};

use core::fmt::Write; // for pretty formatting of the serial output
use vl53l0x::VL53L0x;

use embedded_graphics::egtext;
use embedded_graphics::fonts::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::pixelcolor::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::rectangle::Rectangle;
use embedded_graphics::primitives::Circle;
use embedded_graphics::style::PrimitiveStyleBuilder;
use embedded_graphics::text_style;
use heapless::String;
use st7735_lcd;
use st7735_lcd::Orientation;

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

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // init GPIO object
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    // init clock object
    let rcc = dp.RCC.constrain();
    //let clocks = rcc.cfgr.use_hse(8.mhz()).freeze();
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .freeze();
    // init delay object
    let mut delay = hal::delay::Delay::new(cp.SYST, clocks);

    // for SPI communication
    // PA5 connects to SCL/SCK on the LCD
    let sck = gpioa.pa5.into_alternate_af5();
    // PA6 does not get connected to the LCD
    let miso = gpioa.pa6.into_alternate_af5();
    // PA7 connects to SDA/MOSI on the LCD
    let mosi = gpioa.pa7.into_alternate_af5();
    // GND connects to CS. Therefore, no code is reuired.

    // PC0 connects to RST/RES on the LCD
    let rst = gpioc.pc0.into_push_pull_output();
    // PB0 connects to RS/DC on the LCD
    let dc = gpiob.pb0.into_push_pull_output();

    /* Notice this board is communicating over SPI_1. If it was some other SPI,
    the pins would be different depending on the alternate functions. The alternate
    function group could also end up being some number other than 5. */
    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        16_000_000.hz(),
        //8_000_000.hz(),
        clocks,
    );

    /* Remember the change the width and height to match your LCD screen.
    The RGB parameter specifies whether the LCD screen uses RGB or BGR for
    color. Your LCD might vary so if you find your blues are reds or vice
    versa change this parameter. */
    //let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, false, false, 128, 128);
    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 160, 128);

    // Initialize the display.
    disp.init(&mut delay).unwrap();
    // Set the orientation of the display
    disp.set_orientation(&Orientation::Landscape).unwrap();

    /* Create a style that specifies a color of RED. This will always use
    Rgb565 regardless if your board uses RGB or BGR. */
    //let style = PrimitiveStyleBuilder::new().fill_color(Rgb565::RED).build();
    let style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb565::BLACK)
        .build();

    /* Create a rectangle to fill the background. Make sure the second point
    has a width and height that matches your ST7735. */
    let black_backdrop = Rectangle::new(Point::new(0, 0), Point::new(160, 128)).into_styled(style);
    black_backdrop.draw(&mut disp).unwrap();

    let black_count_area = Rectangle::new(Point::new(96, 0), Point::new(160, 32)).into_styled(style);

    //set up I2C
    let scl = gpiob.pb6.into_alternate_af4_open_drain();
    let sda = gpiob.pb7.into_alternate_af4_open_drain();
    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);
    let mut tof = VL53L0x::new(i2c).unwrap();

    // define RX/TX pins
    let tx_pin = gpioa.pa2.into_alternate_af7();
    let rx_pin = gpioa.pa3.into_alternate_af7();
    // configure serial
    let serial = Serial::usart2(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(9600.bps()),
        clocks,
    )
    .unwrap();
    let (mut tx, mut _rx) = serial.split();

    writeln!(&mut tx, "this is {} example!", "ToF").unwrap();

    let mut repetition = 0;
    let mut distance: [u16; 4] = [0; 4];
    let mut past_direct = Direction::Init;
    let mut corrent_direct = Direction::Init;

    loop {
        // get dist data
        let dist = VL53L0x::read_range_single_millimeters_blocking(&mut tof).unwrap();
        for i in 0..4 {
            if i < 4 - 1 {
                distance[i] = distance[i + 1];
            } else {
                distance[i] = dist;
            }
        }

        // calc direction
        // difference
        let mut push_count = 0;
        let mut pull_count = 0;
        for i in 0..(4 - 1) {
            if distance[i] > distance[i + 1] {
                push_count += 1;
            } else {
                pull_count += 1;
            }
        }
        // judge
        if push_count == 3 {
            corrent_direct = Direction::Push;
        } else if pull_count == 3 {
            corrent_direct = Direction::Pull;
        }
        match (past_direct, corrent_direct) {
            (Direction::Push, Direction::Pull) => {
                repetition += 1;
                black_count_area.draw(&mut disp).unwrap();
            }
            _ => (),
        }

        writeln!(
            tx,
            "distance: {:?} mm past_direct:{} corrent_direct:{} rep:{}  \r",
            distance, past_direct, corrent_direct, repetition
        )
        .unwrap();

        let mut textbuffer: String<8> = String::new();
        write!(&mut textbuffer, "rep:{}", repetition).unwrap();
        egtext!(
            text = textbuffer.as_str(),
            top_left = (0, 0),
            style = text_style!(font = Font24x32, text_color = Rgb565::WHITE)
        )
        .draw(&mut disp)
        .unwrap();

        
        delay.delay_ms(500_u16);
        past_direct = corrent_direct;
    }
}
