#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::String;
use core::fmt::Write;
use core::mem::MaybeUninit;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Text};
use esp_backtrace as _;
use esp_println::println;
use hal::gpio::NO_PIN;
use hal::prelude::_fugit_RateExtU32;
use hal::timer::systimer::SystemTimer;
use hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::timg::TimerGroup,
    delay::Delay, rtc_cntl::Rtc,
    gpio::{Io, Output, Level},
};
use hal::spi::master::Spi;


#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {    
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[hal::entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LPWR);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
    );
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();
    println!("Hello world!");

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    // Set GPIO4 as an output, and set its state high initially.
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio38, Level::Low);
    //let user_btn = io.pins.gpio21.into_pull_down_input();
    //let boot0_btn = io.pins.gpio0.into_pull_up_input(); // default pull up
    println!("GPIO init OK");

    let sclk = io.pins.gpio47;
    let rst = io.pins.gpio17;
    let cs = io.pins.gpio6;

    let d0 = io.pins.gpio18;
    let d1 = io.pins.gpio7;
    let d2 = io.pins.gpio48;
    let d3 = io.pins.gpio5;

    let mut rst = Output::new(rst, Level::Low);

    led.set_high();
    let spi = Spi::new_half_duplex(
        peripherals.SPI2, // use spi2 host
        75_u32.MHz(), // max 75MHz
        hal::spi::SpiMode::Mode0,
        &clocks)
        .with_pins(Some(sclk),Some(d0),Some(d1),Some(d2),Some(d3),NO_PIN);

    let cs = Output::new(cs, Level::High);

    let mut display = t_display_s3_amoled::rm67162::RM67162::new(spi, cs);
    display.reset(&mut rst, &mut delay).unwrap();
    println!("reset display");
    display.init(&mut delay).unwrap();
    display
        .set_orientation(t_display_s3_amoled::rm67162::Orientation::LandscapeFlipped)
        .unwrap();

    println!("init display");

    display.clear(Rgb565::WHITE).unwrap();
    println!("screen init ok");

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::RED);
    Text::with_alignment(
        "Hello,\nRust World!",
        Point::new(300, 20),
        character_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

    let mut cnt = 0;
    let started = now_ms();

    loop {
        // fps testing
        let mut s = String::new();

        let elapsed = now_ms() - started;
        core::write!(
            &mut s,
            "Frames: {}\nFPS: {:.1}",
            cnt,
            if elapsed > 0 {
                cnt as f32 / (elapsed as f32 / 1000.0)
            } else {
                0.0
            }
        )
        .unwrap();
        Text::with_alignment(
            &s,
            Point::new(100, 40),
            MonoTextStyleBuilder::new()
                .background_color(Rgb565::BLACK)
                .text_color(Rgb565::CSS_BISQUE)
                .font(&FONT_10X20)
                .build(),
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
        cnt += 1;
    }
}

fn now_ms() -> u64 {
    SystemTimer::now() * 1_000 / SystemTimer::TICKS_PER_SECOND
}
