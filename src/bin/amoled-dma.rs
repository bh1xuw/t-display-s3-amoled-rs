#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::String;
use core::fmt::Write;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Text};
use esp_backtrace as _;
use esp_println::println;
use hal::dma::DmaPriority;
use hal::gdma::Gdma;
use hal::gpio::NO_PIN;
use hal::prelude::_fugit_RateExtU32;
use hal::systimer::SystemTimer;
use hal::{
    adc::{AdcConfig, Attenuation, ADC, ADC1},
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay, Rtc, Spi, IO,
};
use t_display_s3_amoled::rm67162::Orientation;
#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;

    extern "C" {
        static mut _heap_start: u32;
        static mut _heap_end: u32;
    }

    unsafe {
        let heap_start = &_heap_start as *const _ as usize;
        let heap_end = &_heap_end as *const _ as usize;
        assert!(
            heap_end - heap_start > HEAP_SIZE,
            "Not enough available heap memory."
        );
        ALLOCATOR.init(heap_start as *mut u8, HEAP_SIZE);
    }
}

#[hal::entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
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
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio38.into_push_pull_output();
    let user_btn = io.pins.gpio21.into_pull_down_input();
    let boot0_btn = io.pins.gpio0.into_pull_up_input(); // default pull up

    led.set_high().unwrap();

    println!("GPIO init OK");

    // Create ADC instances
    let analog = peripherals.SENS.split();
    // let vbat_pin = io.pins.gpio4; // ADC1_CH3

    let mut adc1_config = AdcConfig::new();
    let mut vbat_pin =
        adc1_config.enable_pin(io.pins.gpio4.into_analog(), Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();
    println!("ADC init OK");

    println!("init display");

    let sclk = io.pins.gpio47;
    let rst = io.pins.gpio17;
    let cs = io.pins.gpio6;

    let d0 = io.pins.gpio18;
    let d1 = io.pins.gpio7;
    let d2 = io.pins.gpio48;
    let d3 = io.pins.gpio5;

    let mut cs = cs.into_push_pull_output();
    cs.set_high().unwrap();

    let mut rst = rst.into_push_pull_output();

    let dma = Gdma::new(peripherals.DMA, &mut system.peripheral_clock_control);
    let dma_channel = dma.channel0;

    // Descriptors should be sized as (BUFFERSIZE / 4092) * 3
    // 8184 / 4092 * 3 = 6
    let mut descriptors = [0u32; 6];
    let spi = Spi::new_half_duplex(
        peripherals.SPI2, // use spi2 host
        Some(sclk),
        Some(d0),
        Some(d1),
        Some(d2),
        Some(d3),
        NO_PIN,       // Some(cs), NOTE: manually control cs
        85_u32.MHz(), // max 75MHz
        hal::spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .with_dma(dma_channel.configure(false, &mut descriptors, &mut [], DmaPriority::Priority0));

    let mut display = t_display_s3_amoled::rm67162dma::RM67162Dma::new(spi, cs);
    display.reset(&mut rst, &mut delay).unwrap();
    println!("reset display");
    display.init(&mut delay).unwrap();
    display
        .set_orientation(Orientation::LandscapeFlipped)
        .unwrap();

    display.clear(Rgb565::BLACK).unwrap();
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

    loop {
        let _ = led.toggle();

        let pin3_value: u16 = nb::block!(adc1.read(&mut vbat_pin)).unwrap();
        let pin3_voltage: f32 = pin3_value as f32 * 3.3 / 4095.0;
        println!("vbat ADC reading = {}", pin3_voltage);

        let mut s = String::new();
        core::write!(&mut s, "vbat ADC reading = {}\n", pin3_voltage).unwrap();

        Text::with_alignment(
            &s,
            Point::new(100, 20 + 40 * 2),
            MonoTextStyleBuilder::new()
                .background_color(Rgb565::BLACK)
                .text_color(Rgb565::YELLOW)
                .font(&FONT_10X20)
                .build(),
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();

        delay.delay_ms(1000_u32);
    }
}

fn now_ms() -> u64 {
    SystemTimer::now() * 1_000 / SystemTimer::TICKS_PER_SECOND
}
