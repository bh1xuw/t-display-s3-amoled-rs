#![no_std]
#![no_main]

extern crate alloc;

use alloc::string::String;
use core::fmt::Write;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::raw::ToBytes;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::{Alignment, Text};
use embedded_graphics::Pixel;
use esp_backtrace as _;
use esp_println::println;
use hal::gpio::NO_PIN;
use hal::peripherals::SPI2;
use hal::prelude::_fugit_RateExtU32;
use hal::spi::{Address, Command, HalfDuplexReadWrite, SpiDataMode};
use hal::{
    adc::{AdcConfig, Attenuation, ADC, ADC1},
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    spi::HalfDuplexMode,
    timer::TimerGroup,
    Delay, Rtc, Spi, IO,
};
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

pub struct RM67162<'a, T, CS> {
    spi: Spi<'a, T, HalfDuplexMode>,
    cs: CS,
}

impl<CS> RM67162<'_, SPI2, CS>
where
    CS: eh1::_embedded_hal_digital_blocking_OutputPin,
{
    pub fn new<'a>(spi: Spi<'a, SPI2, HalfDuplexMode>, cs: CS) -> RM67162<'a, SPI2, CS> {
        RM67162 { spi, cs } // Cannot use Self here...... YOU Rust suck
    }

    pub fn reset(
        &self,
        rst: &mut impl embedded_hal_1::digital::OutputPin,
        delay: &mut impl embedded_hal_1::delay::DelayUs,
    ) -> Result<(), ()> {
        rst.set_low().unwrap();
        delay.delay_ms(300);

        rst.set_high().unwrap();
        delay.delay_ms(200);
        Ok(())
    }

    fn send_cmd(&mut self, cmd: u32, data: &[u8]) -> Result<(), ()> {
        self.cs.set_low().unwrap();
        self.spi
            .write(
                SpiDataMode::Single,
                Command::Command8(0x02, SpiDataMode::Single),
                Address::Address24(cmd << 8, SpiDataMode::Single),
                0,
                data,
            )
            .unwrap();
        self.cs.set_high().unwrap();
        Ok(())
    }

    // rm67162_qspi_init
    pub fn init(&mut self, delay: &mut impl embedded_hal_1::delay::DelayUs) -> Result<(), ()> {
        for _ in 0..3 {
            self.send_cmd(0x11, &[])?; // sleep out
            delay.delay_ms(120);

            self.send_cmd(0x3A, &[0x55])?; // 16bit mode

            self.send_cmd(0x51, &[0x00])?; // write brightness

            //self.send_cmd(0x23, &[])?; // debug: all pixel on

            self.send_cmd(0x29, &[])?; // display on
            delay.delay_ms(120);

            self.send_cmd(0x51, &[0xD0])?; // write brightness
        }
        Ok(())
    }

    pub fn set_address(&mut self, x1: u16, y1: u16, x2: u16, y2: u16) -> Result<(), ()> {
        self.send_cmd(
            0x2a,
            &[
                (x1 >> 8) as u8,
                (x1 & 0xFF) as u8,
                (x2 >> 8) as u8,
                (x2 & 0xFF) as u8,
            ],
        )?;
        self.send_cmd(
            0x2b,
            &[
                (y1 >> 8) as u8,
                (y1 & 0xFF) as u8,
                (y2 >> 8) as u8,
                (y2 & 0xFF) as u8,
            ],
        )?;
        self.send_cmd(0x2c, &[])?;
        Ok(())
    }

    fn draw_raw_point(&mut self, x: u16, y: u16, color: [u8; 2]) -> Result<(), ()> {
        println!("draw raw point: {} {} {:x?}", x, y, color);
        self.set_address(x, y, x, y)?;
        self.cs.set_low().unwrap();
        self.spi
            .write(
                SpiDataMode::Quad,
                Command::Command8(0x32, SpiDataMode::Single),
                Address::Address24(0x2C << 8, SpiDataMode::Single),
                0,
                &color[..],
            )
            .unwrap();
        self.cs.set_high().unwrap();
        Ok(())
    }

    fn fill_color(&mut self, x: u16, y: u16, w: u16, h: u16, color: [u8; 2]) -> Result<(), ()> {
        println!("fill color: {} {} {} {} {:x?}", x, y, w, h, color);
        self.set_address(x, y, x + w - 1, y + h - 1)?;
        self.cs.set_low().unwrap();
        self.spi
            .write(
                SpiDataMode::Quad,
                Command::Command8(0x32, SpiDataMode::Single),
                Address::Address24(0x2C << 8, SpiDataMode::Single),
                0,
                &color[..],
            )
            .unwrap();

        for _ in 1..((w as u32) * (h as u32)) {
            self.spi
                .write(
                    SpiDataMode::Quad,
                    Command::None,
                    Address::None,
                    0,
                    &color[..],
                )
                .unwrap();
        }
        self.cs.set_high().unwrap();
        Ok(())
    }
}

impl<CS> OriginDimensions for RM67162<'_, SPI2, CS>
where
    CS: eh1::_embedded_hal_digital_blocking_OutputPin,
{
    fn size(&self) -> Size {
        Size::new(240, 536)
    }
}

impl<CS> DrawTarget for RM67162<'_, SPI2, CS>
where
    CS: eh1::_embedded_hal_digital_blocking_OutputPin,
{
    type Color = Rgb565;

    type Error = ();

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = embedded_graphics::Pixel<Self::Color>>,
    {
        for Pixel(pt, color) in pixels {
            if pt.x < 0 || pt.y < 0 {
                continue;
            }

            self.draw_raw_point(pt.x as u16, pt.y as u16, color.to_be_bytes())?;
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_color(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
            color.to_be_bytes(),
        )?;
        Ok(())
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
    println!("GPIO init OK");

    // Create ADC instances
    let analog = peripherals.SENS.split();
    // let vbat_pin = io.pins.gpio4; // ADC1_CH3

    let mut adc1_config = AdcConfig::new();
    let mut vbat_pin =
        adc1_config.enable_pin(io.pins.gpio4.into_analog(), Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::adc(analog.adc1, adc1_config).unwrap();

    println!("ADC init OK");

    let sclk = io.pins.gpio47;
    let rst = io.pins.gpio17;
    let cs = io.pins.gpio6;

    let d0 = io.pins.gpio18;
    let d1 = io.pins.gpio7;
    let d2 = io.pins.gpio48;
    let d3 = io.pins.gpio5;

    let mut rst = rst.into_push_pull_output();

    led.set_high().unwrap();

    let spi = Spi::new_half_duplex(
        peripherals.SPI2, // use spi2 host
        Some(sclk),
        Some(d0),
        Some(d1),
        Some(d2),
        Some(d3),
        NO_PIN,       // Some(cs),
        80_u32.MHz(), // max 75MHz
        hal::spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    /* .with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));*/

    let mut cs = cs.into_push_pull_output();
    cs.set_high().unwrap();

    let mut display = RM67162::new(spi, cs);
    display.reset(&mut rst, &mut delay).unwrap();
    println!("reset display");
    display.init(&mut delay).unwrap();
    println!("init display");

    display.clear(Rgb565::WHITE).unwrap();
    println!("screen init ok");

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb565::RED);
    Text::with_alignment(
        "Hello,\nRust World!",
        Point::new(100, 20 + 40 * 5),
        character_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();

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
