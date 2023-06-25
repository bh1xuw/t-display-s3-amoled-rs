#![no_std]
#![no_main]

extern crate alloc;

use alloc::boxed::Box;
use alloc::rc::Rc;

use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::prelude::{DrawTarget, Point, Size};
use embedded_graphics::primitives::Rectangle;
use esp_backtrace as _;
use esp_println::println;
use hal::dma::{Rx, Tx};
use hal::systimer::SystemTimer;
use hal::{
    clock::ClockControl, dma::DmaPriority, gdma::Gdma, gpio::NO_PIN, peripherals::Peripherals,
    prelude::*, timer::TimerGroup, Delay, Rtc, Spi, IO,
};
use slint::platform::software_renderer::{MinimalSoftwareWindow, Rgb565Pixel};
use slint::platform::{software_renderer as renderer, Platform, WindowEvent};
use slint::PhysicalSize;

use t_display_s3_amoled::rm67162::dma::RM67162Dma;
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

slint::include_modules!();

struct Backend {
    window: Rc<renderer::MinimalSoftwareWindow>,
}

impl Platform for Backend {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() * 1_000 / SystemTimer::TICKS_PER_SECOND,
        )
    }

    // fn run_event_loop(&self) -> Result<(), slint::PlatformError>
    fn debug_log(&self, arguments: core::fmt::Arguments) {
        println!("Slint: {:?}", arguments);
    }
}

struct DisplayWrapper<'a, TX: Tx, RX: Rx, CS> {
    display: &'a mut RM67162Dma<'a, TX, RX, CS>,
    line_buffer: &'a mut [Rgb565Pixel; 536],
}

impl<TX: Tx, RX: Rx, CS> renderer::LineBufferProvider for &mut DisplayWrapper<'_, TX, RX, CS>
where
    CS: embedded_hal_1::digital::OutputPin,
{
    type TargetPixel = Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        render_fn(&mut self.line_buffer[range.clone()]);

        let _ = self.display.fill_contiguous(
            &Rectangle::new(
                Point::new(range.start as _, line as _),
                Size::new(range.len() as _, 1),
            ),
            self.line_buffer[range.clone()]
                .iter()
                .map(|p| RawU16::new(p.0).into()),
        );
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
    println!("Hello board!");

    // Set GPIO4 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio38.into_push_pull_output();
    let mut button = io.pins.gpio21.into_pull_down_input();

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

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
    let mut descriptors = [0u32; 12];
    let spi = Spi::new_half_duplex(
        peripherals.SPI2, // use spi2 host
        Some(sclk),
        Some(d0),
        Some(d1),
        Some(d2),
        Some(d3),
        NO_PIN,
        75_u32.MHz(), // max 75MHz
        hal::spi::SpiMode::Mode0,
        &mut system.peripheral_clock_control,
        &clocks,
    )
    .with_dma(dma_channel.configure(false, &mut descriptors, &mut [], DmaPriority::Priority0));

    let mut display = t_display_s3_amoled::rm67162::dma::RM67162Dma::new(spi, cs);
    display.reset(&mut rst, &mut delay).unwrap();
    display.init(&mut delay).unwrap();
    display
        .set_orientation(Orientation::LandscapeFlipped)
        .unwrap();

    println!("display init ok");

    let window = MinimalSoftwareWindow::new(renderer::RepaintBufferType::ReusedBuffer);
    slint::platform::set_platform(Box::new(Backend {
        window: window.clone(),
    }))
    .unwrap();
    window.set_size(PhysicalSize::new(536, 240));

    let ui = AppWindow::new().unwrap();
    let ui_handle = ui.as_weak();

    let mut line_buffer = [Rgb565Pixel(0); 536];
    let mut wrapper = DisplayWrapper {
        display: &mut display,
        line_buffer: &mut line_buffer,
    };

    let mut i = 0;
    loop {
        slint::platform::update_timers_and_animations();

        // Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut wrapper);
        });

        if !window.has_active_animations() {
            // if no animation is running, wait for the next input event
            ui.set_counter(i);
            i += 1;
            if i > 100 {
                i = 0;
            }
        }

        led.toggle();
    }
}
