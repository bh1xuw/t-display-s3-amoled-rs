//! On-board RM67162 AMOLED screen driver

use core::{iter, ops};

use embedded_graphics::{
    pixelcolor::{raw::ToBytes, Rgb565},
    prelude::{DrawTarget, OriginDimensions, Size},
    primitives::Rectangle,
    Pixel,
};
use embedded_hal_1::{delay::DelayUs, digital::OutputPin};
use hal::{
    dma::{Rx, Tx},
    peripherals::SPI2,
    prelude::_esp_hal_dma_DmaTransfer,
    spi::{dma::SpiDma, Address, Command, HalfDuplexMode, HalfDuplexReadWrite, SpiDataMode},
    Spi,
};
use hal::{gdma::SuitablePeripheral0, prelude::_embedded_dma_ReadBuffer};

use crate::rm67162::Orientation;

static mut DMA_BUFFER: [u8; 4096] = [0u8; 4096];

pub struct RM67162Dma<'a, TX: Tx, RX: Rx, CS> {
    spi: Option<SpiDma<'a, SPI2, TX, RX, SuitablePeripheral0, HalfDuplexMode>>,
    cs: CS,
    orientation: Orientation,
}

impl<TX, RX, CS> RM67162Dma<'_, TX, RX, CS>
where
    CS: OutputPin,
    TX: Tx,
    RX: Rx,
{
    pub fn new<'a>(
        spi: SpiDma<'a, SPI2, TX, RX, SuitablePeripheral0, HalfDuplexMode>,
        cs: CS,
    ) -> RM67162Dma<'a, TX, RX, CS> {
        RM67162Dma {
            spi: Some(spi),
            cs,
            orientation: Orientation::Portrait,
        }
    }

    pub fn set_orientation(&mut self, orientation: Orientation) -> Result<(), ()> {
        self.orientation = orientation;

        self.send_cmd(0x36, &[self.orientation.to_madctr()])
    }

    pub fn reset(&self, rst: &mut impl OutputPin, delay: &mut impl DelayUs) -> Result<(), ()> {
        rst.set_low().unwrap();
        delay.delay_ms(300);

        rst.set_high().unwrap();
        delay.delay_ms(200);
        Ok(())
    }

    fn send_cmd(&mut self, cmd: u32, data: &[u8]) -> Result<(), ()> {
        unsafe {
            DMA_BUFFER[..data.len()].copy_from_slice(data);
        }
        let txbuf = StaticReadBuffer::new(unsafe { &DMA_BUFFER[..] }, data.len());

        self.cs.set_low().unwrap();

        let mut spi = self.spi.take().unwrap();
        let tx = spi
            .write(
                SpiDataMode::Single,
                Command::Command8(0x02, SpiDataMode::Single),
                Address::Address24(cmd << 8, SpiDataMode::Single),
                0,
                txbuf,
            )
            .unwrap();
        (_, spi) = tx.wait();
        self.spi.replace(spi);

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

            self.send_cmd(0x29, &[])?; // display on
            delay.delay_ms(120);

            self.send_cmd(0x51, &[0xD0])?; // write brightness
        }

        self.set_orientation(self.orientation)?;
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

    fn draw_point(&mut self, x: u16, y: u16, color: Rgb565) -> Result<(), ()> {
        self.set_address(x, y, x, y)?;

        unsafe {
            DMA_BUFFER[..2].copy_from_slice(&color.to_be_bytes()[..]);
        }
        let txbuf = StaticReadBuffer::new(unsafe { &DMA_BUFFER[..] }, 2);

        self.cs.set_low().unwrap();

        let mut spi = self.spi.take().unwrap();
        let tx = spi
            .write(
                SpiDataMode::Quad,
                Command::Command8(0x32, SpiDataMode::Single),
                Address::Address24(0x2C << 8, SpiDataMode::Single),
                0,
                txbuf,
            )
            .unwrap();
        (_, spi) = tx.wait();
        self.spi.replace(spi);

        self.cs.set_high().unwrap();
        Ok(())
    }

    #[inline]
    fn dma_send_colors(&mut self, txbuf: StaticReadBuffer<'_>, first_send: bool) -> Result<(), ()> {
        let mut spi = self.spi.take().unwrap();

        let tx = if first_send {
            spi.write(
                SpiDataMode::Quad,
                Command::Command8(0x32, SpiDataMode::Single),
                Address::Address24(0x2C << 8, SpiDataMode::Single),
                0,
                txbuf,
            )
            .unwrap()
        } else {
            spi.write(SpiDataMode::Quad, Command::None, Address::None, 0, txbuf)
                .unwrap()
        };
        (_, spi) = tx.wait();
        self.spi.replace(spi);
        Ok(())
    }

    pub fn fill_colors(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        colors: impl Iterator<Item = Rgb565>,
    ) -> Result<(), ()> {
        self.set_address(x, y, x + w - 1, y + h - 1)?;

        let mut first_send = true;
        self.cs.set_low().unwrap();

        let mut i = 0;

        for color in colors.into_iter().take(w as usize * h as usize) {
            if i == 2048 {
                let txbuf = StaticReadBuffer::new(unsafe { &DMA_BUFFER[..] }, 4096);

                self.dma_send_colors(txbuf, first_send)?;
                first_send = false;

                i = 0;
            }
            unsafe {
                DMA_BUFFER[2 * i..2 * i + 2].copy_from_slice(&color.to_be_bytes()[..]);
            }
            i += 1;
        }
        if i > 0 {
            let txbuf = StaticReadBuffer::new(unsafe { &DMA_BUFFER[..] }, 2 * i);
            self.dma_send_colors(txbuf, first_send)?;
        }

        self.cs.set_high().unwrap();
        Ok(())
    }

    fn fill_color(&mut self, x: u16, y: u16, w: u16, h: u16, color: Rgb565) -> Result<(), ()> {
        self.fill_colors(x, y, w, h, iter::repeat(color))?;
        Ok(())
    }
}

impl<TX, RX, CS> OriginDimensions for RM67162Dma<'_, TX, RX, CS>
where
    TX: Tx,
    RX: Rx,
    CS: OutputPin,
{
    fn size(&self) -> Size {
        if matches!(
            self.orientation,
            Orientation::Landscape | Orientation::LandscapeFlipped
        ) {
            Size::new(536, 240)
        } else {
            Size::new(240, 536)
        }
    }
}

impl<TX, RX, CS> DrawTarget for RM67162Dma<'_, TX, RX, CS>
where
    TX: Tx,
    RX: Rx,
    CS: OutputPin,
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
            self.draw_point(pt.x as u16, pt.y as u16, color)?;
        }
        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_color(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
            color,
        )?;
        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.fill_colors(
            area.top_left.x as u16,
            area.top_left.y as u16,
            area.size.width as u16,
            area.size.height as u16,
            colors.into_iter(),
        )?;
        Ok(())
    }
}

#[derive(Copy, Clone)]
pub struct StaticReadBuffer<'a> {
    buffer: &'a [u8],
    len: usize,
}

impl StaticReadBuffer<'_> {
    pub fn new(buffer: &[u8], len: usize) -> StaticReadBuffer {
        StaticReadBuffer { buffer, len }
    }
}

unsafe impl hal::prelude::_embedded_dma_ReadBuffer for StaticReadBuffer<'_> {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.buffer.as_ptr(), self.len)
    }
}
