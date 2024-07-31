#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use spirit1_rs::prelude::*;
use embedded_hal::spi::{Operation, SpiDevice};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{gpio::{Level, Output, Speed}, rcc::{Hse, HseMode}, spi, time::Hertz, Config};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[derive(Debug)]
struct SpiritSpiHal<SPI> {
    base_frequency: u32,
    band_select: BandSelect,
    xtal_frequency: u32,
    spi: SPI
}

type WORD = u8;

impl<SPI> Spirit1Hal for SpiritSpiHal<SPI> where SPI: SpiDevice {
    fn get_base_frequency(&self) -> u32 {
        self.base_frequency
    }

    fn get_frequency_band(&self) -> BandSelect {
        self.band_select.clone()
    }

    fn get_xtal_frequency(&self) -> u32 {
        self.xtal_frequency
    }

    fn read_register<R>(&mut self) -> R where R: Register<WORD> + ReadableRegister<WORD>, [(); R::LENGTH]: Sized, {
        // TODO: Should return result not unwrap
        let mut buf = [0; R::LENGTH];
        self.spi.transaction(&mut [
            Operation::Write(&[R::ADDRESS]),
            Operation::Read(&mut buf)
        ]).map_err(|_| RadioError::Spi).unwrap();
        
        R::from_bytes(&buf).unwrap()
    }

    fn write_raw(&mut self, base: u8, value: &mut [u8]) -> RadioResult<()> {
        self.spi.transaction(&mut [
            Operation::Write(&[base]),
            Operation::Write(value)
        ]).map_err(|_| RadioError::Spi)
    }

    fn write_register<R>(&mut self, value: R) -> RadioResult<()> where R: WriteableRegister<WORD>, [(); R::LENGTH]: Sized {
        self.spi.write(&value.into_bytes()?).map_err(|_| RadioError::Spi)
    }
}

impl<SPI> SpiritSpiHal<SPI> where SPI: SpiDevice {
    fn new(spi: SPI, base_frequency: u32, xtal_frequency: u32) -> Option<Self> {
        let band_select = BandSelect::from_hz(base_frequency)?;

        Some(Self {
            base_frequency,
            band_select,
            xtal_frequency,
            spi
        })
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // let spirit1 = SpiritSpiHal::new(433_400_000, 50_000_000).unwrap();
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Oscillator,
    });
    let mut spi_config = spi::Config::default();

    let p = embassy_stm32::init(config);
    let mut relay = Output::new(p.PA3, Level::Low, Speed::Low);
    
    // let spi = spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);
    // let spi = spi::Spi::new


    // let spirit1 = SpiritSpiHal::new(spi, 433_400_000, 50_000_000).unwrap();

    loop {
        info!("Hello");
        relay.toggle();
        Timer::after_secs(3).await;
    }
}

