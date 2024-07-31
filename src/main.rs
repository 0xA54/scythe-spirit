#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use core::cell::RefCell;

use defmt::*;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as SpiDeviceBlocking;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    rcc::{Hse, HseMode},
    spi::{self, Spi},
    time::Hertz,
    Config,
};
use embassy_sync::{
    blocking_mutex::{raw::NoopRawMutex, NoopMutex},
    mutex::Mutex,
};
use embassy_time::Timer;
use embedded_hal::spi::{Operation, SpiDevice};
use spirit1_rs::prelude::*;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[derive(Debug)]
struct SpiritSpiHal<SPI> {
    base_frequency: u32,
    band_select: BandSelect,
    xtal_frequency: u32,
    spi: SPI,
}

type WORD = u8;

impl<SPI> Spirit1HalBlocking for SpiritSpiHal<SPI>
where
    SPI: SpiDevice,
{
    fn get_base_frequency(&self) -> u32 {
        self.base_frequency
    }

    fn get_frequency_band(&self) -> BandSelect {
        self.band_select.clone()
    }

    fn get_xtal_frequency(&self) -> u32 {
        self.xtal_frequency
    }

    fn read_register<R>(&mut self) -> R
    where
        R: Register<WORD> + ReadableRegister<WORD>,
        [(); R::LENGTH]: Sized,
    {
        // TODO: Should return result not unwrap
        let mut buf = [0; R::LENGTH];
        let mut status = [0; 2];
        self.spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[1, R::ADDRESS]),
                // Operation::Write(&[1, R::ADDRESS]),
                Operation::Read(&mut buf),
            ])
            .map_err(|_| RadioError::Spi)
            .unwrap();

        trace!("read_register[{:x}; {}] {:x}", R::ADDRESS, R::LENGTH, buf);
        trace!("status: {:b}", status); // MC_STATE

        R::from_bytes(&buf).unwrap()
    }

    fn write_raw(&mut self, base: u8, value: &mut [u8]) -> RadioResult<()> {
        self.spi
            .transaction(&mut [Operation::Write(&[0, base]), Operation::Write(value)])
            .map_err(|_| RadioError::Spi)
    }

    fn write_register<R>(&mut self, value: R) -> RadioResult<()>
    where
        R: WriteableRegister<WORD>,
        [(); R::LENGTH]: Sized,
    {
        self.spi
            .write(&value.into_bytes()?)
            .map_err(|_| RadioError::Spi)
    }
}

impl<SPI> SpiritSpiHal<SPI>
where
    SPI: SpiDevice,
{
    fn new(spi: SPI, base_frequency: u32, xtal_frequency: u32) -> Option<Self> {
        let band_select = BandSelect::from_hz(base_frequency)?;

        Some(Self {
            base_frequency,
            band_select,
            xtal_frequency,
            spi,
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
    spi_config.frequency = Hertz::mhz(1);

    let p = embassy_stm32::init(config);
    let mut relay = Output::new(p.PA3, Level::Low, Speed::Low);
    // SPIRIT_G0 = PA1
    // SPIRIT_G1 = PA0 (not connected)

    // async - have not implemented Spirit1HalAsync yet
    // static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<embassy_stm32::mode::Async>>> =
    //     StaticCell::new();
    // let spi = spi::Spi::new(
    //     p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, spi_config,
    // );

    // blocking - Use Spirit1HalBlocking
    static SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<embassy_stm32::mode::Blocking>>>> =
        StaticCell::new();
    let spi = spi::Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);
    let spi_cs = Output::new(p.PA2, Level::Low, Speed::VeryHigh);

    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = SPI_BUS.init(spi_bus);
    let spirit_spi = SpiDeviceBlocking::new(spi_bus, spi_cs);

    let mut spirit1 = SpiritSpiHal::new(spirit_spi, 433_400_000, 50_000_000).unwrap();
    let info = spirit1.read_register::<DeviceInfo>();
    info!("Part {:x}, Version {:x}", info.part_number, info.version);

    // info!("Trying to init...");
    // if let Err(init_err) = spirit1.init(RadioInitOpts::default()) {
    //     error!("Failed to initialize Spirit1");
    //     loop {
    //         relay.toggle();
    //         Timer::after_secs(3).await;
    //     }
    // }

    info!("looping...");
    loop {
        relay.toggle();
        Timer::after_secs(3).await;
    }
}
