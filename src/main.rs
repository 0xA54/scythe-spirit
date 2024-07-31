#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use cortex_m::peripheral::SCB;
use cortex_m_rt::*;
use defmt::*;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice as SpiDeviceBlocking;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    rcc::{Hse, HseMode},
    spi::{self, Spi},
    time::Hertz,
    Config,
};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Timer};
use embedded_hal::spi::{Operation, SpiDevice};
use spirit1_rs::prelude::*;
use spirit1_rs::WORD;
use static_cell::StaticCell;
// use panic_probe as _;
use panic_halt as _;

#[exception]
unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
    SCB::sys_reset() // <- you could do something other than reset
}

#[derive(Debug)]
struct SpiritSpiHal<SPI> {
    base_frequency: u32,
    band_select: BandSelect,
    xtal_frequency: u32,
    spi: SPI,
}

// TODO: Move this back into `spirit1-rs` with the embedded-hal
impl<SPI> Spirit1HalBlocking for SpiritSpiHal<SPI>
where
    SPI: SpiDevice,
{
    fn delay_ms(&self, ms: u32) {
        trace!("delay {}", ms);
        embassy_time::block_for(Duration::from_millis(ms as u64));
    }

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
        R: Register<WORD> + ReadableRegister<WORD> + defmt::Format,
        [(); R::LENGTH]: Sized,
    {
        // TODO: Should return result not unwrap
        let mut buf = [0; R::LENGTH];
        let mut status = [0; 2];
        self.spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[spirit1_rs::READ, R::ADDRESS]),
                Operation::Read(&mut buf),
            ])
            .map_err(|_| RadioError::Spi)
            .unwrap();

        let state = McState::from_bytes(&status).unwrap();
        let ret = R::from_bytes(&buf).unwrap();
        // trace!("read_register[{:x}; {}] {:x}", R::ADDRESS, R::LENGTH, buf);
        trace!("READ@0x{:x}: {}", R::ADDRESS, ret);
        trace!("State = {}", state.state);

        ret
    }

    fn write_register<R>(&mut self, value: R) -> RadioResult<()>
    where
        R: WriteableRegister<WORD> + defmt::Format,
        [(); R::LENGTH]: Sized,
    {
        trace!("WRITE@0x{:x}: {}", R::ADDRESS, value);
        let mut status = [0; 2];

        let ret = self
            .spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[spirit1_rs::WRITE, R::ADDRESS]),
                Operation::Write(&value.into_bytes()?),
            ])
            .map_err(|_| RadioError::Spi);

        let state = McState::from_bytes(&status).unwrap();
        trace!("State = {}", state.state);

        ret
    }

    fn write_raw(&mut self, base: u8, value: &mut [u8]) -> RadioResult<()> {
        trace!("WRITE_RAW@0x{:x}: {}", base, value);
        let mut status = [0; 2];

        let ret = self
            .spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[spirit1_rs::WRITE, base]),
                Operation::Write(value),
            ])
            .map_err(|_| RadioError::Spi);

        let state = McState::from_bytes(&status).unwrap();
        trace!("State = {}", state.state);

        ret
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
    // Configure STM32
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Oscillator,
    });
    let p = embassy_stm32::init(config);

    // Configure GPIO
    let mut relay = Output::new(p.PA3, Level::Low, Speed::Low);
    // SPIRIT_G0 = PA1
    // SPIRIT_G1 = PA0 (not connected)

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz::mhz(1);

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

    // Configure radio
    let mut radio = SpiritSpiHal::new(spirit_spi, 433_400_000, 50_000_000).unwrap();

    // Perform a test read
    let info = radio.read_register::<DeviceInfo>();
    info!("{:x}", info);
    crate::assert_eq!(info.version, 0x30, "silicon version mismatch");

    // Initialize radio
    info!("Initializing radio...");
    if let Err(err) = configure_radio(&mut radio) {
        error!("Failed to initialize Spirit1: {}", err);
        loop {} // really we should panic
    }

    // dummy stuff
    info!("looping...");
    loop {
        relay.toggle();
        Timer::after_secs(3).await;
    }
}

/// Copy of the implementation @ [https://forum.digikey.com/t/getting-started-with-the-spirit1-transceiver/15624]
fn configure_radio(radio: &mut impl Spirit1) -> RadioResult<()> {
    // Restart the radio (can't do on this device)
    radio.management_wa_extra_current()?;
    radio.wait_for_ready()?;

    // Initialize the radio
    radio.init(RadioInitOpts {
        xtal_offset_ppm: 50,
        ..Default::default()
    })?;

    // Set the transmitter power level
    let pa_level_1 =
        PaPower1::from_dbm(0.0, radio.get_base_frequency()).expect("Invalid output power");
    let mut pa_config: PaPower = radio.read_register();
    pa_config.level_max_index = PaSlot::Slot1;
    radio.write_register(pa_level_1)?;
    radio.write_register(pa_config)?;

    // 🚧  Configure the packet format
    radio.configure_packet_protocol(PacketConfiguration::Basic(BasicProtocolOpts {
        preamble_length: PreambleLength::Bytes01,
        sync_length: PacketSyncLength::Bytes01,
        sync_words: (0, 0, 0, 0),
        fix_var_length: PacketLengthMode::Fixed,
        packet_length_width: 1,
        crc_mode: CrcMode::Crc0x07,
        control_length: PacketControlLength::Bytes01,
        address_field: true,
        fec: false,
        data_whitening: false,
    }))?;

    radio.configure_basic_filter(BasicAddressOpts {
        filter_on_my_address: false,
        my_address: 0,
        filter_on_multicast_address: false,
        multicast_address: 0,
        filter_on_broadcast_address: true,
        broadcast_address: 255,
    })?;

    // Configure interrupt pin
    // We have GPIO0 and GPIO1 wired up
    let gpio_0 = GpioConf::new(GpioMode::OutputLowPower(DigitalOutputMode::nIRQ));
    radio.write_register(Gpio0Conf(gpio_0))?;

    // Configure IRQ listener
    // TODO: Did I implement this?
    radio.irq_silence()?;
    let mut irq = IrqMaskBuilder::new();
    irq.set(InterruptEvent::TxDataSent)
        .set(InterruptEvent::RxDataReady)
        .set(InterruptEvent::RxDataDiscarded)
        .set(InterruptEvent::TimerRxTimeout);
    radio.write_register(IrqMask::from(irq))?;
    radio.irq_clear()?;

    // Receiver Quality Indicator Configuration
    // Enable the SQI threshold to 0 to require a perfect match between
    // the expected synchronization byte and the received synchronization byte
    let mut qi: QI = radio.read_register();
    qi.sqi_th = 0;
    qi.sqi_en = true;
    radio.write_register(qi)?;

    radio.set_rssi_threshold(-120)?;

    // Timer Configuration ❔
    radio.set_rx_timeout(2000.0)?;
    radio.set_rx_timeout_stop_condition(RxTimeoutStopCondition::SqiAboveThreshold)?;

    // Ok(radio)
    Ok(())
}
