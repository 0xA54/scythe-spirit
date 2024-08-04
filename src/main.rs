#![no_std]
#![no_main]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use cortex_m::peripheral::SCB;
use cortex_m_rt::*;
use defmt::{error, info, trace};
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

// use panic_halt as _;
// use panic_probe as _;

// #[exception]
// unsafe fn HardFault(_frame: &ExceptionFrame) -> ! {
//     error!("Yeah I died");
//     SCB::sys_reset() // <- you could do something other than reset
// }

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

#[derive(Debug)]
struct SpiritSpiHal<SPI, CONFIG> {
    base_frequency: u32,
    band_select: BandSelect,
    // xtal_frequency: u32,
    spi: SPI,
    config: CONFIG,
}

// TODO: Move this back into `spirit1-rs` with the embedded-hal
impl<SPI, CONFIG> Spirit1HalBlocking for SpiritSpiHal<SPI, CONFIG>
where
    SPI: SpiDevice,
    CONFIG: SpiritHardwareConst,
{
    fn delay_ms(&self, ms: u32) {
        embassy_time::block_for(Duration::from_millis(ms as u64));
    }

    fn get_base_frequency(&self) -> u32 {
        self.base_frequency
    }

    fn get_frequency_band(&self) -> BandSelect {
        self.band_select.clone()
    }

    fn get_xtal_frequency(&self) -> u32 {
        // Self::XTAL_FREQUENCY
        // self.xtal_frequency
        50_000_000
    }

    fn read_register<R>(&mut self) -> RadioResult<R>
    where
        R: Register<WORD> + ReadableRegister<WORD> + defmt::Format,
        [(); R::LENGTH]: Sized,
    {
        // TODO: Should return result not unwrap
        let mut buf = [0; R::LENGTH];
        let mut status = [0; 2];
        let r = self
            .spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[spirit1_rs::READ, R::ADDRESS]),
                Operation::Read(&mut buf),
            ])
            .map_err(|_| RadioError::Spi);

        if let Err(_) = r {
            error!("spi_error");
        }

        let ret = if let Ok(ret) = R::from_bytes(&buf) {
            Ok(ret)
        } else {
            error!("Failed to decode");
            Err(RadioError::ParameterError)
        };

        match McState::from_bytes(&status) {
            Ok(state) => {
                trace!("READ@0x{:x}: {}", R::ADDRESS, ret);
                trace!("State = {}", state.state);
            }
            Err(_) => {
                error!("Failed to decode McState: {}", status);
            }
        }

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

        match McState::from_bytes(&status) {
            Ok(state) => {
                trace!("State = {}", state.state);
            }
            Err(_) => {
                error!("Failed to decode McState");
            }
        }

        self.delay_ms(1);

        ret
    }

    fn write_raw(&mut self, base: u8, value: &[u8]) -> RadioResult<()> {
        trace!("WRITE_RAW@0x{:x}: {}", base, value);
        let mut status = [0; 2];

        let ret = self
            .spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[spirit1_rs::WRITE, base]),
                Operation::Write(value),
            ])
            .map_err(|_| RadioError::Spi);

        match McState::from_bytes(&status) {
            Ok(state) => {
                trace!("State = {}", state.state);
            }
            Err(_) => {
                error!("Failed to decode McState");
            }
        }

        ret
    }

    fn read_raw(&mut self, address: u8, length: usize, buffer: &mut [u8]) -> RadioResult<()> {
        trace!("READ_RAW@0x{:x}: len={}", address, length);
        let mut status = [0; 2];

        let ret = self
            .spi
            .transaction(&mut [
                Operation::Transfer(&mut status, &[spirit1_rs::WRITE, address]),
                Operation::Read(&mut buffer[..length]),
            ])
            .map_err(|_| RadioError::Spi);
        trace!("Buffer = {}", buffer[..length]);

        match McState::from_bytes(&status) {
            Ok(state) => {
                trace!("State = {}", state.state);
            }
            Err(_) => {
                error!("Failed to decode McState");
            }
        }

        ret
    }

    fn write_command(&mut self, command: SpiritCommand) -> RadioResult<McState> {
        trace!("WRITE_CMD: {}", command);
        let mut status = [0; 2];

        let ret = if let Err(e) = self
            .spi
            .transaction(&mut [Operation::Transfer(
                &mut status,
                &[spirit1_rs::COMMAND, command.try_into()?],
            )])
            .map_err(|_| RadioError::Spi)
        {
            Err(e)
        } else {
            let a = McState::from_bytes(&status);
            match a {
                Ok(state) => {
                    trace!("State = {}", state.state);
                    Ok(state)
                }
                Err(_) => {
                    error!("Failed to decode McState");
                    Err(RadioError::ParameterError)
                }
            }
        };

        self.delay_ms(100);

        ret
    }
}

impl<SPI, CONFIG> SpiritSpiHal<SPI, CONFIG>
where
    SPI: SpiDevice,
    CONFIG: SpiritHardwareConst,
{
    fn new(spi: SPI, base_frequency: u32, config: CONFIG) -> Option<Self> {
        let band_select = BandSelect::from_hz(base_frequency)?;

        Some(Self {
            base_frequency,
            band_select,
            config,
            spi,
        })
    }
}

trait SpiritHardwareConst {
    const XTAL_FREQUENCY: u32;
}

struct HardwareOpts();
impl SpiritHardwareConst for HardwareOpts {
    const XTAL_FREQUENCY: u32 = 50_000_000;
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Configure STM32
    let mut config = Config::default();
    config.rcc.hse = Some(Hse {
        freq: Hertz::mhz(8),
        mode: HseMode::Oscillator,
    });
    config.enable_debug_during_sleep = true;
    let p = embassy_stm32::init(config);

    // let mut buffer = [0u8; 96];

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
    let mut radio = SpiritSpiHal::new(spirit_spi, 433_400_000, HardwareOpts()).unwrap();

    // Perform a test read
    let _ = radio.write_command(SpiritCommand::S_RES);
    radio.delay_ms(50);
    let info = radio.read_register::<DeviceInfo>().unwrap(); // ahahaha
    info!("{:x}", info);
    assert_eq!(info.version, 0x30, "silicon version mismatch");

    // Initialize radio
    info!("Initializing radio...");
    if let Err(err) = configure_radio(&mut radio) {
        error!("Failed to initialize Spirit1: {}", err);
        // panic!()
        loop {}
    }
    info!("Radio initialized!");

    // // Debug
    // info!("Looping...");
    // loop {
    //     relay.toggle();
    //     Timer::after_secs(3).await;
    // }

    // Transmitter
    let mut counter: u8 = 0;
    loop {
        info!("Transmitting: 0xDE 0xAD 0x{:x}", counter);
        match radio.tx_blocking(&[0xDE, 0xAD, counter]) {
            Ok(e) => {
                debug!("Transmitted {} bytes", e)
            }
            Err(e) => {
                error!("TX Error: {}", e)
            }
        }

        counter = counter.wrapping_add(1);

        relay.toggle();
        // Timer::after_secs(3).await;
        radio.delay_ms(3000);
    }

    // // Receiver
    // loop {
    //     info!("Receiving...");
    //     match radio.rx_blocking(&mut buffer) {
    //         Ok(e) => {
    //             debug!("Received {} bytes", e);
    //             info!("Data: {:x}", buffer[..e]);
    //             relay.toggle();
    //         }
    //         Err(e) => {
    //             error!("RX Error: {}", e)
    //         }
    //     }

    //     Timer::after_secs(1).await;
    // }
}

/// Copy of the implementation @ [https://forum.digikey.com/t/getting-started-with-the-spirit1-transceiver/15624]
fn configure_radio(radio: &mut impl Spirit1HalBlocking) -> RadioResult<()> {
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
    let mut pa_config: PaPower = radio.read_register()?;
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
    let mut qi: QI = radio.read_register()?;
    qi.sqi_th = 0;
    qi.sqi_en = true;
    radio.write_register(qi)?;

    radio.set_rssi_threshold(-120)?;

    // Timer Configuration ❔
    const RX_TIMEOUT: (u8, u8) =
        spirit1_rs::calculate_rx_timeout(2000, HardwareOpts::XTAL_FREQUENCY);
    radio.set_rx_timeout(RX_TIMEOUT.0, RX_TIMEOUT.1)?;
    radio.set_rx_timeout_stop_condition(RxTimeoutStopCondition::SqiAboveThreshold)?;

    trace!("Radio ready");
    Ok(())
}
