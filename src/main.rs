// #![no_std]
#![feature(generic_const_exprs)]

use spirit1_rs::prelude::*;

#[derive(Debug)]
struct SpiritSpiHal {
    base_frequency: u32,
    band_select: BandSelect,
    xtal_frequency: u32
}

impl Spirit1Hal for SpiritSpiHal {
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
        R::reset_value()
    }

    fn write_raw(&mut self, base: u8, value: &mut [u8]) -> RadioResult<()> {
        Err(RadioError::NotImplemented)
    }

    fn write_register<R>(&mut self, value: R) -> RadioResult<()> where R: WriteableRegister<WORD>, [(); R::LENGTH]: Sized {
        Err(RadioError::NotImplemented)
    }
}

impl SpiritSpiHal {
    fn new(base_frequency: u32, xtal_frequency: u32) -> Option<Self> {
        let band_select = BandSelect::from_hz(base_frequency)?;

        Some(Self {
            base_frequency,
            band_select,
            xtal_frequency
        })
    }
}

fn main() {
    let spirit1 = SpiritSpiHal::new(433_400_000, 50_000_000).unwrap();

    println!("{:?}", spirit1);
}

