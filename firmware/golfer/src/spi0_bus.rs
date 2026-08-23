use core::cell::RefCell;

use defmt::info;

use embassy_rp::{
    peripherals::{PIN_16, PIN_18, PIN_19, SPI0},
    spi::{Blocking, Config as SpiConfig, Spi},
    Peri,
};
use static_cell::StaticCell;

// -----------------------------------------------------------------------------
// SHARED SPI0 BUS
//
// SPI0 is the common peripheral bus for:
//
//   * prototype TFT
//   * microSD card
//   * possible prototype touch controller later
//
// Each peripheral receives its own SpiDevice wrapper and chip-select line.
// The physical SPI bus itself is constructed exactly once here.
// -----------------------------------------------------------------------------

pub const SD_INIT_FREQUENCY_HZ: u32 = 400_000;
pub const RUN_FREQUENCY_HZ: u32 = 24_000_000;

pub type Spi0Bus = Spi<'static, SPI0, Blocking>;

static SPI0_BUS: StaticCell<RefCell<Spi0Bus>> = StaticCell::new();

pub fn init(
    spi0: Peri<'static, SPI0>,
    sck: Peri<'static, PIN_18>,
    mosi: Peri<'static, PIN_19>,
    miso: Peri<'static, PIN_16>,
) -> &'static RefCell<Spi0Bus> {
    let mut config = SpiConfig::default();
    config.frequency = SD_INIT_FREQUENCY_HZ;

    let spi = Spi::new_blocking(
        spi0,
        sck,
        mosi,
        miso,
        config,
    );

    info!(
        "SPI0 shared bus initialized @ {} Hz",
        SD_INIT_FREQUENCY_HZ
    );

    SPI0_BUS.init(RefCell::new(spi))
}

pub fn set_frequency(
    bus: &'static RefCell<Spi0Bus>,
    frequency_hz: u32,
) {
    bus.borrow_mut().set_frequency(frequency_hz);

    info!(
        "SPI0 shared bus frequency set to {} Hz",
        frequency_hz
    );
}

/// SD cards require at least 74 clocks with CS deasserted before entering
/// SPI mode. Ten 0xFF bytes provide 80 clocks while MOSI remains high.
pub fn send_sd_startup_clocks(
    bus: &'static RefCell<Spi0Bus>,
) -> bool {
    let clocks = [0xFFu8; 10];

    bus.borrow_mut()
        .blocking_write(&clocks)
        .is_ok()
}
