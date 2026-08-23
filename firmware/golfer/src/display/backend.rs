use core::cell::RefCell;

use defmt::info;

use embassy_rp::{
    gpio::{Level, Output},
    peripherals::{PIN_13, PIN_14, PIN_21},
    Peri,
};
use embassy_time::Delay;

use embedded_hal_bus::spi::RefCellDevice;
use mipidsi::{
    interface::SpiInterface,
    models::ILI9341Rgb565,
    options::Orientation,
    Builder,
};
use static_cell::StaticCell;

use crate::spi0_bus::Spi0Bus;

// -----------------------------------------------------------------------------
// HARDWARE-SPECIFIC DISPLAY BACKEND
//
// This file knows:
//
//   * which physical TFT controller is attached
//   * which driver crate is used
//   * reset/backlight pins
//   * module-specific orientation quirks
//
// SPI0 bus construction is intentionally NOT owned here anymore. The TFT now
// shares SPI0 with the microSD card through embedded-hal-bus SpiDevice wrappers.
//
// When GOLFER moves from the prototype ILI9341 board to the final Newhaven
// display, this is the file we expect to replace/rework.
// -----------------------------------------------------------------------------

pub const TFT_SPI_FREQUENCY_HZ: u32 = 24_000_000;
const TFT_BUFFER_SIZE: usize = 512;

type TftSpiDevice =
    RefCellDevice<'static, Spi0Bus, Output<'static>, Delay>;
type TftInterface =
    SpiInterface<'static, TftSpiDevice, Output<'static>>;
pub type DrawTargetImpl =
    mipidsi::Display<TftInterface, ILI9341Rgb565, Output<'static>>;

static TFT_BUFFER: StaticCell<[u8; TFT_BUFFER_SIZE]> = StaticCell::new();

pub struct Backend {
    driver: DrawTargetImpl,
    _backlight: Output<'static>,
}

impl Backend {
    pub fn new(
        bus: &'static RefCell<Spi0Bus>,
        cs: Output<'static>,
        dc: Peri<'static, PIN_13>,
        reset: Peri<'static, PIN_14>,
        backlight: Peri<'static, PIN_21>,
    ) -> Self {
        info!(
            "Initializing prototype ILI9341 TFT backend @ {} Hz",
            TFT_SPI_FREQUENCY_HZ
        );

        let dc = Output::new(dc, Level::Low);
        let reset = Output::new(reset, Level::High);

        let mut backlight = Output::new(backlight, Level::Low);
        let mut delay = Delay;

        let spi_device =
            RefCellDevice::new(bus, cs, delay.clone()).unwrap();

        let buffer = TFT_BUFFER.init([0; TFT_BUFFER_SIZE]);

        let interface = SpiInterface::new(
            spi_device,
            dc,
            buffer,
        );

        let driver = Builder::new(
            ILI9341Rgb565,
            interface,
        )
        .reset_pin(reset)
        // Prototype MSP2402 quirk. This belongs here, NOT in UI code.
        .orientation(Orientation::new().flip_horizontal())
        .init(&mut delay)
        .unwrap();

        backlight.set_high();

        info!("Prototype TFT backend ready");

        Self {
            driver,
            _backlight: backlight,
        }
    }

    /// The UI sees only an embedded-graphics DrawTarget-compatible object.
    pub fn target(&mut self) -> &mut DrawTargetImpl {
        &mut self.driver
    }
}
