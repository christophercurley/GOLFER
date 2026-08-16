use core::cell::RefCell;

use defmt::info;

use embassy_rp::{
    gpio::{Level, Output},
    peripherals::{
        PIN_13, PIN_14, PIN_16, PIN_17, PIN_18, PIN_19, PIN_21, SPI0,
    },
    spi::{Blocking, Config as SpiConfig, Spi},
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

// -----------------------------------------------------------------------------
// HARDWARE-SPECIFIC DISPLAY BACKEND
//
// This is the only file that should know:
//
//   * which physical TFT controller is attached
//   * which driver crate is used
//   * SPI frequency / bus construction
//   * reset/backlight pins
//   * module-specific orientation quirks
//
// When GOLFER moves from the prototype ILI9341 board to the final Newhaven
// display, this is the file we expect to replace/rework.
// -----------------------------------------------------------------------------

pub const TFT_SPI_FREQUENCY_HZ: u32 = 24_000_000;
const TFT_BUFFER_SIZE: usize = 512;

type TftSpi = Spi<'static, SPI0, Blocking>;
type TftSpiDevice =
    RefCellDevice<'static, TftSpi, Output<'static>, Delay>;
type TftInterface =
    SpiInterface<'static, TftSpiDevice, Output<'static>>;
pub type DrawTargetImpl =
    mipidsi::Display<TftInterface, ILI9341Rgb565, Output<'static>>;

static TFT_SPI_BUS: StaticCell<RefCell<TftSpi>> = StaticCell::new();
static TFT_BUFFER: StaticCell<[u8; TFT_BUFFER_SIZE]> = StaticCell::new();

pub struct Backend {
    driver: DrawTargetImpl,
    _backlight: Output<'static>,
}

impl Backend {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        spi0: Peri<'static, SPI0>,
        sck: Peri<'static, PIN_18>,
        mosi: Peri<'static, PIN_19>,
        miso: Peri<'static, PIN_16>,
        cs: Peri<'static, PIN_17>,
        dc: Peri<'static, PIN_13>,
        reset: Peri<'static, PIN_14>,
        backlight: Peri<'static, PIN_21>,
    ) -> Self {
        info!(
            "Initializing prototype ILI9341 TFT backend @ {} Hz",
            TFT_SPI_FREQUENCY_HZ
        );

        let mut spi_config = SpiConfig::default();
        spi_config.frequency = TFT_SPI_FREQUENCY_HZ;

        let spi = Spi::new_blocking(
            spi0,
            sck,
            mosi,
            miso,
            spi_config,
        );

        let bus = TFT_SPI_BUS.init(RefCell::new(spi));

        let cs = Output::new(cs, Level::High);
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
