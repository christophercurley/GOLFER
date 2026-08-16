use core::{
    cell::RefCell,
    fmt::Write as _,
};

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

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
    text::{Baseline, Text},
};

use embedded_hal_bus::spi::RefCellDevice;
use heapless::String;
use mipidsi::{
    interface::SpiInterface,
    models::ILI9341Rgb565,
    options::Orientation,
    Builder,
};
use static_cell::StaticCell;

// -----------------------------------------------------------------------------
// TFT hardware
//
// GP16 = SPI0 MISO
// GP17 = TFT CS
// GP18 = SPI0 SCK
// GP19 = SPI0 MOSI
// GP13 = TFT DC/RS
// GP14 = TFT RESET
// GP21 = TFT backlight enable
//
// Rendering strategy:
//   - clear the whole screen ONCE at startup
//   - draw all static labels ONCE
//   - each dynamic field owns a small fixed rectangle
//   - only fields whose values actually changed are erased/redrawn
//
// This is still intentionally an ugly functional layout. UI design comes next.
// -----------------------------------------------------------------------------

const TFT_SPI_FREQUENCY_HZ: u32 = 24_000_000;
const TFT_BUFFER_SIZE: usize = 512;

// Dynamic value column.
const VALUE_X: i32 = 74;
const VALUE_W: u32 = 162;
const VALUE_H: u32 = 12;

// Radio field rows.
const LINK_Y: i32 = 42;
const SEQ_Y: i32 = 58;
const RSSI_Y: i32 = 74;
const SNR_Y: i32 = 90;
const RX_Y: i32 = 106;
const MISSED_Y: i32 = 122;

// GPS field rows.
const GPS_Y: i32 = 166;
const SAT_Y: i32 = 182;
const LAT_Y: i32 = 198;
const LON_Y: i32 = 214;

type TftSpi = Spi<'static, SPI0, Blocking>;
type TftSpiDevice =
    RefCellDevice<'static, TftSpi, Output<'static>, Delay>;
type TftInterface =
    SpiInterface<'static, TftSpiDevice, Output<'static>>;
type TftDriver =
    mipidsi::Display<TftInterface, ILI9341Rgb565, Output<'static>>;

static TFT_SPI_BUS: StaticCell<RefCell<TftSpi>> = StaticCell::new();
static TFT_BUFFER: StaticCell<[u8; TFT_BUFFER_SIZE]> = StaticCell::new();

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DisplayPage {
    General,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RadioLinkState {
    Waiting,
    Connected,
    Lost,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct RadioDisplayState {
    pub link: RadioLinkState,
    pub sequence: Option<u32>,
    pub rssi: Option<i16>,
    pub snr: Option<i16>,
    pub received: u32,
    pub missed: u32,
}

impl RadioDisplayState {
    pub const fn waiting() -> Self {
        Self {
            link: RadioLinkState::Waiting,
            sequence: None,
            rssi: None,
            snr: None,
            received: 0,
            missed: 0,
        }
    }

    pub const fn connected(
        sequence: u32,
        rssi: i16,
        snr: i16,
        received: u32,
        missed: u32,
    ) -> Self {
        Self {
            link: RadioLinkState::Connected,
            sequence: Some(sequence),
            rssi: Some(rssi),
            snr: Some(snr),
            received,
            missed,
        }
    }

    pub const fn lost(
        sequence: Option<u32>,
        rssi: Option<i16>,
        snr: Option<i16>,
        received: u32,
        missed: u32,
    ) -> Self {
        Self {
            link: RadioLinkState::Lost,
            sequence,
            rssi,
            snr,
            received,
            missed,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct GpsDisplayState {
    pub online: bool,
    pub fix: bool,
    pub latitude_e7: Option<i32>,
    pub longitude_e7: Option<i32>,
    pub satellites: Option<u8>,
}

impl GpsDisplayState {
    pub const fn offline() -> Self {
        Self {
            online: false,
            fix: false,
            latitude_e7: None,
            longitude_e7: None,
            satellites: None,
        }
    }
}

pub struct Display {
    driver: TftDriver,
    _backlight: Output<'static>,
    page: DisplayPage,
    radio: RadioDisplayState,
    gps: GpsDisplayState,
}

impl Display {
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
        info!("Initializing ILI9341 TFT...");

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

        let mut driver = Builder::new(
            ILI9341Rgb565,
            interface,
        )
        .reset_pin(reset)
        .orientation(Orientation::new().flip_horizontal())
        .init(&mut delay)
        .unwrap();

        // One and only routine full-screen clear.
        driver.clear(Rgb565::BLACK).unwrap();

        backlight.set_high();

        let mut display = Self {
            driver,
            _backlight: backlight,
            page: DisplayPage::General,
            radio: RadioDisplayState::waiting(),
            gps: GpsDisplayState::offline(),
        };

        display.draw_static();
        display.redraw_all_dynamic();

        info!(
            "ILI9341 TFT initialized: portrait, per-field redraw, SPI={} Hz",
            TFT_SPI_FREQUENCY_HZ
        );

        display
    }

    pub fn page(&self) -> DisplayPage {
        self.page
    }

    pub fn set_page(&mut self, page: DisplayPage) {
        if self.page == page {
            return;
        }

        self.page = page;

        // Full redraw is fine for an infrequent page transition.
        self.driver.clear(Rgb565::BLACK).unwrap();
        self.draw_static();
        self.redraw_all_dynamic();
    }

    pub fn update_radio(&mut self, state: RadioDisplayState) {
        let old = self.radio;
        self.radio = state;

        if self.page != DisplayPage::General {
            return;
        }

        if old.link != state.link {
            self.draw_link();
        }

        if old.sequence != state.sequence {
            self.draw_sequence();
        }

        if old.rssi != state.rssi {
            self.draw_rssi();
        }

        if old.snr != state.snr {
            self.draw_snr();
        }

        if old.received != state.received {
            self.draw_received();
        }

        if old.missed != state.missed {
            self.draw_missed();
        }
    }

    pub fn update_gps(&mut self, state: GpsDisplayState) {
        let old = self.gps;
        self.gps = state;

        if self.page != DisplayPage::General {
            return;
        }

        // "GPS" status depends on both online and fix.
        if old.online != state.online || old.fix != state.fix {
            self.draw_gps_status();
        }

        if old.satellites != state.satellites {
            self.draw_satellites();
        }

        if old.latitude_e7 != state.latitude_e7 {
            self.draw_latitude();
        }

        if old.longitude_e7 != state.longitude_e7 {
            self.draw_longitude();
        }
    }

    fn draw_static(&mut self) {
        let style = Self::text_style();

        Self::draw_text(
            &mut self.driver,
            "GOLFER GENERAL",
            Point::new(4, 4),
            style,
        );

        Self::draw_text(
            &mut self.driver,
            "MODE: RECEIVER",
            Point::new(4, 18),
            style,
        );

        // Radio labels.
        Self::draw_text(&mut self.driver, "LINK", Point::new(4, LINK_Y), style);
        Self::draw_text(&mut self.driver, "SEQ", Point::new(4, SEQ_Y), style);
        Self::draw_text(&mut self.driver, "RSSI", Point::new(4, RSSI_Y), style);
        Self::draw_text(&mut self.driver, "SNR", Point::new(4, SNR_Y), style);
        Self::draw_text(&mut self.driver, "RX", Point::new(4, RX_Y), style);
        Self::draw_text(
            &mut self.driver,
            "MISSED",
            Point::new(4, MISSED_Y),
            style,
        );

        // GPS labels.
        Self::draw_text(&mut self.driver, "GPS", Point::new(4, GPS_Y), style);
        Self::draw_text(&mut self.driver, "SAT", Point::new(4, SAT_Y), style);
        Self::draw_text(&mut self.driver, "LAT", Point::new(4, LAT_Y), style);
        Self::draw_text(&mut self.driver, "LON", Point::new(4, LON_Y), style);

        // Future/static placeholders. These intentionally do not update yet.
        Self::draw_text(
            &mut self.driver,
            "TIME  --:--:--",
            Point::new(4, 270),
            style,
        );

        Self::draw_text(
            &mut self.driver,
            "BAT   ---- V",
            Point::new(4, 284),
            style,
        );

        Self::draw_text(
            &mut self.driver,
            "ENV/HDG: future",
            Point::new(4, 298),
            style,
        );
    }

    fn redraw_all_dynamic(&mut self) {
        self.draw_link();
        self.draw_sequence();
        self.draw_rssi();
        self.draw_snr();
        self.draw_received();
        self.draw_missed();

        self.draw_gps_status();
        self.draw_satellites();
        self.draw_latitude();
        self.draw_longitude();
    }

    fn draw_link(&mut self) {
        let text = match self.radio.link {
            RadioLinkState::Waiting => "WAITING FOR RX",
            RadioLinkState::Connected => "CONNECTED",
            RadioLinkState::Lost => "*** LOST ***",
        };

        self.draw_value(LINK_Y, text);
    }

    fn draw_sequence(&mut self) {
        let mut line: String<32> = String::new();

        if let Some(sequence) = self.radio.sequence {
            write!(&mut line, "{}", sequence).unwrap();
        } else {
            write!(&mut line, "---").unwrap();
        }

        self.draw_value(SEQ_Y, line.as_str());
    }

    fn draw_rssi(&mut self) {
        let mut line: String<32> = String::new();

        if let Some(rssi) = self.radio.rssi {
            write!(&mut line, "{} dBm", rssi).unwrap();
        } else {
            write!(&mut line, "--- dBm").unwrap();
        }

        self.draw_value(RSSI_Y, line.as_str());
    }

    fn draw_snr(&mut self) {
        let mut line: String<32> = String::new();

        if let Some(snr) = self.radio.snr {
            write!(&mut line, "{} dB", snr).unwrap();
        } else {
            write!(&mut line, "--- dB").unwrap();
        }

        self.draw_value(SNR_Y, line.as_str());
    }

    fn draw_received(&mut self) {
        let mut line: String<32> = String::new();
        write!(&mut line, "{}", self.radio.received).unwrap();
        self.draw_value(RX_Y, line.as_str());
    }

    fn draw_missed(&mut self) {
        let mut line: String<32> = String::new();
        write!(&mut line, "{}", self.radio.missed).unwrap();
        self.draw_value(MISSED_Y, line.as_str());
    }

    fn draw_gps_status(&mut self) {
        let text = if !self.gps.online {
            "OFFLINE"
        } else if self.gps.fix {
            "FIX"
        } else {
            "NO FIX"
        };

        self.draw_value(GPS_Y, text);
    }

    fn draw_satellites(&mut self) {
        let mut line: String<32> = String::new();

        if let Some(satellites) = self.gps.satellites {
            write!(&mut line, "{}", satellites).unwrap();
        } else {
            write!(&mut line, "---").unwrap();
        }

        self.draw_value(SAT_Y, line.as_str());
    }

    fn draw_latitude(&mut self) {
        let mut line: String<32> = String::new();

        if let Some(latitude_e7) = self.gps.latitude_e7 {
            Self::write_coordinate_value(&mut line, latitude_e7);
        } else {
            write!(&mut line, "---").unwrap();
        }

        self.draw_value(LAT_Y, line.as_str());
    }

    fn draw_longitude(&mut self) {
        let mut line: String<32> = String::new();

        if let Some(longitude_e7) = self.gps.longitude_e7 {
            Self::write_coordinate_value(&mut line, longitude_e7);
        } else {
            write!(&mut line, "---").unwrap();
        }

        self.draw_value(LON_Y, line.as_str());
    }

    /// Clear and redraw exactly one dynamic value rectangle.
    fn draw_value(&mut self, y: i32, text: &str) {
        let region = Rectangle::new(
            Point::new(VALUE_X, y),
            Size::new(VALUE_W, VALUE_H),
        );

        self.driver
            .fill_solid(&region, Rgb565::BLACK)
            .unwrap();

        Self::draw_text(
            &mut self.driver,
            text,
            Point::new(VALUE_X, y),
            Self::text_style(),
        );
    }

    fn text_style() -> MonoTextStyle<'static, Rgb565> {
        MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE)
    }

    fn draw_text(
        driver: &mut TftDriver,
        text: &str,
        point: Point,
        style: MonoTextStyle<'static, Rgb565>,
    ) {
        Text::with_baseline(text, point, style, Baseline::Top)
            .draw(driver)
            .unwrap();
    }

    fn write_coordinate_value(
        line: &mut String<32>,
        value_e7: i32,
    ) {
        let negative = value_e7 < 0;
        let absolute = value_e7.unsigned_abs();
        let whole = absolute / 10_000_000;
        let fraction = absolute % 10_000_000;

        if negative {
            write!(
                line,
                "-{}.{:07}",
                whole,
                fraction
            )
            .unwrap();
        } else {
            write!(
                line,
                "{}.{:07}",
                whole,
                fraction
            )
            .unwrap();
        }
    }
}
