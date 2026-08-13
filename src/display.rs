use core::fmt::Write as _;

use defmt::info;

use embassy_rp::{
    i2c::{Blocking, Config as I2cConfig, I2c},
    peripherals::{I2C0, PIN_4, PIN_5},
    Peri,
};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use heapless::String;

use ssd1306::{
    mode::BufferedGraphicsMode,
    prelude::*,
    I2CDisplayInterface,
    Ssd1306,
};

type OledI2c = I2c<'static, I2C0, Blocking>;
type OledInterface = I2CInterface<OledI2c>;
type OledDriver = Ssd1306<
    OledInterface,
    DisplaySize128x64,
    BufferedGraphicsMode<DisplaySize128x64>,
>;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DisplayPage {
    Radio,
    Gps,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum RadioLinkState {
    Waiting,
    Connected,
    Lost,
}

#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
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

/// Owns the current OLED hardware and all drawing behavior.
///
/// The rest of the application provides *state*; this module decides how that
/// state is rendered. When the 2.4" TFT arrives, most application code should
/// remain unchanged while this module is replaced/expanded.
pub struct Display {
    driver: OledDriver,
    page: DisplayPage,
    radio: RadioDisplayState,
    gps: GpsDisplayState,
}

impl Display {
    pub fn new(
        i2c0: Peri<'static, I2C0>,
        scl: Peri<'static, PIN_5>,
        sda: Peri<'static, PIN_4>,
        initial_page: DisplayPage,
    ) -> Self {
        info!("Initializing OLED...");

        let i2c = I2c::new_blocking(i2c0, scl, sda, I2cConfig::default());
        let interface = I2CDisplayInterface::new(i2c);

        let mut driver = Ssd1306::new(
            interface,
            DisplaySize128x64,
            DisplayRotation::Rotate0,
        )
        .into_buffered_graphics_mode();

        driver.init().unwrap();

        let mut display = Self {
            driver,
            page: initial_page,
            radio: RadioDisplayState::waiting(),
            gps: GpsDisplayState::offline(),
        };

        display.render_current();

        info!("OLED initialized");

        display
    }

    pub fn page(&self) -> DisplayPage {
        self.page
    }

    /// Switch pages and immediately redraw using the most recently supplied
    /// state for that page.
    pub fn set_page(&mut self, page: DisplayPage) {
        if self.page == page {
            return;
        }

        self.page = page;
        self.render_current();
    }

    /// Convenience hook for a future button/encoder/UI action.
    pub fn toggle_page(&mut self) {
        let next = match self.page {
            DisplayPage::Radio => DisplayPage::Gps,
            DisplayPage::Gps => DisplayPage::Radio,
        };

        self.set_page(next);
    }

    /// Store the latest radio state. Redraw only if the radio page is active.
    pub fn update_radio(&mut self, state: RadioDisplayState) {
        self.radio = state;

        if self.page == DisplayPage::Radio {
            self.render_radio();
        }
    }

    /// Store the latest GPS state. Redraw only if the GPS page is active.
    pub fn update_gps(&mut self, state: GpsDisplayState) {
        self.gps = state;

        if self.page == DisplayPage::Gps {
            self.render_gps();
        }
    }

    fn render_current(&mut self) {
        match self.page {
            DisplayPage::Radio => self.render_radio(),
            DisplayPage::Gps => self.render_gps(),
        }
    }

    fn render_radio(&mut self) {
        self.driver.clear(BinaryColor::Off).unwrap();

        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let mut line: String<32> = String::new();

        Self::draw_text(
            &mut self.driver,
            "LORAM RECEIVER",
            Point::new(0, 0),
            text_style,
        );

        match self.radio.link {
            RadioLinkState::Waiting => {
                Self::draw_text(
                    &mut self.driver,
                    "Waiting for RX...",
                    Point::new(0, 16),
                    text_style,
                );
            }

            RadioLinkState::Connected => {
                if let Some(sequence) = self.radio.sequence {
                    write!(&mut line, "SEQ  {}", sequence).unwrap();
                    Self::draw_text(
                        &mut self.driver,
                        line.as_str(),
                        Point::new(0, 12),
                        text_style,
                    );
                    line.clear();
                }

                if let Some(rssi) = self.radio.rssi {
                    write!(&mut line, "RSSI {} dBm", rssi).unwrap();
                    Self::draw_text(
                        &mut self.driver,
                        line.as_str(),
                        Point::new(0, 24),
                        text_style,
                    );
                    line.clear();
                }

                if let Some(snr) = self.radio.snr {
                    write!(&mut line, "SNR  {} dB", snr).unwrap();
                    Self::draw_text(
                        &mut self.driver,
                        line.as_str(),
                        Point::new(0, 36),
                        text_style,
                    );
                    line.clear();
                }

                write!(
                    &mut line,
                    "RX {} MISS {}",
                    self.radio.received,
                    self.radio.missed
                )
                .unwrap();

                Self::draw_text(
                    &mut self.driver,
                    line.as_str(),
                    Point::new(0, 48),
                    text_style,
                );
            }

            RadioLinkState::Lost => {
                Self::draw_text(
                    &mut self.driver,
                    "!!! LINK LOST !!!",
                    Point::new(0, 12),
                    text_style,
                );

                if let Some(sequence) = self.radio.sequence {
                    write!(&mut line, "LAST SEQ {}", sequence).unwrap();
                    Self::draw_text(
                        &mut self.driver,
                        line.as_str(),
                        Point::new(0, 24),
                        text_style,
                    );
                    line.clear();
                }

                if let (Some(rssi), Some(snr)) = (self.radio.rssi, self.radio.snr) {
                    write!(&mut line, "RSSI {} SNR {}", rssi, snr).unwrap();
                    Self::draw_text(
                        &mut self.driver,
                        line.as_str(),
                        Point::new(0, 36),
                        text_style,
                    );
                    line.clear();
                }

                write!(
                    &mut line,
                    "RX {} MISS {}",
                    self.radio.received,
                    self.radio.missed
                )
                .unwrap();

                Self::draw_text(
                    &mut self.driver,
                    line.as_str(),
                    Point::new(0, 48),
                    text_style,
                );
            }
        }

        self.driver.flush().unwrap();
    }

    fn render_gps(&mut self) {
        self.driver.clear(BinaryColor::Off).unwrap();

        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let mut line: String<32> = String::new();

        Self::draw_text(
            &mut self.driver,
            "LORAM GPS",
            Point::new(0, 0),
            text_style,
        );

        if !self.gps.online {
            Self::draw_text(
                &mut self.driver,
                "GPS NOT ONLINE",
                Point::new(0, 16),
                text_style,
            );

            Self::draw_text(
                &mut self.driver,
                "Awaiting gps.rs",
                Point::new(0, 28),
                text_style,
            );

            self.driver.flush().unwrap();
            return;
        }

        if self.gps.fix {
            if let Some(satellites) = self.gps.satellites {
                write!(&mut line, "FIX  SAT {}", satellites).unwrap();
            } else {
                write!(&mut line, "GPS FIX").unwrap();
            }
        } else if let Some(satellites) = self.gps.satellites {
            write!(&mut line, "NO FIX SAT {}", satellites).unwrap();
        } else {
            write!(&mut line, "NO GPS FIX").unwrap();
        }

        Self::draw_text(
            &mut self.driver,
            line.as_str(),
            Point::new(0, 12),
            text_style,
        );
        line.clear();

        if let Some(latitude_e7) = self.gps.latitude_e7 {
            Self::write_coordinate(&mut line, "LAT", latitude_e7);
            Self::draw_text(
                &mut self.driver,
                line.as_str(),
                Point::new(0, 26),
                text_style,
            );
            line.clear();
        }

        if let Some(longitude_e7) = self.gps.longitude_e7 {
            Self::write_coordinate(&mut line, "LON", longitude_e7);
            Self::draw_text(
                &mut self.driver,
                line.as_str(),
                Point::new(0, 40),
                text_style,
            );
        }

        self.driver.flush().unwrap();
    }

    fn draw_text(
        driver: &mut OledDriver,
        text: &str,
        point: Point,
        style: MonoTextStyle<'static, BinaryColor>,
    ) {
        Text::with_baseline(text, point, style, Baseline::Top)
            .draw(driver)
            .unwrap();
    }

    fn write_coordinate(line: &mut String<32>, label: &str, value_e7: i32) {
        let negative = value_e7 < 0;
        let absolute = value_e7.unsigned_abs();
        let whole = absolute / 10_000_000;
        let fraction = absolute % 10_000_000;

        if negative {
            write!(
                line,
                "{} -{}.{:07}",
                label,
                whole,
                fraction
            )
            .unwrap();
        } else {
            write!(
                line,
                "{} {}.{:07}",
                label,
                whole,
                fraction
            )
            .unwrap();
        }
    }
}
