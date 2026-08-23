mod backend;
mod ui;

use crate::system::{SystemConfig, SystemInfo};

use core::cell::RefCell;

use embassy_rp::{
    gpio::Output,
    peripherals::{PIN_13, PIN_14, PIN_21},
    Peri,
};

use crate::spi0_bus::Spi0Bus;

pub use backend::TFT_SPI_FREQUENCY_HZ;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DisplayPage {
    Boot,
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

/// Public display façade used by the rest of GOLFER.
///
/// Everything outside this module talks to this type. It deliberately does not
/// expose mipidsi, ILI9341, ST7789, SPI details, or embedded-graphics internals.
pub struct Display {
    backend: backend::Backend,
    page: DisplayPage,
    system_info: SystemInfo,
    system_config: SystemConfig,
    radio: RadioDisplayState,
    gps: GpsDisplayState,
}

impl Display {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        bus: &'static RefCell<Spi0Bus>,
        cs: Output<'static>,
        dc: Peri<'static, PIN_13>,
        reset: Peri<'static, PIN_14>,
        backlight: Peri<'static, PIN_21>,
        system_info: SystemInfo,
        system_config: SystemConfig,
    ) -> Self {
        let backend = backend::Backend::new(
            bus,
            cs,
            dc,
            reset,
            backlight,
        );

        let mut display = Self {
            backend,
            page: DisplayPage::Boot,
            system_info,
            system_config,
            radio: RadioDisplayState::waiting(),
            gps: GpsDisplayState::offline(),
        };

        display.redraw_page();
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
        self.redraw_page();
    }

    pub fn update_radio(&mut self, state: RadioDisplayState) {
        let old = self.radio;
        self.radio = state;

        if self.page != DisplayPage::General {
            return;
        }

        let target = self.backend.target();

        if old.link != state.link {
            ui::draw_link(target, &state);
        }

        if old.sequence != state.sequence {
            ui::draw_sequence(target, &state);
        }

        if old.rssi != state.rssi {
            ui::draw_rssi(target, &state);
        }

        if old.snr != state.snr {
            ui::draw_snr(target, &state);
        }

        if old.received != state.received {
            ui::draw_received(target, &state);
        }

        if old.missed != state.missed {
            ui::draw_missed(target, &state);
        }
    }

    pub fn update_gps(&mut self, state: GpsDisplayState) {
        let old = self.gps;
        self.gps = state;

        if self.page != DisplayPage::General {
            return;
        }

        let target = self.backend.target();

        if old.online != state.online || old.fix != state.fix {
            ui::draw_gps_status(target, &state);
        }

        if old.satellites != state.satellites {
            ui::draw_satellites(target, &state);
        }

        if old.latitude_e7 != state.latitude_e7 {
            ui::draw_latitude(target, &state);
        }

        if old.longitude_e7 != state.longitude_e7 {
            ui::draw_longitude(target, &state);
        }
    }

    fn redraw_page(&mut self) {
        let target = self.backend.target();

        match self.page {
            DisplayPage::Boot => {
                ui::clear_screen(target);
                ui::draw_boot(target, &self.system_info, &self.system_config);
            }

            DisplayPage::General => {
                ui::clear_screen(target);
                ui::draw_general_static(target);
                ui::draw_all_radio(target, &self.radio);
                ui::draw_all_gps(target, &self.gps);
            }
        }
    }
}
