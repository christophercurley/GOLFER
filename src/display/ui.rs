use core::{
    fmt::{Debug, Write as _},
};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
    text::{Baseline, Text},
};
use heapless::String;

use super::{
    GpsDisplayState,
    RadioDisplayState,
    RadioLinkState,
};

// -----------------------------------------------------------------------------
// HARDWARE-INDEPENDENT UI
//
// This file knows NOTHING about:
//   * ILI9341
//   * ST7789
//   * mipidsi
//   * SPI
//   * GPIO pins
//
// It renders against embedded-graphics' DrawTarget trait only.
//
// Any future TFT backend that presents a DrawTarget<Color = Rgb565> can use
// this UI without changing this file.
// -----------------------------------------------------------------------------

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

pub fn clear_screen<D>(target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    target.clear(Rgb565::BLACK).unwrap();
}

pub fn draw_general_static<D>(target: &mut D)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let style = text_style();

    draw_text(target, "GOLFER GENERAL", Point::new(4, 4), style);
    draw_text(target, "MODE: RECEIVER", Point::new(4, 18), style);

    draw_text(target, "LINK", Point::new(4, LINK_Y), style);
    draw_text(target, "SEQ", Point::new(4, SEQ_Y), style);
    draw_text(target, "RSSI", Point::new(4, RSSI_Y), style);
    draw_text(target, "SNR", Point::new(4, SNR_Y), style);
    draw_text(target, "RX", Point::new(4, RX_Y), style);
    draw_text(target, "MISSED", Point::new(4, MISSED_Y), style);

    draw_text(target, "GPS", Point::new(4, GPS_Y), style);
    draw_text(target, "SAT", Point::new(4, SAT_Y), style);
    draw_text(target, "LAT", Point::new(4, LAT_Y), style);
    draw_text(target, "LON", Point::new(4, LON_Y), style);

    // Placeholders until these data sources exist.
    draw_text(target, "TIME  --:--:--", Point::new(4, 270), style);
    draw_text(target, "BAT   ---- V", Point::new(4, 284), style);
    draw_text(target, "ENV/HDG: future", Point::new(4, 298), style);
}

pub fn draw_all_radio<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    draw_link(target, state);
    draw_sequence(target, state);
    draw_rssi(target, state);
    draw_snr(target, state);
    draw_received(target, state);
    draw_missed(target, state);
}

pub fn draw_all_gps<D>(
    target: &mut D,
    state: &GpsDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    draw_gps_status(target, state);
    draw_satellites(target, state);
    draw_latitude(target, state);
    draw_longitude(target, state);
}

pub fn draw_link<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let text = match state.link {
        RadioLinkState::Waiting => "WAITING FOR RX",
        RadioLinkState::Connected => "CONNECTED",
        RadioLinkState::Lost => "*** LOST ***",
    };

    draw_value(target, LINK_Y, text);
}

pub fn draw_sequence<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();

    if let Some(sequence) = state.sequence {
        write!(&mut line, "{}", sequence).unwrap();
    } else {
        write!(&mut line, "---").unwrap();
    }

    draw_value(target, SEQ_Y, line.as_str());
}

pub fn draw_rssi<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();

    if let Some(rssi) = state.rssi {
        write!(&mut line, "{} dBm", rssi).unwrap();
    } else {
        write!(&mut line, "--- dBm").unwrap();
    }

    draw_value(target, RSSI_Y, line.as_str());
}

pub fn draw_snr<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();

    if let Some(snr) = state.snr {
        write!(&mut line, "{} dB", snr).unwrap();
    } else {
        write!(&mut line, "--- dB").unwrap();
    }

    draw_value(target, SNR_Y, line.as_str());
}

pub fn draw_received<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();
    write!(&mut line, "{}", state.received).unwrap();
    draw_value(target, RX_Y, line.as_str());
}

pub fn draw_missed<D>(
    target: &mut D,
    state: &RadioDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();
    write!(&mut line, "{}", state.missed).unwrap();
    draw_value(target, MISSED_Y, line.as_str());
}

pub fn draw_gps_status<D>(
    target: &mut D,
    state: &GpsDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let text = if !state.online {
        "OFFLINE"
    } else if state.fix {
        "FIX"
    } else {
        "NO FIX"
    };

    draw_value(target, GPS_Y, text);
}

pub fn draw_satellites<D>(
    target: &mut D,
    state: &GpsDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();

    if let Some(satellites) = state.satellites {
        write!(&mut line, "{}", satellites).unwrap();
    } else {
        write!(&mut line, "---").unwrap();
    }

    draw_value(target, SAT_Y, line.as_str());
}

pub fn draw_latitude<D>(
    target: &mut D,
    state: &GpsDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();

    if let Some(latitude_e7) = state.latitude_e7 {
        write_coordinate_value(&mut line, latitude_e7);
    } else {
        write!(&mut line, "---").unwrap();
    }

    draw_value(target, LAT_Y, line.as_str());
}

pub fn draw_longitude<D>(
    target: &mut D,
    state: &GpsDisplayState,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let mut line: String<32> = String::new();

    if let Some(longitude_e7) = state.longitude_e7 {
        write_coordinate_value(&mut line, longitude_e7);
    } else {
        write!(&mut line, "---").unwrap();
    }

    draw_value(target, LON_Y, line.as_str());
}

fn draw_value<D>(
    target: &mut D,
    y: i32,
    text: &str,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let region = Rectangle::new(
        Point::new(VALUE_X, y),
        Size::new(VALUE_W, VALUE_H),
    );

    target
        .fill_solid(&region, Rgb565::BLACK)
        .unwrap();

    draw_text(
        target,
        text,
        Point::new(VALUE_X, y),
        text_style(),
    );
}

fn text_style() -> MonoTextStyle<'static, Rgb565> {
    MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE)
}

fn draw_text<D>(
    target: &mut D,
    text: &str,
    point: Point,
    style: MonoTextStyle<'static, Rgb565>,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    Text::with_baseline(
        text,
        point,
        style,
        Baseline::Top,
    )
    .draw(target)
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
