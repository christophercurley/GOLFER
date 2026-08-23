use core::fmt::Debug;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
    text::{Baseline, Text},
};

use super::{BootStatus, InitStatus, InitSubsystem};

// -----------------------------------------------------------------------------
// BOOT / INITIALIZATION STATUS UI
//
// Keep this intentionally simple for now. The goal is a responsive, useful
// startup screen that communicates subsystem state immediately.
//
// Yellow = initializing
// Green  = OK
// Red    = NOK
// -----------------------------------------------------------------------------

const LABEL_X: i32 = 12;
const STATUS_X: i32 = 104;
const ROW_H: u32 = 18;
const STATUS_W: u32 = 124;

const SYSTEM_Y: i32 = 72;
const DISPLAY_Y: i32 = 94;
const SD_Y: i32 = 116;
const GPS_Y: i32 = 138;
const LORA_Y: i32 = 160;

pub fn draw<D>(
    target: &mut D,
    status: &BootStatus,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    target.clear(Rgb565::BLACK).unwrap();

    draw_text(
        target,
        "GOLFER",
        Point::new(12, 16),
        white_style(),
    );

    draw_text(
        target,
        "SYSTEM INITIALIZATION",
        Point::new(12, 34),
        white_style(),
    );

    draw_row(target, InitSubsystem::System, status.system);
    draw_row(target, InitSubsystem::Display, status.display);
    draw_row(target, InitSubsystem::SdCard, status.sd_card);
    draw_row(target, InitSubsystem::Gps, status.gps);
    draw_row(target, InitSubsystem::Lora, status.lora);
}

pub fn update_row<D>(
    target: &mut D,
    subsystem: InitSubsystem,
    status: InitStatus,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    draw_row(target, subsystem, status);
}

fn draw_row<D>(
    target: &mut D,
    subsystem: InitSubsystem,
    status: InitStatus,
)
where
    D: DrawTarget<Color = Rgb565>,
    D::Error: Debug,
{
    let y = row_y(subsystem);

    draw_text(
        target,
        label(subsystem),
        Point::new(LABEL_X, y),
        white_style(),
    );

    let region = Rectangle::new(
        Point::new(STATUS_X, y),
        Size::new(STATUS_W, ROW_H),
    );

    target
        .fill_solid(&region, Rgb565::BLACK)
        .unwrap();

    draw_text(
        target,
        status_text(status),
        Point::new(STATUS_X, y),
        status_style(status),
    );
}

fn row_y(subsystem: InitSubsystem) -> i32 {
    match subsystem {
        InitSubsystem::System => SYSTEM_Y,
        InitSubsystem::Display => DISPLAY_Y,
        InitSubsystem::SdCard => SD_Y,
        InitSubsystem::Gps => GPS_Y,
        InitSubsystem::Lora => LORA_Y,
    }
}

fn label(subsystem: InitSubsystem) -> &'static str {
    match subsystem {
        InitSubsystem::System => "SYSTEM",
        InitSubsystem::Display => "DISPLAY",
        InitSubsystem::SdCard => "SD CARD",
        InitSubsystem::Gps => "GPS",
        InitSubsystem::Lora => "LORA",
    }
}

fn status_text(status: InitStatus) -> &'static str {
    match status {
        InitStatus::Initializing => "initializing...",
        InitStatus::Ok => "OK",
        InitStatus::Nok => "NOK",
    }
}

fn white_style() -> MonoTextStyle<'static, Rgb565> {
    MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE)
}

fn status_style(
    status: InitStatus,
) -> MonoTextStyle<'static, Rgb565> {
    let color = match status {
        InitStatus::Initializing => Rgb565::YELLOW,
        InitStatus::Ok => Rgb565::GREEN,
        // Prototype ILI9341 board currently presents red/blue swapped
        // relative to embedded-graphics' logical RGB565 constants.
        // Keep this hardware quirk local for now; the final display backend
        // can normalize color order properly later.
        InitStatus::Nok => Rgb565::BLUE,
    };

    MonoTextStyle::new(&FONT_6X10, color)
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
