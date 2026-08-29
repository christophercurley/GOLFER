use defmt::{error, info, warn};

use embassy_rp::{
    Peri, bind_interrupts,
    peripherals::{PIN_1, UART0},
    uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig},
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

use embedded_io_async::Read;

use heapless::String;
use static_cell::StaticCell;

const GPS_BAUDRATE: u32 = 9_600;
const UART_RX_BUFFER_SIZE: usize = 256;
const NMEA_LINE_CAPACITY: usize = 160;

// GOLFER v1 did not exist before 2024. Some GPS cold-start states can emit
// syntactically valid RMC date/time fields anchored near the GPS/FAT epoch
// before the receiver has learned the current calendar date. Never expose
// those provisional values as authoritative UTC.
const MIN_PLAUSIBLE_UTC_YEAR: u32 = 2024;
const MAX_PLAUSIBLE_UTC_YEAR: u32 = 2079;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

static UART_RX_BUFFER: StaticCell<[u8; UART_RX_BUFFER_SIZE]> = StaticCell::new();

/// Latest local GPS state.
///
/// Logging V2B deliberately carries more than the display currently renders.
/// The PA1616S already provides this information in GGA/RMC; keeping it here
/// lets LOCAL.LOG continue describing the GOLFER's path even when RF is absent.
#[derive(Clone, Copy)]
pub struct GpsState {
    pub online: bool,
    pub fix: bool,
    pub latitude_e7: Option<i32>,
    pub longitude_e7: Option<i32>,
    pub satellites: Option<u8>,
    pub altitude_half_m: Option<i16>,
    pub speed_cm_s: Option<u16>,
    pub course_cdeg: Option<u16>,
    pub hdop_tenths: Option<u8>,
    pub utc_unix_ms: Option<u64>,
}

impl GpsState {
    pub const fn offline() -> Self {
        Self {
            online: false,
            fix: false,
            latitude_e7: None,
            longitude_e7: None,
            satellites: None,
            altitude_half_m: None,
            speed_cm_s: None,
            course_cdeg: None,
            hdop_tenths: None,
            utc_unix_ms: None,
        }
    }
}

/// Latest parsed GPS state.
///
/// A Signal is intentional here: GPS position is state, not an event stream.
/// If the consumer has not yet taken the previous value, a newer fix may
/// replace it. LOCAL.LOG samples the newest state independently at 1 Hz.
pub static GPS_STATE_SIGNAL: Signal<CriticalSectionRawMutex, GpsState> = Signal::new();

/// PA1616S receive / parse task.
///
/// GGA contributes fix/position/satellites/HDOP/altitude. RMC contributes UTC
/// date+time, position, speed over ground, and course over ground. The parser
/// merges both sentence types into one latest-state object.
#[embassy_executor::task]
pub async fn receive_task(uart0: Peri<'static, UART0>, rx_pin: Peri<'static, PIN_1>) {
    let mut config = UartConfig::default();
    config.baudrate = GPS_BAUDRATE;

    let rx_buffer = UART_RX_BUFFER.init([0u8; UART_RX_BUFFER_SIZE]);

    let mut uart = BufferedUartRx::new(uart0, Irqs, rx_pin, rx_buffer, config);

    info!("GPS UART online: UART0 RX on GP1 @ 9600 baud");
    info!("Waiting for PA1616S NMEA data...");

    let mut read_buffer = [0u8; 32];
    let mut line: String<NMEA_LINE_CAPACITY> = String::new();
    let mut state = GpsState::offline();

    loop {
        match uart.read(&mut read_buffer).await {
            Ok(count) => {
                for &byte in &read_buffer[..count] {
                    match byte {
                        b'\n' => {
                            if !line.is_empty() {
                                info!("GPS NMEA: {}", line.as_str());

                                let updated = apply_gga(line.as_str(), &mut state)
                                    || apply_rmc(line.as_str(), &mut state);

                                if updated {
                                    log_state(state);
                                    GPS_STATE_SIGNAL.signal(state);
                                }

                                line.clear();
                            }
                        }

                        b'\r' => {}

                        byte if byte.is_ascii() => {
                            if line.push(byte as char).is_err() {
                                warn!("GPS NMEA line overflow; dropping partial line");
                                line.clear();
                            }
                        }

                        _ => {
                            warn!("GPS UART received non-ASCII byte: {=u8:#04x}", byte);
                        }
                    }
                }
            }

            Err(err) => {
                error!("GPS UART RX error: {:?}", err);
            }
        }
    }
}

fn apply_gga(line: &str, state: &mut GpsState) -> bool {
    let mut fields = line.split(',');

    let Some(sentence) = fields.next() else {
        return false;
    };

    if sentence != "$GPGGA" && sentence != "$GNGGA" {
        return false;
    }

    let _utc_time = fields.next().unwrap_or("");
    let latitude = fields.next().unwrap_or("");
    let latitude_hemisphere = fields.next().unwrap_or("");
    let longitude = fields.next().unwrap_or("");
    let longitude_hemisphere = fields.next().unwrap_or("");
    let fix_quality = parse_u8(fields.next().unwrap_or("")).unwrap_or(0);
    let satellites = parse_u8(fields.next().unwrap_or(""));
    let hdop = fields.next().unwrap_or("");
    let altitude_m = fields.next().unwrap_or("");

    state.online = true;
    state.satellites = satellites;
    state.hdop_tenths = parse_decimal_fixed_i32(hdop, 1)
        .and_then(|value| u8::try_from(value).ok());
    state.altitude_half_m = parse_altitude_half_m(altitude_m);

    if fix_quality == 0 {
        state.fix = false;
        state.latitude_e7 = None;
        state.longitude_e7 = None;
        return true;
    }

    let Some(latitude_e7) = parse_coordinate_e7(latitude, latitude_hemisphere) else {
        return true;
    };
    let Some(longitude_e7) = parse_coordinate_e7(longitude, longitude_hemisphere) else {
        return true;
    };

    state.fix = true;
    state.latitude_e7 = Some(latitude_e7);
    state.longitude_e7 = Some(longitude_e7);
    true
}

fn apply_rmc(line: &str, state: &mut GpsState) -> bool {
    let mut fields = line.split(',');

    let Some(sentence) = fields.next() else {
        return false;
    };

    if sentence != "$GPRMC" && sentence != "$GNRMC" {
        return false;
    }

    let utc_time = fields.next().unwrap_or("");
    let status = fields.next().unwrap_or("");
    let latitude = fields.next().unwrap_or("");
    let latitude_hemisphere = fields.next().unwrap_or("");
    let longitude = fields.next().unwrap_or("");
    let longitude_hemisphere = fields.next().unwrap_or("");
    let speed_knots = fields.next().unwrap_or("");
    let course_degrees = fields.next().unwrap_or("");
    let date = fields.next().unwrap_or("");

    state.online = true;

    // UTC remains valuable even when the receiver reports no navigation fix.
    if let Some(unix_ms) = parse_rmc_unix_ms(date, utc_time) {
        state.utc_unix_ms = Some(unix_ms);
    }

    if status != "A" {
        state.fix = false;
        state.latitude_e7 = None;
        state.longitude_e7 = None;
        state.speed_cm_s = None;
        state.course_cdeg = None;
        return true;
    }

    if let (Some(latitude_e7), Some(longitude_e7)) = (
        parse_coordinate_e7(latitude, latitude_hemisphere),
        parse_coordinate_e7(longitude, longitude_hemisphere),
    ) {
        state.fix = true;
        state.latitude_e7 = Some(latitude_e7);
        state.longitude_e7 = Some(longitude_e7);
    }

    state.speed_cm_s = parse_speed_cm_s(speed_knots);
    state.course_cdeg = parse_course_cdeg(course_degrees);
    true
}

fn parse_u8(value: &str) -> Option<u8> {
    value.parse().ok()
}

fn parse_altitude_half_m(value: &str) -> Option<i16> {
    // Parse altitude to decimeters, then round to the nearest half-meter.
    let decimeters = parse_decimal_fixed_i32(value, 1)?;
    let half_m = round_div_signed(decimeters, 5)?;
    i16::try_from(half_m).ok()
}

fn parse_speed_cm_s(knots: &str) -> Option<u16> {
    // 1 knot = 51.4444... cm/s. Parse to milli-knots to avoid float math.
    let milli_knots = parse_decimal_fixed_i32(knots, 3)?;
    if milli_knots < 0 {
        return None;
    }

    let numerator = u64::try_from(milli_knots).ok()?
        .checked_mul(51_444)?
        .checked_add(500_000)?;
    let cm_s = numerator / 1_000_000;
    u16::try_from(cm_s).ok()
}

fn parse_course_cdeg(degrees: &str) -> Option<u16> {
    let cdeg = parse_decimal_fixed_i32(degrees, 2)?;
    if !(0..36_000).contains(&cdeg) {
        return None;
    }
    u16::try_from(cdeg).ok()
}

/// Parse a signed decimal string into a fixed-point integer with `places`
/// decimal places. Extra decimal digits are validated and truncated.
fn parse_decimal_fixed_i32(value: &str, places: usize) -> Option<i32> {
    if value.is_empty() {
        return None;
    }

    let bytes = value.as_bytes();
    let (negative, digits) = if bytes.first().copied() == Some(b'-') {
        (true, &bytes[1..])
    } else if bytes.first().copied() == Some(b'+') {
        (false, &bytes[1..])
    } else {
        (false, bytes)
    };

    if digits.is_empty() {
        return None;
    }

    let decimal_index = digits.iter().position(|&b| b == b'.').unwrap_or(digits.len());
    let whole = if decimal_index == 0 {
        0
    } else {
        parse_digits(&digits[..decimal_index])?
    };

    let mut scale = 1u32;
    for _ in 0..places {
        scale = scale.checked_mul(10)?;
    }

    let mut fraction = 0u32;
    let mut used = 0usize;
    if decimal_index < digits.len() {
        for &byte in &digits[decimal_index + 1..] {
            if !byte.is_ascii_digit() {
                return None;
            }
            if used < places {
                fraction = fraction.checked_mul(10)?.checked_add(u32::from(byte - b'0'))?;
                used += 1;
            }
        }
    }

    while used < places {
        fraction = fraction.checked_mul(10)?;
        used += 1;
    }

    let magnitude = whole.checked_mul(scale)?.checked_add(fraction)?;
    let signed = i32::try_from(magnitude).ok()?;
    if negative { signed.checked_neg() } else { Some(signed) }
}

fn round_div_signed(value: i32, divisor: i32) -> Option<i32> {
    if divisor <= 0 {
        return None;
    }

    if value >= 0 {
        value.checked_add(divisor / 2)?.checked_div(divisor)
    } else {
        value.checked_sub(divisor / 2)?.checked_div(divisor)
    }
}

fn parse_rmc_unix_ms(date: &str, time: &str) -> Option<u64> {
    if date.len() != 6 || time.len() < 6 {
        return None;
    }

    let date_bytes = date.as_bytes();
    let day = parse_digits(&date_bytes[0..2])?;
    let month = parse_digits(&date_bytes[2..4])?;
    let year_2 = parse_digits(&date_bytes[4..6])?;

    let year = if year_2 >= 80 { 1900 + year_2 } else { 2000 + year_2 };

    // Reject provisional/epoch-like dates before they can poison TIME_SYNC.
    // The upper bound matches the unambiguous range of the NMEA two-digit year
    // mapping used above.
    if !(MIN_PLAUSIBLE_UTC_YEAR..=MAX_PLAUSIBLE_UTC_YEAR).contains(&year) {
        return None;
    }

    let time_bytes = time.as_bytes();
    let hour = parse_digits(&time_bytes[0..2])?;
    let minute = parse_digits(&time_bytes[2..4])?;
    let second = parse_digits(&time_bytes[4..6])?;

    if hour >= 24 || minute >= 60 || second >= 60 {
        return None;
    }

    let days = days_since_unix_epoch(year, month, day)?;
    let seconds = u64::from(days)
        .checked_mul(86_400)?
        .checked_add(u64::from(hour) * 3_600)?
        .checked_add(u64::from(minute) * 60)?
        .checked_add(u64::from(second))?;

    let millis = parse_nmea_time_millis(time)?;
    seconds.checked_mul(1_000)?.checked_add(u64::from(millis))
}

fn parse_nmea_time_millis(time: &str) -> Option<u16> {
    let bytes = time.as_bytes();
    let Some(dot) = bytes.iter().position(|&b| b == b'.') else {
        return Some(0);
    };

    let fraction = &bytes[dot + 1..];
    let mut value = 0u16;
    let mut used = 0usize;

    for &byte in fraction {
        if !byte.is_ascii_digit() {
            return None;
        }
        if used < 3 {
            value = value.checked_mul(10)?.checked_add(u16::from(byte - b'0'))?;
            used += 1;
        }
    }

    while used < 3 {
        value = value.checked_mul(10)?;
        used += 1;
    }

    Some(value)
}

fn days_since_unix_epoch(year: u32, month: u32, day: u32) -> Option<u32> {
    if year < 1970 || !(1..=12).contains(&month) {
        return None;
    }

    let dim = days_in_month(year, month)?;
    if day == 0 || day > dim {
        return None;
    }

    let mut days = 0u32;
    let mut y = 1970u32;
    while y < year {
        days = days.checked_add(if is_leap_year(y) { 366 } else { 365 })?;
        y += 1;
    }

    let mut m = 1u32;
    while m < month {
        days = days.checked_add(days_in_month(year, m)?)?;
        m += 1;
    }

    days.checked_add(day - 1)
}

fn days_in_month(year: u32, month: u32) -> Option<u32> {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => Some(31),
        4 | 6 | 9 | 11 => Some(30),
        2 => Some(if is_leap_year(year) { 29 } else { 28 }),
        _ => None,
    }
}

fn is_leap_year(year: u32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || year % 400 == 0
}

/// Convert an NMEA coordinate such as 3451.5469,N into signed decimal degrees
/// scaled by 10^7. This intentionally avoids floating-point parsing.
fn parse_coordinate_e7(coordinate: &str, hemisphere: &str) -> Option<i32> {
    let bytes = coordinate.as_bytes();

    let decimal_index = bytes
        .iter()
        .position(|&byte| byte == b'.')
        .unwrap_or(bytes.len());

    if decimal_index < 3 {
        return None;
    }

    let degree_digits = decimal_index - 2;

    let degrees = parse_digits(&bytes[..degree_digits])?;
    let whole_minutes = parse_digits(&bytes[degree_digits..decimal_index])?;

    if whole_minutes >= 60 {
        return None;
    }

    let fractional_minutes = if decimal_index < bytes.len() {
        scale_fraction_to_e7(&bytes[decimal_index + 1..])?
    } else {
        0
    };

    let minutes_e7 = whole_minutes
        .checked_mul(10_000_000)?
        .checked_add(fractional_minutes)?;

    let max_degrees = match hemisphere {
        "N" | "S" => 90,
        "E" | "W" => 180,
        _ => return None,
    };

    if degrees > max_degrees || (degrees == max_degrees && minutes_e7 != 0) {
        return None;
    }

    let decimal_degrees_e7 = degrees
        .checked_mul(10_000_000)?
        .checked_add(minutes_e7 / 60)?;

    let signed = i32::try_from(decimal_degrees_e7).ok()?;

    match hemisphere {
        "S" | "W" => signed.checked_neg(),
        "N" | "E" => Some(signed),
        _ => None,
    }
}

fn parse_digits(bytes: &[u8]) -> Option<u32> {
    if bytes.is_empty() {
        return None;
    }

    let mut value = 0u32;

    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }

        value = value.checked_mul(10)?.checked_add(u32::from(byte - b'0'))?;
    }

    Some(value)
}

fn scale_fraction_to_e7(bytes: &[u8]) -> Option<u32> {
    let mut value = 0u32;
    let mut digits_used = 0usize;

    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }

        if digits_used < 7 {
            value = value.checked_mul(10)?.checked_add(u32::from(byte - b'0'))?;
            digits_used += 1;
        }
    }

    while digits_used < 7 {
        value = value.checked_mul(10)?;
        digits_used += 1;
    }

    Some(value)
}

fn log_state(state: GpsState) {
    if state.fix {
        info!(
            "GPS FIX: lat_e7={:?} lon_e7={:?} sats={:?} alt_half_m={:?} speed_cm_s={:?} course_cdeg={:?} hdop_tenths={:?} utc_ms={:?}",
            state.latitude_e7,
            state.longitude_e7,
            state.satellites,
            state.altitude_half_m,
            state.speed_cm_s,
            state.course_cdeg,
            state.hdop_tenths,
            state.utc_unix_ms
        );
    } else {
        info!(
            "GPS NO FIX: sats={:?} hdop_tenths={:?} utc_ms={:?}",
            state.satellites,
            state.hdop_tenths,
            state.utc_unix_ms
        );
    }
}
