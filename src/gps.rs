use defmt::{error, info, warn};

use embassy_rp::{
    bind_interrupts,
    peripherals::{PIN_1, UART0},
    uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig},
    Peri,
};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};

use embedded_io_async::Read;

use heapless::String;
use static_cell::StaticCell;

const GPS_BAUDRATE: u32 = 9_600;
const UART_RX_BUFFER_SIZE: usize = 256;
const NMEA_LINE_CAPACITY: usize = 160;

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

static UART_RX_BUFFER: StaticCell<[u8; UART_RX_BUFFER_SIZE]> = StaticCell::new();

#[derive(Clone, Copy)]
pub struct GpsState {
    pub online: bool,
    pub fix: bool,
    pub latitude_e7: Option<i32>,
    pub longitude_e7: Option<i32>,
    pub satellites: Option<u8>,
}

impl GpsState {
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

/// Latest parsed GPS state.
///
/// A Signal is intentional here: GPS position is state, not an event stream.
/// If the consumer has not yet taken the previous value, a newer fix may
/// replace it. main.rs only needs the newest position for the display.
pub static GPS_STATE_SIGNAL:
    Signal<CriticalSectionRawMutex, GpsState> =
    Signal::new();

/// PA1616S receive / parse task.
///
/// Stage B keeps the proven raw NMEA logging from Stage A, parses GGA sentences,
/// and publishes the newest position state to the rest of the application.
///
/// Current hardware path:
///
///     PA1616S TX -> Pico GP1 / UART0 RX
///
/// Pico GP0 remains reserved for GPS RX when we later add outbound
/// configuration commands.
#[embassy_executor::task]
pub async fn receive_task(
    uart0: Peri<'static, UART0>,
    rx_pin: Peri<'static, PIN_1>,
) {
    let mut config = UartConfig::default();
    config.baudrate = GPS_BAUDRATE;

    let rx_buffer = UART_RX_BUFFER.init([0u8; UART_RX_BUFFER_SIZE]);

    let mut uart = BufferedUartRx::new(
        uart0,
        Irqs,
        rx_pin,
        rx_buffer,
        config,
    );

    info!("GPS UART online: UART0 RX on GP1 @ 9600 baud");
    info!("Waiting for PA1616S NMEA data...");

    let mut read_buffer = [0u8; 32];
    let mut line: String<NMEA_LINE_CAPACITY> = String::new();

    loop {
        match uart.read(&mut read_buffer).await {
            Ok(count) => {
                for &byte in &read_buffer[..count] {
                    match byte {
                        b'\n' => {
                            if !line.is_empty() {
                                // Keep the raw sentence visible while the parser
                                // is still young. This is extremely useful when
                                // validating odd GPS behavior in the field.
                                info!("GPS NMEA: {}", line.as_str());

                                if let Some(state) = parse_gga(line.as_str()) {
                                    log_state(state);
                                    GPS_STATE_SIGNAL.signal(state);
                                }

                                line.clear();
                            }
                        }

                        b'\r' => {
                            // Ignore CR; LF terminates the NMEA sentence.
                        }

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

/// Parse the minimum GGA state LORAM v1 currently needs:
/// fix validity, latitude, longitude, and satellites used.
///
/// Both GP and GN talker IDs are accepted so the parser is not coupled to a
/// single NMEA talker prefix.
fn parse_gga(line: &str) -> Option<GpsState> {
    let mut fields = line.split(',');

    let sentence = fields.next()?;

    if sentence != "$GPGGA" && sentence != "$GNGGA" {
        return None;
    }

    let _utc_time = fields.next()?;
    let latitude = fields.next()?;
    let latitude_hemisphere = fields.next()?;
    let longitude = fields.next()?;
    let longitude_hemisphere = fields.next()?;
    let fix_quality = parse_u8(fields.next()?).unwrap_or(0);
    let satellites = parse_u8(fields.next()?);

    // GGA fix quality 0 means the position fields are not currently valid.
    if fix_quality == 0 {
        return Some(GpsState {
            online: true,
            fix: false,
            latitude_e7: None,
            longitude_e7: None,
            satellites,
        });
    }

    let latitude_e7 =
        parse_coordinate_e7(latitude, latitude_hemisphere)?;
    let longitude_e7 =
        parse_coordinate_e7(longitude, longitude_hemisphere)?;

    Some(GpsState {
        online: true,
        fix: true,
        latitude_e7: Some(latitude_e7),
        longitude_e7: Some(longitude_e7),
        satellites,
    })
}

fn parse_u8(value: &str) -> Option<u8> {
    value.parse().ok()
}

/// Convert an NMEA coordinate such as:
///
///     3451.5469,N
///
/// into signed decimal degrees scaled by 10^7:
///
///     348591150  ==  34.8591150 degrees
///
/// This intentionally avoids floating-point parsing.
fn parse_coordinate_e7(
    coordinate: &str,
    hemisphere: &str,
) -> Option<i32> {
    let bytes = coordinate.as_bytes();

    let decimal_index = bytes
        .iter()
        .position(|&byte| byte == b'.')
        .unwrap_or(bytes.len());

    // We need at least one degree digit plus the two minute digits.
    if decimal_index < 3 {
        return None;
    }

    let degree_digits = decimal_index - 2;

    let degrees = parse_digits(&bytes[..degree_digits])?;
    let whole_minutes =
        parse_digits(&bytes[degree_digits..decimal_index])?;

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

    if degrees > max_degrees {
        return None;
    }

    if degrees == max_degrees && minutes_e7 != 0 {
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

        value = value
            .checked_mul(10)?
            .checked_add(u32::from(byte - b'0'))?;
    }

    Some(value)
}

/// Scale a decimal fraction to seven digits.
///
/// Examples:
///
///     "5469"    -> 5_469_000
///     "5"       -> 5_000_000
///     "1234567" -> 1_234_567
///
/// Extra digits beyond seven are validated but intentionally truncated because
/// GpsState stores coordinates at 1e-7 degree resolution.
fn scale_fraction_to_e7(bytes: &[u8]) -> Option<u32> {
    let mut value = 0u32;
    let mut digits_used = 0usize;

    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }

        if digits_used < 7 {
            value = value
                .checked_mul(10)?
                .checked_add(u32::from(byte - b'0'))?;

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
    match (
        state.fix,
        state.latitude_e7,
        state.longitude_e7,
        state.satellites,
    ) {
        (true, Some(latitude), Some(longitude), Some(satellites)) => {
            info!(
                "GPS FIX: lat_e7={} lon_e7={} sats={}",
                latitude,
                longitude,
                satellites
            );
        }

        (true, Some(latitude), Some(longitude), None) => {
            info!(
                "GPS FIX: lat_e7={} lon_e7={} sats=?",
                latitude,
                longitude
            );
        }

        (_, _, _, Some(satellites)) => {
            info!("GPS NO FIX: sats={}", satellites);
        }

        _ => {
            info!("GPS NO FIX");
        }
    }
}
