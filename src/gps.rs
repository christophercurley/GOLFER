use defmt::{error, info, warn};

use embassy_rp::{
    bind_interrupts,
    peripherals::{PIN_1, UART0},
    uart::{BufferedInterruptHandler, BufferedUartRx, Config as UartConfig},
    Peri,
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

/// First-stage PA1616S bring-up task.
///
/// This intentionally does one thing only:
///
///     PA1616S TX -> Pico GP1 / UART0 RX -> raw NMEA lines -> defmt
///
/// GPS RX / Pico GP0 will be added when we actually need to send
/// configuration commands to the module.
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
                                info!("GPS NMEA: {}", line.as_str());
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
