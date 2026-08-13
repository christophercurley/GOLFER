#![no_std]
#![no_main]

use core::fmt::Write as _;

use defmt::{error, info, warn};
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Level, Output},
    i2c::{Config as I2cConfig, I2c},
};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use heapless::String;

use panic_probe as _;

use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

mod radio;

// The current nRF beacon increments its sequence once per second.
const BEACON_SEQUENCE_RATE_HZ: u64 = 1;

// If no packet arrives for this long, show LINK LOST on the OLED.
const LINK_LOSS_TIMEOUT_SECS: u64 = 5;

// Extra slack when deciding whether a forward sequence jump is physically
// plausible for a 1 Hz beacon. This protects the missed-packet counter from
// corrupted sequence bytes near the edge of reception.
const SEQUENCE_GAP_TOLERANCE: u64 = 5;

// If the transmitter reboots, its sequence starts near zero. A small backward
// jump into this window is treated as a beacon restart rather than corruption.
const REBOOT_SEQUENCE_WINDOW: u32 = 10;

// Current test-packet magic from the nRF beacon.
const PACKET_MAGIC: &[u8; 4] = b"MRU1";

fn decode_sequence(payload: &[u8]) -> Option<u32> {
    if payload.len() < 10 || &payload[0..4] != PACKET_MAGIC {
        return None;
    }

    Some(u32::from_le_bytes([
        payload[6], payload[7], payload[8], payload[9],
    ]))
}

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Pico 2 onboard LED.
    // Pulses briefly whenever a valid packet is accepted.
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("LORAMv1 online");

    // -------------------------------------------------------------------------
    // OLED
    //
    // I2C0
    // GP4 = SDA
    // GP5 = SCL
    // -------------------------------------------------------------------------

    info!("Initializing OLED...");

    let i2c = I2c::new_blocking(
        p.I2C0,
        p.PIN_5, // SCL
        p.PIN_4, // SDA
        I2cConfig::default(),
    );

    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

    display.clear(BinaryColor::Off).unwrap();

    Text::with_baseline(
        "LORAM RECEIVER",
        Point::new(0, 0),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    Text::with_baseline(
        "Waiting for RX...",
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().unwrap();

    info!("OLED initialized");

    // -------------------------------------------------------------------------
    // LoRa radio
    //
    // All SX1262 hardware setup and the proven SF7 RX configuration now live
    // in radio.rs. main.rs only supplies the peripherals and owns application
    // behavior around received packets.
    // -------------------------------------------------------------------------

    let mut radio = match radio::Radio::new(
        p.SPI1, p.PIN_10, // SCK
        p.PIN_11, // MOSI
        p.PIN_12, // MISO
        p.DMA_CH0, p.DMA_CH1, p.PIN_3,  // NSS / CS
        p.PIN_15, // RESET
        p.PIN_20, // DIO1
        p.PIN_2,  // BUSY
    )
    .await
    {
        Ok(radio) => radio,

        Err(err) => {
            error!("Radio initialization aborted: {}", err);

            loop {
                led.set_high();
                Timer::after_millis(100).await;

                led.set_low();
                Timer::after_millis(100).await;
            }
        }
    };

    // -------------------------------------------------------------------------
    // Receiver state
    // -------------------------------------------------------------------------

    let mut rx_buffer = [0u8; radio::RX_BUFFER_SIZE];

    let mut last_sequence: Option<u32> = None;
    let mut last_valid_rx_time: Option<Instant> = None;

    let mut received_packets: u32 = 0;
    let mut missed_packets: u32 = 0;

    let mut last_rssi: Option<i16> = None;
    let mut last_snr: Option<i16> = None;

    let mut link_lost_displayed = false;

    // -------------------------------------------------------------------------
    // Receive packets forever
    // -------------------------------------------------------------------------

    loop {
        rx_buffer.fill(0);

        let rx_result = with_timeout(
            Duration::from_secs(LINK_LOSS_TIMEOUT_SECS),
            radio.receive(&mut rx_buffer),
        )
        .await;

        match rx_result {
            // -----------------------------------------------------------------
            // Radio returned a packet/result before the link-loss timeout.
            // -----------------------------------------------------------------
            Ok(Ok((received_len, packet_status))) => {
                let len = received_len as usize;

                let Some(sequence) = decode_sequence(&rx_buffer[..len]) else {
                    warn!("Ignoring invalid/unknown packet: len={}", received_len);
                    continue;
                };

                let now = Instant::now();

                // -------------------------------------------------------------
                // Sequence validation / packet-loss accounting
                //
                // We deliberately validate the sequence BEFORE updating counters
                // or last_sequence. This prevents one corrupted sequence field
                // from poisoning the rest of the test.
                // -------------------------------------------------------------

                if let Some(last) = last_sequence {
                    if sequence == last {
                        warn!("Ignoring duplicate packet: seq={}", sequence);
                        continue;
                    }

                    if sequence > last {
                        let sequence_gap = sequence - last;

                        // The beacon increments once per second, so compare the
                        // observed jump against the actual time since the last
                        // valid packet, plus a little tolerance.
                        let elapsed_secs = last_valid_rx_time
                            .map(|last_time| now.duration_since(last_time).as_secs())
                            .unwrap_or(0);

                        let max_plausible_gap = elapsed_secs
                            .saturating_mul(BEACON_SEQUENCE_RATE_HZ)
                            .saturating_add(SEQUENCE_GAP_TOLERANCE)
                            .max(1);

                        if u64::from(sequence_gap) > max_plausible_gap {
                            warn!(
                                "Ignoring implausible sequence jump: last={} current={} gap={} max_plausible={}",
                                last, sequence, sequence_gap, max_plausible_gap
                            );
                            continue;
                        }

                        let newly_missed = sequence_gap - 1;

                        if newly_missed > 0 {
                            missed_packets = missed_packets.saturating_add(newly_missed);

                            warn!(
                                "PACKET LOSS: missed {} packet(s) between seq={} and seq={}",
                                newly_missed, last, sequence
                            );
                        }
                    } else {
                        // A real transmitter reboot should restart the sequence
                        // close to zero. Accept that small reset; reject other
                        // backward jumps as out-of-order/corrupted data.
                        if sequence <= REBOOT_SEQUENCE_WINDOW {
                            warn!(
                                "Beacon sequence reset detected: last={} current={}",
                                last, sequence
                            );
                        } else {
                            warn!(
                                "Ignoring backward/out-of-order sequence: last={} current={}",
                                last, sequence
                            );
                            continue;
                        }
                    }
                }

                // Packet is now considered valid and accepted.
                received_packets = received_packets.saturating_add(1);
                last_sequence = Some(sequence);
                last_valid_rx_time = Some(now);
                last_rssi = Some(packet_status.rssi);
                last_snr = Some(packet_status.snr);
                link_lost_displayed = false;

                info!(
                    "RX seq={} RSSI={} dBm SNR={} dB | received={} missed={}",
                    sequence,
                    packet_status.rssi,
                    packet_status.snr,
                    received_packets,
                    missed_packets
                );

                // -------------------------------------------------------------
                // Update OLED with live receiver data.
                // -------------------------------------------------------------

                display.clear(BinaryColor::Off).unwrap();

                let mut line: String<32> = String::new();

                Text::with_baseline(
                    "LORAM RECEIVER",
                    Point::new(0, 0),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                write!(&mut line, "SEQ  {}", sequence).unwrap();
                Text::with_baseline(line.as_str(), Point::new(0, 12), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                line.clear();

                write!(&mut line, "RSSI {} dBm", packet_status.rssi).unwrap();
                Text::with_baseline(line.as_str(), Point::new(0, 24), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                line.clear();

                write!(&mut line, "SNR  {} dB", packet_status.snr).unwrap();
                Text::with_baseline(line.as_str(), Point::new(0, 36), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                line.clear();

                write!(&mut line, "RX {} MISS {}", received_packets, missed_packets).unwrap();
                Text::with_baseline(line.as_str(), Point::new(0, 48), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                display.flush().unwrap();

                // Brief visible indication of an accepted packet.
                led.set_high();
                Timer::after_millis(75).await;
                led.set_low();
            }

            // -----------------------------------------------------------------
            // The radio itself returned an RX error.
            // -----------------------------------------------------------------
            Ok(Err(err)) => {
                error!("RX error: {}", err);
            }

            // -----------------------------------------------------------------
            // No RX result for LINK_LOSS_TIMEOUT_SECS.
            // -----------------------------------------------------------------
            Err(_) => {
                // Before the first valid packet, keep the startup
                // "Waiting for RX..." screen rather than calling it a lost link.
                if last_sequence.is_none() {
                    continue;
                }

                // Only redraw once per outage. When a valid packet returns,
                // link_lost_displayed is cleared and the live screen returns.
                if link_lost_displayed {
                    continue;
                }

                warn!(
                    "LINK LOST: no valid packet for {} seconds",
                    LINK_LOSS_TIMEOUT_SECS
                );

                display.clear(BinaryColor::Off).unwrap();

                let mut line: String<32> = String::new();

                Text::with_baseline(
                    "LORAM RECEIVER",
                    Point::new(0, 0),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                Text::with_baseline(
                    "!!! LINK LOST !!!",
                    Point::new(0, 12),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                if let Some(sequence) = last_sequence {
                    write!(&mut line, "LAST SEQ {}", sequence).unwrap();
                    Text::with_baseline(
                        line.as_str(),
                        Point::new(0, 24),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();

                    line.clear();
                }

                if let (Some(rssi), Some(snr)) = (last_rssi, last_snr) {
                    write!(&mut line, "RSSI {} SNR {}", rssi, snr).unwrap();
                    Text::with_baseline(
                        line.as_str(),
                        Point::new(0, 36),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();

                    line.clear();
                }

                write!(&mut line, "RX {} MISS {}", received_packets, missed_packets).unwrap();
                Text::with_baseline(line.as_str(), Point::new(0, 48), text_style, Baseline::Top)
                    .draw(&mut display)
                    .unwrap();

                display.flush().unwrap();

                link_lost_displayed = true;
            }
        }
    }
}
