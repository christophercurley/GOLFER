#![no_std]
#![no_main]

use core::fmt::Write as _;

use defmt::{error, info, warn};
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull},
    i2c::{Config as I2cConfig, I2c},
    peripherals::{DMA_CH0, DMA_CH1},
    spi::{Config as SpiConfig, Spi},
};
use embassy_time::{with_timeout, Delay, Duration, Instant, Timer};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal_bus::spi::ExclusiveDevice;

use heapless::String;

use lora_phy::{
    iv::GenericSx126xInterfaceVariant,
    mod_params::{Bandwidth, CodingRate, SpreadingFactor},
    sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage},
    LoRa, RxMode,
};

use panic_probe as _;

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const LORA_FREQUENCY_HZ: u32 = 915_000_000;

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

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>;
});

fn decode_sequence(payload: &[u8]) -> Option<u32> {
    if payload.len() < 10 || &payload[0..4] != PACKET_MAGIC {
        return None;
    }

    Some(u32::from_le_bytes([
        payload[6],
        payload[7],
        payload[8],
        payload[9],
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

    let mut display = Ssd1306::new(
        interface,
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
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
    // Waveshare Pico-LoRa-SX1262 pin mapping
    //
    // GP2  = BUSY
    // GP3  = NSS / CS
    // GP10 = SPI1 SCK
    // GP11 = SPI1 MOSI
    // GP12 = SPI1 MISO
    // GP15 = RESET
    // GP20 = DIO1
    // -------------------------------------------------------------------------

    let nss = Output::new(p.PIN_3, Level::High);
    let reset = Output::new(p.PIN_15, Level::High);

    let dio1 = Input::new(p.PIN_20, Pull::None);
    let busy = Input::new(p.PIN_2, Pull::None);

    // -------------------------------------------------------------------------
    // SPI1
    // -------------------------------------------------------------------------

    let spi = Spi::new(
        p.SPI1,
        p.PIN_10,
        p.PIN_11,
        p.PIN_12,
        p.DMA_CH0,
        p.DMA_CH1,
        Irqs,
        SpiConfig::default(),
    );

    let spi = ExclusiveDevice::new(spi, nss, Delay).unwrap();

    // -------------------------------------------------------------------------
    // SX1262 interface
    // -------------------------------------------------------------------------

    let interface =
        GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    let radio_config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: false,
    };

    info!("Initializing SX1262...");

    let mut lora = match LoRa::new(
        Sx126x::new(spi, interface, radio_config),
        false, // private LoRa sync word
        Delay,
    )
    .await
    {
        Ok(lora) => {
            info!("SX1262 initialization successful");
            lora
        }

        Err(err) => {
            error!("SX1262 initialization FAILED: {}", err);

            loop {
                led.set_high();
                Timer::after_millis(100).await;

                led.set_low();
                Timer::after_millis(100).await;
            }
        }
    };

    // -------------------------------------------------------------------------
    // LoRa modulation parameters
    //
    // These exactly match the current nRF52840 transmitter:
    //
    // 915 MHz
    // SF7
    // BW 125 kHz
    // Coding rate 4/5
    // -------------------------------------------------------------------------

    let modulation_params = match lora.create_modulation_params(
        SpreadingFactor::_7,
        Bandwidth::_125KHz,
        CodingRate::_4_5,
        LORA_FREQUENCY_HZ,
    ) {
        Ok(params) => params,

        Err(err) => {
            error!("Failed to create modulation params: {}", err);

            loop {
                Timer::after_secs(1).await;
            }
        }
    };

    // -------------------------------------------------------------------------
    // RX packet parameters
    //
    // These also exactly match the transmitter:
    //
    // 8-symbol preamble
    // explicit header
    // CRC enabled
    // normal IQ
    // -------------------------------------------------------------------------

    let mut rx_buffer = [0u8; 255];

    let rx_packet_params = match lora.create_rx_packet_params(
        8,                     // preamble symbols
        false,                 // explicit header
        rx_buffer.len() as u8, // maximum payload length
        true,                  // CRC enabled
        false,                 // normal IQ
        &modulation_params,
    ) {
        Ok(params) => params,

        Err(err) => {
            error!("Failed to create RX packet params: {}", err);

            loop {
                Timer::after_secs(1).await;
            }
        }
    };

    // -------------------------------------------------------------------------
    // Enter continuous RX mode
    // -------------------------------------------------------------------------

    info!("Preparing continuous RX...");

    match lora
        .prepare_for_rx(
            RxMode::Continuous,
            &modulation_params,
            &rx_packet_params,
        )
        .await
    {
        Ok(()) => {
            info!("LORAMv1 RX READY");
        }

        Err(err) => {
            error!("Failed to prepare RX: {}", err);

            loop {
                Timer::after_secs(1).await;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Receiver state
    // -------------------------------------------------------------------------

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
            lora.rx(&rx_packet_params, &mut rx_buffer),
        )
        .await;

        match rx_result {
            // -----------------------------------------------------------------
            // Radio returned a packet/result before the link-loss timeout.
            // -----------------------------------------------------------------
            Ok(Ok((received_len, packet_status))) => {
                let len = received_len as usize;

                let Some(sequence) = decode_sequence(&rx_buffer[..len]) else {
                    warn!(
                        "Ignoring invalid/unknown packet: len={}",
                        received_len
                    );
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
                                last,
                                sequence,
                                sequence_gap,
                                max_plausible_gap
                            );
                            continue;
                        }

                        let newly_missed = sequence_gap - 1;

                        if newly_missed > 0 {
                            missed_packets = missed_packets.saturating_add(newly_missed);

                            warn!(
                                "PACKET LOSS: missed {} packet(s) between seq={} and seq={}",
                                newly_missed,
                                last,
                                sequence
                            );
                        }
                    } else {
                        // A real transmitter reboot should restart the sequence
                        // close to zero. Accept that small reset; reject other
                        // backward jumps as out-of-order/corrupted data.
                        if sequence <= REBOOT_SEQUENCE_WINDOW {
                            warn!(
                                "Beacon sequence reset detected: last={} current={}",
                                last,
                                sequence
                            );
                        } else {
                            warn!(
                                "Ignoring backward/out-of-order sequence: last={} current={}",
                                last,
                                sequence
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
                Text::with_baseline(
                    line.as_str(),
                    Point::new(0, 12),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                line.clear();

                write!(&mut line, "RSSI {} dBm", packet_status.rssi).unwrap();
                Text::with_baseline(
                    line.as_str(),
                    Point::new(0, 24),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                line.clear();

                write!(&mut line, "SNR  {} dB", packet_status.snr).unwrap();
                Text::with_baseline(
                    line.as_str(),
                    Point::new(0, 36),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                line.clear();

                write!(
                    &mut line,
                    "RX {} MISS {}",
                    received_packets,
                    missed_packets
                )
                .unwrap();
                Text::with_baseline(
                    line.as_str(),
                    Point::new(0, 48),
                    text_style,
                    Baseline::Top,
                )
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

                write!(
                    &mut line,
                    "RX {} MISS {}",
                    received_packets,
                    missed_packets
                )
                .unwrap();
                Text::with_baseline(
                    line.as_str(),
                    Point::new(0, 48),
                    text_style,
                    Baseline::Top,
                )
                .draw(&mut display)
                .unwrap();

                display.flush().unwrap();

                link_lost_displayed = true;
            }
        }
    }
}
