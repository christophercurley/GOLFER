#![no_std]
#![no_main]

use defmt::{error, info, warn};
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use panic_probe as _;

mod display;
mod gps;
mod radio;
mod system;

use display::{Display, DisplayPage, GpsDisplayState, RadioDisplayState};

// How long the startup system-information page remains visible.
// This is intentionally generous for bring-up and will be shortened later.
const BOOT_SCREEN_DURATION_SECS: u64 = 10;

// The current nRF beacon increments its sequence once per second.
const BEACON_SEQUENCE_RATE_HZ: u64 = 1;

// If no packet arrives for this long, show LINK LOST on the OLED.
const LINK_LOSS_TIMEOUT_SECS: u64 = 5;

// The application wakes periodically to evaluate link age. This timeout is
// applied only to the Embassy channel, never to the SX1262 receive future.
const LINK_WATCHDOG_INTERVAL_MS: u64 = 250;

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
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Establish this physical GOLFER's immutable hardware-derived identity.
    // The resulting SystemInfo is also the data source for the boot screen.
    let system = system::init(p.FLASH);

    // Pico 2 onboard LED.
    // Pulses briefly whenever a valid packet is accepted.
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("LORAMv1 online");

    // -------------------------------------------------------------------------
    // Display
    //
    // ILI9341 TFT on SPI0, native portrait orientation (240 x 320).
    // GP16 = MISO
    // GP17 = TFT CS
    // GP18 = SCK
    // GP19 = MOSI
    // GP13 = DC/RS
    // GP14 = RESET
    // GP21 = backlight enable (PWM later)
    // -------------------------------------------------------------------------

    let mut display = Display::new(
        p.SPI0, p.PIN_18, // SCK
        p.PIN_19, // MOSI
        p.PIN_16, // MISO
        p.PIN_17, // TFT CS
        p.PIN_13, // TFT DC/RS
        p.PIN_14, // TFT RESET
        p.PIN_21, // TFT backlight
        system.info(),
        system.config().clone(),
    );

    info!(
        "Boot screen active for {} seconds",
        BOOT_SCREEN_DURATION_SECS
    );

    Timer::after_secs(BOOT_SCREEN_DURATION_SECS).await;

    display.set_page(DisplayPage::General);
    info!("Boot screen complete; entering general display");

    // -------------------------------------------------------------------------
    // GPS
    //
    // gps.rs listens to PA1616S TX on GP1 / UART0 RX, keeps raw NMEA logging,
    // parses GGA into a latest GpsState, and publishes that state through an
    // Embassy Signal. GP0 remains reserved for Pico -> GPS TX later.
    // -------------------------------------------------------------------------

    spawner.spawn(
        gps::receive_task(
            p.UART0, p.PIN_1, // GPS TX -> Pico UART0 RX
        )
        .expect("failed to create GPS receive task"),
    );

    // -------------------------------------------------------------------------
    // LoRa radio
    //
    // All SX1262 hardware setup and the proven SF7 RX configuration now live
    // in radio.rs. main.rs only supplies the peripherals and owns application
    // behavior around received packets.
    // -------------------------------------------------------------------------

    let radio = match radio::Radio::new(
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

    // The SX1262 receive future now lives permanently inside its own task.
    // The application only waits on RX_CHANNEL, which is safe to timeout.
    spawner.spawn(radio::receive_task(radio).expect("failed to create radio receive task"));

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
        // ---------------------------------------------------------------------
        // Consume the newest GPS state, if one has arrived.
        //
        // GPS_STATE_SIGNAL intentionally stores only the latest state. The
        // display does not need to render every intermediate 1 Hz update if the
        // application was briefly busy with a radio packet.
        // ---------------------------------------------------------------------

        if let Some(gps_state) = gps::GPS_STATE_SIGNAL.try_take() {
            display.update_gps(GpsDisplayState {
                online: gps_state.online,
                fix: gps_state.fix,
                latitude_e7: gps_state.latitude_e7,
                longitude_e7: gps_state.longitude_e7,
                satellites: gps_state.satellites,
            });
        }

        let rx_result = with_timeout(
            Duration::from_millis(LINK_WATCHDOG_INTERVAL_MS),
            radio::RX_CHANNEL.receive(),
        )
        .await;

        match rx_result {
            // -----------------------------------------------------------------
            // Dedicated radio task delivered a packet.
            // -----------------------------------------------------------------
            Ok(packet) => {
                let received_len = packet.len;
                let len = received_len as usize;
                let packet_status = packet.status;

                let Some(sequence) = decode_sequence(&packet.data[..len]) else {
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
                // Update display state.
                // -------------------------------------------------------------

                display.update_radio(RadioDisplayState::connected(
                    sequence,
                    packet_status.rssi,
                    packet_status.snr,
                    received_packets,
                    missed_packets,
                ));

                // Brief visible indication of an accepted packet.
                led.set_high();
                Timer::after_millis(75).await;
                led.set_low();
            }

            // -----------------------------------------------------------------
            // Watchdog tick: no packet arrived on the application channel during
            // this short interval. The SX1262 task is still receiving normally.
            // -----------------------------------------------------------------
            Err(_) => {
                // Before the first valid packet, keep the startup
                // "Waiting for RX..." screen rather than calling it a lost link.
                let Some(last_rx_time) = last_valid_rx_time else {
                    continue;
                };

                let link_age_secs = Instant::now().duration_since(last_rx_time).as_secs();

                if link_age_secs < LINK_LOSS_TIMEOUT_SECS {
                    continue;
                }

                // Only redraw once per outage. When a valid packet returns,
                // link_lost_displayed is cleared and the live screen returns.
                if link_lost_displayed {
                    continue;
                }

                warn!("LINK LOST: no valid packet for {} seconds", link_age_secs);

                display.update_radio(RadioDisplayState::lost(
                    last_sequence,
                    last_rssi,
                    last_snr,
                    received_packets,
                    missed_packets,
                ));

                link_lost_displayed = true;
            }
        }
    }
}
