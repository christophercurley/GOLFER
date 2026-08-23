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
mod spi0_bus;
mod storage;
mod system;

use display::{Display, DisplayPage, GpsDisplayState, RadioDisplayState};
use storage::PersistentLogLevel;

// How long the startup system-information page remains visible.
// This is intentionally generous for bring-up and will be shortened later.
const BOOT_SCREEN_DURATION_SECS: u64 = 2;

// The current nRF beacon increments its sequence once per second.
const BEACON_SEQUENCE_RATE_HZ: u64 = 1;

// If no packet arrives for this long, show LINK LOST on the OLED.
const LINK_LOSS_TIMEOUT_SECS: u64 = 5;

// The application wakes periodically to evaluate link age. This timeout is
// applied only to the Embassy channel, never to the SX1262 receive future.
const LINK_WATCHDOG_INTERVAL_MS: u64 = 250;

// Sequence/time comparison is now DIAGNOSTIC ONLY.
//
// A suspicious forward jump is never permanently rejected. Instead, it becomes
// an untrusted candidate. If the next packet is candidate + 1, the receiver
// confirms the new baseline and resynchronizes. If the next packet returns to a
// sane sequence, the candidate is discarded without poisoning sequence state.
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

    info!("GOLFER firmware is online!");

    // -------------------------------------------------------------------------
    // Shared SPI0 bus + storage
    //
    // GP16 = MISO
    // GP18 = SCK
    // GP19 = MOSI
    //
    // GP17 = TFT CS
    // GP22 = SD CS
    //
    // SPI0 begins at the SD-card initialization speed. Both chip-selects are
    // driven HIGH before the card receives its startup clocks.
    // -------------------------------------------------------------------------

    let spi0_bus = spi0_bus::init(
        p.SPI0,
        p.PIN_18, // SCK
        p.PIN_19, // MOSI
        p.PIN_16, // MISO
    );

    let tft_cs = Output::new(p.PIN_17, Level::High);
    let sd_cs = Output::new(p.PIN_22, Level::High);

    // Storage failure is intentionally non-fatal. If available, Storage keeps
    // tonight's hardcoded receiver telemetry segment open for this entire boot.
    let mut storage = storage::Storage::init(
        spi0_bus,
        sd_cs,
    );

    if let Some(storage) = storage.as_ref() {
        info!(
            "Receiver telemetry logging enabled: segment={}",
            storage.segment_number()
        );
    } else {
        warn!("Receiver telemetry logging unavailable");
    }

    // The card has completed its low-speed initialization attempt. SPI0 can now
    // run at the normal TFT/runtime clock. The SD card shares this bus.
    spi0_bus::set_frequency(
        spi0_bus,
        spi0_bus::RUN_FREQUENCY_HZ,
    );

    // -------------------------------------------------------------------------
    // Display
    //
    // ILI9341 TFT on the shared SPI0 bus, native portrait orientation
    // (240 x 320).
    //
    // GP17 = TFT CS
    // GP13 = DC/RS
    // GP14 = RESET
    // GP21 = backlight enable (physically bypassed on current prototype)
    // -------------------------------------------------------------------------

    let mut display = Display::new(
        spi0_bus,
        tft_cs,
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

    // Retain the newest GPS state so every accepted radio packet can snapshot
    // local position into its telemetry record.
    let mut latest_gps_state = gps::GpsState::offline();

    let mut last_sequence: Option<u32> = None;

    // Time associated with last_sequence specifically. This must remain
    // separate from last_valid_rx_time because a suspicious-but-received packet
    // should keep the link alive without immediately becoming sequence truth.
    let mut last_sequence_rx_time: Option<Instant> = None;

    // A wild forward jump gets one packet of probation rather than being
    // rejected forever. candidate + 1 on the next packet confirms resync.
    let mut pending_sequence_candidate: Option<u32> = None;

    // Any structurally-valid packet keeps the RF link alive.
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
            latest_gps_state = gps_state;

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
                // IMPORTANT:
                //
                // Time-vs-sequence plausibility is no longer an acceptance gate.
                // A large mismatch gets one packet of probation so a corrupted
                // sequence cannot poison state, but legitimate long-outage
                // reacquisition can confirm itself with the very next packet.
                // -------------------------------------------------------------

                let mut trust_sequence_now = true;
                let mut sequence_already_confirmed = false;

                if let Some(last) = last_sequence {
                    // Repeated suspicious candidate: do not let it become a new
                    // baseline merely because it arrived twice identically.
                    if pending_sequence_candidate == Some(sequence) {
                        warn!(
                            "Ignoring duplicate suspicious sequence candidate: seq={}",
                            sequence
                        );

                        if let Some(storage) = storage.as_mut() {
                            storage.diag(
                                PersistentLogLevel::Warn,
                                now.as_millis(),
                                "RADIO",
                                "SEQ_CANDIDATE_DUPLICATE",
                                format_args!(
                                    "seq={} rssi={} snr={}",
                                    sequence,
                                    packet_status.rssi,
                                    packet_status.snr
                                ),
                            );
                        }

                        continue;
                    }

                    // If last packet was a suspicious jump and this packet is
                    // exactly its successor, we have strong evidence that the
                    // jump was real. Resynchronize immediately.
                    if let Some(candidate) = pending_sequence_candidate {
                        if candidate.checked_add(1) == Some(sequence)
                            && candidate > last
                        {
                            let newly_missed =
                                candidate.saturating_sub(last).saturating_sub(1);

                            if newly_missed > 0 {
                                missed_packets =
                                    missed_packets.saturating_add(newly_missed);
                            }

                            warn!(
                                "Sequence resync confirmed: last={} candidate={} current={} missed={}",
                                last,
                                candidate,
                                sequence,
                                newly_missed
                            );

                            if let Some(storage) = storage.as_mut() {
                                storage.diag(
                                    PersistentLogLevel::Info,
                                    now.as_millis(),
                                    "RADIO",
                                    "SEQ_RESYNC_CONFIRMED",
                                    format_args!(
                                        "prev={} candidate={} current={} missed={}",
                                        last,
                                        candidate,
                                        sequence,
                                        newly_missed
                                    ),
                                );
                            }

                            pending_sequence_candidate = None;
                            sequence_already_confirmed = true;
                        } else {
                            // Candidate failed confirmation. Do not poison the
                            // trusted baseline; evaluate this packet normally
                            // against the last trusted sequence.
                            if let Some(storage) = storage.as_mut() {
                                storage.diag(
                                    PersistentLogLevel::Debug,
                                    now.as_millis(),
                                    "RADIO",
                                    "SEQ_CANDIDATE_NOT_CONFIRMED",
                                    format_args!(
                                        "candidate={} next={} trusted_prev={}",
                                        candidate,
                                        sequence,
                                        last
                                    ),
                                );
                            }

                            pending_sequence_candidate = None;
                        }
                    }

                    if !sequence_already_confirmed {
                        if sequence == last {
                            warn!("Ignoring duplicate packet: seq={}", sequence);
                            continue;
                        }

                        if sequence > last {
                            let sequence_gap = sequence - last;

                            let elapsed_ms = last_sequence_rx_time
                                .map(|last_time| {
                                    now.duration_since(last_time).as_millis()
                                })
                                .unwrap_or(0);

                            let expected_gap = elapsed_ms
                                .saturating_mul(BEACON_SEQUENCE_RATE_HZ)
                                / 1_000;

                            let max_plausible_gap = expected_gap
                                .saturating_add(SEQUENCE_GAP_TOLERANCE)
                                .max(1);

                            if u64::from(sequence_gap) > max_plausible_gap {
                                // Do NOT reject the RF packet and do NOT move the
                                // trusted sequence baseline. Put the sequence on
                                // one-packet probation instead.
                                warn!(
                                    "Suspicious sequence candidate: last={} current={} gap={} elapsed_ms={} expected_gap={} max_gap={}",
                                    last,
                                    sequence,
                                    sequence_gap,
                                    elapsed_ms,
                                    expected_gap,
                                    max_plausible_gap
                                );

                                if let Some(storage) = storage.as_mut() {
                                    storage.diag(
                                        PersistentLogLevel::Warn,
                                        now.as_millis(),
                                        "RADIO",
                                        "SEQ_SUSPICIOUS",
                                        format_args!(
                                            "prev={} current={} gap={} elapsed_ms={} expected={} max={} rssi={} snr={} raw={:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}",
                                            last,
                                            sequence,
                                            sequence_gap,
                                            elapsed_ms,
                                            expected_gap,
                                            max_plausible_gap,
                                            packet_status.rssi,
                                            packet_status.snr,
                                            packet.data[0],
                                            packet.data[1],
                                            packet.data[2],
                                            packet.data[3],
                                            packet.data[4],
                                            packet.data[5],
                                            packet.data[6],
                                            packet.data[7],
                                            packet.data[8],
                                            packet.data[9],
                                        ),
                                    );
                                }

                                pending_sequence_candidate = Some(sequence);
                                trust_sequence_now = false;
                            } else {
                                let newly_missed = sequence_gap - 1;

                                if newly_missed > 0 {
                                    missed_packets =
                                        missed_packets.saturating_add(newly_missed);

                                    warn!(
                                        "PACKET LOSS: missed {} packet(s) between seq={} and seq={}",
                                        newly_missed,
                                        last,
                                        sequence
                                    );
                                }
                            }
                        } else {
                            // A real transmitter reboot should restart the
                            // sequence close to zero. Other backwards packets are
                            // ignored, but this cannot create a sticky lockout.
                            if sequence <= REBOOT_SEQUENCE_WINDOW {
                                warn!(
                                    "Beacon sequence reset detected: last={} current={}",
                                    last,
                                    sequence
                                );

                                if let Some(storage) = storage.as_mut() {
                                    storage.diag(
                                        PersistentLogLevel::Info,
                                        now.as_millis(),
                                        "RADIO",
                                        "SEQ_RESET",
                                        format_args!(
                                            "prev={} current={}",
                                            last,
                                            sequence
                                        ),
                                    );
                                }

                                pending_sequence_candidate = None;
                            } else {
                                warn!(
                                    "Ignoring backward/out-of-order sequence: last={} current={}",
                                    last,
                                    sequence
                                );

                                if let Some(storage) = storage.as_mut() {
                                    storage.diag(
                                        PersistentLogLevel::Warn,
                                        now.as_millis(),
                                        "RADIO",
                                        "SEQ_BACKWARD",
                                        format_args!(
                                            "prev={} current={} rssi={} snr={}",
                                            last,
                                            sequence,
                                            packet_status.rssi,
                                            packet_status.snr
                                        ),
                                    );
                                }

                                continue;
                            }
                        }
                    }
                }

                // The RF packet itself is accepted. A suspicious sequence can
                // keep the link alive and be logged without immediately becoming
                // the trusted sequence baseline.
                received_packets = received_packets.saturating_add(1);

                if trust_sequence_now {
                    last_sequence = Some(sequence);
                    last_sequence_rx_time = Some(now);
                }

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
                // TEMPORARY RECEIVER TELEMETRY LOGGING TEST
                //
                // One synchronous FAT append for every accepted LoRa packet.
                // The file remains open between packets. We deliberately do NOT
                // flush each record; tonight we care about raw append latency
                // and whether it interferes with continuous reception.
                // -------------------------------------------------------------

                if let Some(storage) = storage.as_mut() {
                    let write_result = storage.log_receiver_packet(
                        now.as_millis(),
                        sequence,
                        packet_status.rssi,
                        packet_status.snr,
                        received_packets,
                        missed_packets,
                        latest_gps_state,
                    );

                    match write_result {
                        Some(stats) => {
                            if let Some(checkpoint_us) = stats.checkpoint_us {
                                info!(
                                    "Telemetry append + checkpoint: {} us (append={} us checkpoint={} us) | GPS online={} fix={} sats={:?}",
                                    stats.total_us,
                                    stats.append_us,
                                    checkpoint_us,
                                    latest_gps_state.online,
                                    latest_gps_state.fix,
                                    latest_gps_state.satellites
                                );
                            } else {
                                info!(
                                    "Telemetry append: {} us | GPS online={} fix={} sats={:?}",
                                    stats.append_us,
                                    latest_gps_state.online,
                                    latest_gps_state.fix,
                                    latest_gps_state.satellites
                                );
                            }
                        }

                        None => {
                            warn!("Telemetry write FAILED");
                        }
                    }
                }

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
