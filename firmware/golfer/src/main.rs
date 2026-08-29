#![no_std]
#![no_main]

use defmt::{debug, error, info, warn};
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use panic_probe as _;

mod audio;
mod buttons;
mod display;
mod gps;
mod packet;
mod radio;
mod spi0_bus;
mod storage;
mod system;

use display::{
    Display, DisplayPage, GpsDisplayState, InitStatus, InitSubsystem, RadioDisplayState,
};
use storage::PersistentLogLevel;

// Once subsystem initialization has completed, leave the final boot status
// visible briefly so the user can actually read the result.
const BOOT_FINAL_STATUS_HOLD_MS: u64 = 1_000;

// The temporary native-packet nRF beacon increments its sequence once per second.
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

// -----------------------------------------------------------------------------
// TEMPORARY NATIVE-PACKET ACCEPTANCE CONTEXT
//
// Survey membership / peer authorization will eventually come from GOLFER's
// persistent survey state. During this protocol bring-up, the temporary MRU
// beacon transmits one fixed System ID and Survey ID. Keep this policy here in
// the application layer rather than radio.rs: the radio subsystem should remain
// capable of receiving arbitrary valid GOLFER traffic for future discovery,
// joining, multi-GOLFER operation, and other packet classes.
// -----------------------------------------------------------------------------

const TEST_EXPECTED_SENDER_SYSTEM_ID: u64 = 0x4D52_5500_0000_0001;
const TEST_EXPECTED_SURVEY_ID: u32 = 0xA000_0001;

// -----------------------------------------------------------------------------
// TEMPORARY BUTTON/AUDIO BRING-UP POLICY
//
// buttons.rs owns only the four physical inputs. audio.rs owns only PWM/audio.
// This task is deliberately application-level glue for today's hardware test:
// each held button requests a distinct continuous tone.
//
// If more than one button is held, the most recently pressed button wins. When
// that button is released, another still-held button resumes automatically.
// -----------------------------------------------------------------------------

const BUTTON_TEST_DRIVE_PERCENT: u8 = 10;

fn button_test_frequency_hz(button: buttons::Button) -> u32 {
    match button {
        buttons::Button::One => 330,
        buttons::Button::Two => 440,
        buttons::Button::Three => 550,
        buttons::Button::Four => 660,
    }
}

#[embassy_executor::task]
async fn button_audio_test_task() {
    let mut held = [false; 4];
    let mut active: Option<buttons::Button> = None;

    loop {
        let event = buttons::EVENTS.receive().await;
        let index = event.button.index();

        if event.pressed {
            held[index] = true;
            active = Some(event.button);

            audio::play_tone(
                button_test_frequency_hz(event.button),
                BUTTON_TEST_DRIVE_PERCENT,
            )
            .await;

            continue;
        }

        held[index] = false;

        // Releasing a non-active held button must not interrupt the tone of the
        // button that currently owns the speaker.
        if active != Some(event.button) {
            continue;
        }

        // If another button remains held, resume one of those tones instead of
        // going silent. Reverse order is only a deterministic tie-breaker; this
        // temporary test policy will eventually be replaced by real UI input
        // handling.
        let fallback = buttons::Button::ALL
            .iter()
            .rev()
            .copied()
            .find(|button| held[button.index()]);

        active = fallback;

        if let Some(button) = fallback {
            audio::play_tone(
                button_test_frequency_hz(button),
                BUTTON_TEST_DRIVE_PERCENT,
            )
            .await;
        } else {
            audio::stop().await;
        }
    }
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
    // Native GOLFER RF packet codec
    //
    // Keep the same on-device golden-vector self-test used during packet-format
    // bring-up. GOLFER now also uses this codec for live native TelemetryV1 RX,
    // so a self-test failure is a prominent indication that packet handling must
    // not be trusted.
    // -------------------------------------------------------------------------

    if packet::golden_self_test() {
        info!(
            "Native TelemetryV1 codec self-test OK: {} bytes",
            packet::TELEMETRY_V1_LEN
        );
    } else {
        error!("Native TelemetryV1 codec self-test FAILED");
    }

    // -------------------------------------------------------------------------
    // Shared SPI0 bus + immediate display initialization
    //
    // The user-facing display comes alive BEFORE optional/slow subsystem
    // initialization. This prevents SD-card behavior from producing a blank
    // startup experience.
    // -------------------------------------------------------------------------

    let spi0_bus = spi0_bus::init(
        p.SPI0, p.PIN_18, // SCK
        p.PIN_19, // MOSI
        p.PIN_16, // MISO
    );

    let tft_cs = Output::new(p.PIN_17, Level::High);
    let mut sd_cs = Output::new(p.PIN_22, Level::High);

    // Bring the TFT up at its normal runtime rate immediately.
    spi0_bus::set_frequency(spi0_bus, spi0_bus::RUN_FREQUENCY_HZ);

    let mut display = Display::new(
        spi0_bus,
        tft_cs,
        p.PIN_13, // TFT DC/RS
        p.PIN_14, // TFT RESET
        p.PIN_21, // TFT backlight
        system.info(),
        system.config().clone(),
    );

    info!("Boot initialization screen online");

    // -------------------------------------------------------------------------
    // Audio
    //
    // Audio is intentionally not represented as an initialization-screen
    // subsystem. The dedicated audio task owns PWM5 / GP26 and synthesized
    // sound sequencing. Important UI sounds preempt routine button/sonification
    // audio and the latest routine tone resumes afterward.
    // -------------------------------------------------------------------------

    spawner.spawn(
        audio::task(
            p.PWM_SLICE5,
            p.PIN_26,
        )
        .expect("failed to create audio task"),
    );

    audio::play_ui_sound(audio::UiSound::Startup).await;
    info!("Startup chime queued");

    // -------------------------------------------------------------------------
    // SD card / storage
    //
    // Visible state starts yellow ("initializing..."). We temporarily lower the
    // shared SPI bus to SD initialization speed, perform a cheap bounded CMD0
    // presence probe, and only invoke the full FAT stack if a card responds.
    //
    // Missing/bad storage is non-fatal: mark NOK and continue boot.
    // -------------------------------------------------------------------------

    display.set_init_status(InitSubsystem::SdCard, InitStatus::Initializing);

    spi0_bus::set_frequency(spi0_bus, spi0_bus::SD_INIT_FREQUENCY_HZ);

    let mut storage = if storage::card_present(spi0_bus, &mut sd_cs) {
        storage::Storage::init(spi0_bus, sd_cs)
    } else {
        None
    };

    // Return SPI0 to normal runtime speed before touching the TFT again.
    spi0_bus::set_frequency(spi0_bus, spi0_bus::RUN_FREQUENCY_HZ);

    if let Some(storage_ref) = storage.as_mut() {
        display.set_init_status(InitSubsystem::SdCard, InitStatus::Ok);

        // DEBUG.LOG only becomes available once SD/FAT initialization has
        // succeeded, so record the already-completed early boot stages now.
        let boot_ms = Instant::now().as_millis();

        storage_ref.diag(
            PersistentLogLevel::Info,
            boot_ms,
            "BOOT",
            "SYSTEM_INIT",
            format_args!("result=OK"),
        );

        storage_ref.diag(
            PersistentLogLevel::Info,
            boot_ms,
            "BOOT",
            "DISPLAY_INIT",
            format_args!("result=OK"),
        );

        storage_ref.diag(
            PersistentLogLevel::Info,
            boot_ms,
            "BOOT",
            "SD_CARD_INIT",
            format_args!("result=OK"),
        );

        info!(
            "Receiver telemetry logging enabled: segment={}",
            storage_ref.segment_number()
        );
    } else {
        display.set_init_status(InitSubsystem::SdCard, InitStatus::Nok);

        warn!("Storage unavailable; GOLFER continuing without SD logging");
    }

    // -------------------------------------------------------------------------
    // GPS
    //
    // gps.rs listens to PA1616S TX on GP1 / UART0 RX, keeps raw NMEA logging,
    // parses GGA into a latest GpsState, and publishes that state through an
    // Embassy Signal. GP0 remains reserved for Pico -> GPS TX later.
    // -------------------------------------------------------------------------

    display.set_init_status(InitSubsystem::Gps, InitStatus::Initializing);

    spawner.spawn(
        gps::receive_task(
            p.UART0, p.PIN_1, // GPS TX -> Pico UART0 RX
        )
        .expect("failed to create GPS receive task"),
    );

    // "OK" here means the GPS UART/task is online. It does NOT mean GPS fix.
    display.set_init_status(InitSubsystem::Gps, InitStatus::Ok);

    if let Some(storage_ref) = storage.as_mut() {
        storage_ref.diag(
            PersistentLogLevel::Info,
            Instant::now().as_millis(),
            "BOOT",
            "GPS_INIT",
            format_args!("result=OK"),
        );
    }

    // -------------------------------------------------------------------------
    // LoRa radio
    //
    // All SX1262 hardware setup and the proven SF7 RX configuration now live
    // in radio.rs. main.rs only supplies the peripherals and owns application
    // behavior around received packets.
    // -------------------------------------------------------------------------

    display.set_init_status(InitSubsystem::Lora, InitStatus::Initializing);

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
        Ok(radio) => {
            display.set_init_status(InitSubsystem::Lora, InitStatus::Ok);

            if let Some(storage_ref) = storage.as_mut() {
                storage_ref.diag(
                    PersistentLogLevel::Info,
                    Instant::now().as_millis(),
                    "BOOT",
                    "LORA_INIT",
                    format_args!("result=OK"),
                );
            }

            radio
        }

        Err(err) => {
            display.set_init_status(InitSubsystem::Lora, InitStatus::Nok);

            if let Some(storage_ref) = storage.as_mut() {
                storage_ref.diag(
                    PersistentLogLevel::Error,
                    Instant::now().as_millis(),
                    "BOOT",
                    "LORA_INIT",
                    format_args!("result=NOK error={:?}", err),
                );
            }

            error!("Radio initialization aborted: {}", err);

            // Ensure no routine tone remains requested if boot aborts here.
            audio::stop().await;

            loop {
                led.set_high();
                Timer::after_millis(100).await;

                led.set_low();
                Timer::after_millis(100).await;
            }
        }
    };

    if let Some(storage_ref) = storage.as_mut() {
        storage_ref.diag(
            PersistentLogLevel::Info,
            Instant::now().as_millis(),
            "BOOT",
            "INIT_COMPLETE",
            format_args!("result=OK"),
        );
    }

    info!(
        "Initialization complete; holding final status for {} ms",
        BOOT_FINAL_STATUS_HOLD_MS
    );

    Timer::after_millis(BOOT_FINAL_STATUS_HOLD_MS).await;

    // -------------------------------------------------------------------------
    // Buttons
    //
    // Buttons are intentionally not represented on the initialization screen.
    // Their hardware task owns GP4-GP7 and publishes debounced active-low state
    // changes. A temporary application task turns those events into four held
    // test tones for this bring-up stage.
    // -------------------------------------------------------------------------

    spawner.spawn(
        buttons::task(
            p.PIN_4,
            p.PIN_5,
            p.PIN_6,
            p.PIN_7,
        )
        .expect("failed to create button task"),
    );

    spawner.spawn(
        button_audio_test_task()
            .expect("failed to create button audio test task"),
    );

    display.set_page(DisplayPage::General);
    info!("Boot initialization complete; entering general display");

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

    // Only a fully decoded, application-CRC-valid native TelemetryV1 packet
    // from the expected survey context keeps the RF link alive.
    let mut last_valid_rx_time: Option<Instant> = None;

    // RX means accepted native survey telemetry, not merely an SX1262 RxDone.
    // This is intentionally separate from application CRC failures and other
    // malformed/foreign frames leaked upward by the current PHY stack.
    let mut received_packets: u32 = 0;
    let mut missed_packets: u32 = 0;
    let mut crc_failures: u32 = 0;
    let mut invalid_native_packets: u32 = 0;
    let mut foreign_context_packets: u32 = 0;

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

                let now = Instant::now();

                // -------------------------------------------------------------
                // Native TelemetryV1 validation boundary
                //
                // Nothing below this point is allowed to affect GOLFER's valid
                // RX count, link-alive timer, sequence state, telemetry log, or
                // UI unless the application-level packet CRC and field decoder
                // have both succeeded. This is deliberate defense-in-depth for
                // the current lora-phy 3.0.1 behavior that can surface an
                // SX1262 CRC-failed payload as a successful receive.
                // -------------------------------------------------------------

                let telemetry = match packet::TelemetryV1::decode(&packet.data[..len]) {
                    Ok(telemetry) => telemetry,

                    Err(packet::DecodeError::CrcMismatch) => {
                        crc_failures = crc_failures.saturating_add(1);

                        if let Some((stored_crc, computed_crc)) =
                            packet::crc32_details(&packet.data[..len])
                        {
                            warn!(
                                "Dropping TelemetryV1 CRC failure: count={} rssi={} snr={} stored_crc={} computed_crc={}",
                                crc_failures,
                                packet_status.rssi,
                                packet_status.snr,
                                stored_crc,
                                computed_crc
                            );

                            if let Some(storage) = storage.as_mut() {
                                storage.diag(
                                    PersistentLogLevel::Debug,
                                    now.as_millis(),
                                    "RADIO",
                                    "APP_CRC_FAILED",
                                    format_args!(
                                        "count={} rssi={} snr={} stored_crc={} computed_crc={}",
                                        crc_failures,
                                        packet_status.rssi,
                                        packet_status.snr,
                                        stored_crc,
                                        computed_crc
                                    ),
                                );
                            }
                        } else {
                            warn!(
                                "Dropping TelemetryV1 CRC failure: count={} rssi={} snr={}",
                                crc_failures,
                                packet_status.rssi,
                                packet_status.snr
                            );
                        }

                        continue;
                    }

                    Err(err) => {
                        invalid_native_packets =
                            invalid_native_packets.saturating_add(1);

                        warn!(
                            "Ignoring invalid/unknown native packet: len={} error={} count={}",
                            received_len,
                            err.label(),
                            invalid_native_packets
                        );

                        if let Some(storage) = storage.as_mut() {
                            storage.diag(
                                PersistentLogLevel::Debug,
                                now.as_millis(),
                                "RADIO",
                                "NATIVE_PACKET_REJECTED",
                                format_args!(
                                    "len={} error={} count={} rssi={} snr={}",
                                    received_len,
                                    err.label(),
                                    invalid_native_packets,
                                    packet_status.rssi,
                                    packet_status.snr
                                ),
                            );
                        }

                        continue;
                    }
                };

                // Temporary survey/peer policy for the MRU acceptance beacon.
                // A valid GOLFER packet from some other context is not corrupt;
                // it simply does not belong to this active test survey. Future
                // application routing can branch discovery/join/control packets
                // elsewhere rather than discarding them here.
                if telemetry.sender_system_id != TEST_EXPECTED_SENDER_SYSTEM_ID
                    || telemetry.survey_id != TEST_EXPECTED_SURVEY_ID
                {
                    foreign_context_packets =
                        foreign_context_packets.saturating_add(1);

                    warn!(
                        "Ignoring foreign TelemetryV1: sender={} survey={} seq={} count={}",
                        telemetry.sender_system_id,
                        telemetry.survey_id,
                        telemetry.sequence,
                        foreign_context_packets
                    );

                    if let Some(storage) = storage.as_mut() {
                        storage.diag(
                            PersistentLogLevel::Debug,
                            now.as_millis(),
                            "RADIO",
                            "SURVEY_CONTEXT_REJECTED",
                            format_args!(
                                "sender={} survey={} seq={} mode={} count={}",
                                telemetry.sender_system_id,
                                telemetry.survey_id,
                                telemetry.sequence,
                                telemetry.sender_mode,
                                foreign_context_packets
                            ),
                        );
                    }

                    continue;
                }

                let sequence = telemetry.sequence;

                debug!(
                    "RX TelemetryV1 fields: sender={} survey={} mode={} seq={} utc={:?} tx_lat={:?} tx_lon={:?} alt_half_m={:?} speed_cm_s={:?} course_cdeg={:?} fix={} sats={} hdop_tenths={:?} temp_centi_c={:?} pressure_10pa={:?} humidity_half_pct={:?} battery_soc={:?}",
                    telemetry.sender_system_id,
                    telemetry.survey_id,
                    telemetry.sender_mode,
                    telemetry.sequence,
                    telemetry.gps_unix_time,
                    telemetry.latitude_e7,
                    telemetry.longitude_e7,
                    telemetry.altitude_half_m,
                    telemetry.speed_cm_s,
                    telemetry.course_cdeg,
                    telemetry.gps_fix_class as u8,
                    telemetry.satellites,
                    telemetry.hdop_tenths,
                    telemetry.temperature_centi_c,
                    telemetry.pressure_10pa,
                    telemetry.humidity_half_percent,
                    telemetry.battery_soc_percent
                );

                // -------------------------------------------------------------
                // Sequence validation / packet-loss accounting
                //
                // IMPORTANT:
                //
                // Time-vs-sequence plausibility is no longer an acceptance gate.
                // Application CRC has already proven packet integrity here; the
                // probation logic remains as defense-in-depth and prevents an
                // unexpected sequence transition from poisoning bookkeeping while
                // still allowing legitimate long-outage reacquisition to confirm
                // itself with the very next packet.
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
                                    sequence, packet_status.rssi, packet_status.snr
                                ),
                            );
                        }

                        continue;
                    }

                    // If last packet was a suspicious jump and this packet is
                    // exactly its successor, we have strong evidence that the
                    // jump was real. Resynchronize immediately.
                    if let Some(candidate) = pending_sequence_candidate {
                        if candidate.checked_add(1) == Some(sequence) && candidate > last {
                            let newly_missed = candidate.saturating_sub(last).saturating_sub(1);

                            if newly_missed > 0 {
                                missed_packets = missed_packets.saturating_add(newly_missed);
                            }

                            warn!(
                                "Sequence resync confirmed: last={} candidate={} current={} missed={}",
                                last, candidate, sequence, newly_missed
                            );

                            if let Some(storage) = storage.as_mut() {
                                storage.diag(
                                    PersistentLogLevel::Info,
                                    now.as_millis(),
                                    "RADIO",
                                    "SEQ_RESYNC_CONFIRMED",
                                    format_args!(
                                        "prev={} candidate={} current={} missed={}",
                                        last, candidate, sequence, newly_missed
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
                                        candidate, sequence, last
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
                                .map(|last_time| now.duration_since(last_time).as_millis())
                                .unwrap_or(0);

                            let expected_gap =
                                elapsed_ms.saturating_mul(BEACON_SEQUENCE_RATE_HZ) / 1_000;

                            let max_plausible_gap =
                                expected_gap.saturating_add(SEQUENCE_GAP_TOLERANCE).max(1);

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
                                            "prev={} current={} gap={} elapsed_ms={} expected={} max={} sender={} survey={} mode={} rssi={} snr={} app_crc=OK",
                                            last,
                                            sequence,
                                            sequence_gap,
                                            elapsed_ms,
                                            expected_gap,
                                            max_plausible_gap,
                                            telemetry.sender_system_id,
                                            telemetry.survey_id,
                                            telemetry.sender_mode,
                                            packet_status.rssi,
                                            packet_status.snr,
                                        ),
                                    );
                                }

                                pending_sequence_candidate = Some(sequence);
                                trust_sequence_now = false;
                            } else {
                                let newly_missed = sequence_gap - 1;

                                if newly_missed > 0 {
                                    missed_packets = missed_packets.saturating_add(newly_missed);

                                    warn!(
                                        "PACKET LOSS: missed {} packet(s) between seq={} and seq={}",
                                        newly_missed, last, sequence
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
                                    last, sequence
                                );

                                if let Some(storage) = storage.as_mut() {
                                    storage.diag(
                                        PersistentLogLevel::Info,
                                        now.as_millis(),
                                        "RADIO",
                                        "SEQ_RESET",
                                        format_args!("prev={} current={}", last, sequence),
                                    );
                                }

                                pending_sequence_candidate = None;
                            } else {
                                warn!(
                                    "Ignoring backward/out-of-order sequence: last={} current={}",
                                    last, sequence
                                );

                                if let Some(storage) = storage.as_mut() {
                                    storage.diag(
                                        PersistentLogLevel::Warn,
                                        now.as_millis(),
                                        "RADIO",
                                        "SEQ_BACKWARD",
                                        format_args!(
                                            "prev={} current={} rssi={} snr={}",
                                            last, sequence, packet_status.rssi, packet_status.snr
                                        ),
                                    );
                                }

                                continue;
                            }
                        }
                    }
                }

                // The native packet itself is accepted. A suspicious but
                // application-CRC-valid sequence can keep the link alive and be
                // logged without immediately becoming the trusted sequence
                // baseline. Invalid/CRC-failed/foreign packets never reach here.
                received_packets = received_packets.saturating_add(1);

                if trust_sequence_now {
                    last_sequence = Some(sequence);
                    last_sequence_rx_time = Some(now);
                }

                let link_was_lost = link_lost_displayed;

                last_valid_rx_time = Some(now);
                last_rssi = Some(packet_status.rssi);
                last_snr = Some(packet_status.snr);
                link_lost_displayed = false;

                info!(
                    "RX TelemetryV1 seq={} sender={} survey={} RSSI={} dBm SNR={} dB | received={} missed={} app_crc_fail={}",
                    sequence,
                    telemetry.sender_system_id,
                    telemetry.survey_id,
                    packet_status.rssi,
                    packet_status.snr,
                    received_packets,
                    missed_packets,
                    crc_failures
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

                if link_was_lost {
                    audio::play_ui_sound(audio::UiSound::LinkReacquired).await;
                    info!("Link reacquisition sound queued");
                }

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
                audio::play_ui_sound(audio::UiSound::LinkLost).await;
                info!("Link-loss sound queued");
            }
        }
    }
}
