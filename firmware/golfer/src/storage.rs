use core::{
    cell::RefCell,
    fmt::{Arguments, Write as _},
};

use defmt::{error, info, warn};

use embassy_rp::gpio::Output;
use embassy_time::{Delay, Instant};

use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::{
    Mode,
    RawDirectory,
    RawFile,
    RawVolume,
    SdCard,
    ShortFileName,
    TimeSource,
    Timestamp,
    VolumeIdx,
    VolumeManager,
};
use heapless::String;

use crate::{
    gps::GpsState,
    packet::TelemetryV1,
    spi0_bus::{self, Spi0Bus},
    system::SystemInfo,
};

const SYSTEM_LOG_FILENAME: &str = "SYS_LOG.TXT";
const DEBUG_LOG_FILENAME: &str = "DEBUG.LOG";
const RX_LOG_FILENAME: &str = "RX.LOG";
const LOCAL_LOG_FILENAME: &str = "LOCAL.LOG";
const EVENT_LOG_FILENAME: &str = "EVENT.LOG";
const META_FILENAME: &str = "META.TXT";
const BOOT_COUNTER_FILENAME: &str = "BOOT.NXT";

/// Logging schema introduced by the per-survey/per-boot directory layout.
pub const LOG_SCHEMA_VERSION: u16 = 3;

const SD_CMD0: [u8; 6] = [
    0x40, // CMD0 / GO_IDLE_STATE
    0x00,
    0x00,
    0x00,
    0x00,
    0x95, // valid CMD0 CRC
];
const SD_PRESENCE_ATTEMPTS: usize = 8;

// -----------------------------------------------------------------------------
// PERSISTENT DIAGNOSTIC LOGGING
//
// Current DEBUG.LOG wire/text envelope:
//
//   <timestamp> <time_source> <uptime_ms> <level> <originator> <event_id> ...
//
// Before GPS UTC is available, the envelope begins with `NA UNKNOWN`. Once
// RMC provides UTC, it becomes `<unix_ms> GPS <mono_ms> ...`. Monotonic
// time remains authoritative and is always present.
// -----------------------------------------------------------------------------

/// Persistent DEBUG.LOG threshold.
///
/// Ordering follows the traditional logging model:
///
/// ERROR < WARN < INFO < DEBUG < TRACE
///
/// A configured level includes that level and every more-severe level.
///
/// TRACE is intentionally enabled during current field bring-up so filesystem
/// append timing is captured. This can be lowered later without touching call
/// sites.
pub const PERSISTENT_LOG_LEVEL: PersistentLogLevel =
    PersistentLogLevel::Trace;

const DEBUG_BUFFER_CAPACITY: usize = 2048;
const DEBUG_LINE_CAPACITY: usize = 320;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum PersistentLogLevel {
    Error = 1,
    Warn = 2,
    Info = 3,
    Debug = 4,
    Trace = 5,
}

impl PersistentLogLevel {
    const fn label(self) -> &'static str {
        match self {
            Self::Error => "ERROR",
            Self::Warn => "WARN",
            Self::Info => "INFO",
            Self::Debug => "DEBUG",
            Self::Trace => "TRACE",
        }
    }

    const fn forces_persistent_flush(self) -> bool {
        matches!(self, Self::Error | Self::Warn)
    }
}

pub struct TelemetryWriteStats {
    pub append_us: u64,

    /// Time spent in the explicit FAT checkpoint/flush operation.
    pub checkpoint_us: Option<u64>,

    /// append_us + checkpoint_us when a checkpoint occurred.
    pub total_us: u64,
}

// -----------------------------------------------------------------------------
// TEMPORARY SURVEY-LOGGING STUB
//
// This exists specifically to prove sustained 1 Hz telemetry logging while
// GOLFER continues receiving LoRa packets.
//
// It is NOT the final survey/session model.
// -----------------------------------------------------------------------------

const SURVEY_ROOT_DIR: &str = "SURVEY";

/// Temporary active survey for current bring-up. Bumping this from A0000001
/// creates a clean historical boundary between the legacy flat logs and
/// Logging V2's per-boot/session directory layout.
pub const TEST_SURVEY_ID: u32 = 0xA000_0003;

const BOOT_SESSION_PREFIX: u8 = b'B';
const MAX_BOOT_SESSION: u32 = 9_999_999;

type SdSpiDevice =
    RefCellDevice<'static, Spi0Bus, Output<'static>, Delay>;

type SdCardImpl =
    SdCard<SdSpiDevice, Delay>;

type VolumeManagerImpl =
    VolumeManager<SdCardImpl, PlaceholderTimeSource>;

/// Basic GOLFER storage façade.
///
/// SYS_LOG, survey telemetry, and DEBUG.LOG are deliberately separate products.
///
/// DEBUG/TRACE diagnostic messages are RAM-buffered and written in batches so
/// enabling field diagnostics does not manufacture an extra FAT operation for
/// every normal packet. WARN/ERROR messages force the diagnostic file durable.
pub struct Storage {
    volume_manager: VolumeManagerImpl,

    // One FAT volume stays open for the lifetime of Storage. When a survey is
    // active, `mode` owns the current Bxxxxxxx directory and RX.LOG. When no
    // survey is active, `mode` instead owns the root directory used by the
    // global DEBUG.LOG fallback.
    storage_volume: RawVolume,
    mode: StorageMode,

    // DEBUG.LOG is survey/session-local whenever a survey is active. It falls
    // back to /DEBUG.LOG only when there is no active survey (or session setup
    // fails). Diagnostic-log failure never takes the rest of storage offline.
    debug_file: Option<RawFile>,
    debug_buffer: String<DEBUG_BUFFER_CAPACITY>,

    local_records_written: u32,

    // Once GPS UTC is known, this anchor lets every subsequent log product
    // carry both monotonic boot time and an inferred UTC second. Analyzer can
    // also use the explicit TIME_SYNC event to map earlier monotonic records.
    utc_anchor: Option<UtcAnchor>,
}

#[derive(Clone, Copy)]
enum StorageMode {
    Survey {
        session_dir: RawDirectory,
        rx_file: RawFile,
        local_file: RawFile,
        event_file: RawFile,
        survey_id: u32,
        session_number: u32,
    },
    Global {
        root_dir: RawDirectory,
    },
}

#[derive(Clone, Copy)]
struct UtcAnchor {
    mono_ms: u64,
    unix_ms: u64,
}

const LOCAL_FLAG_SURVEY_ACTIVE: u32 = 1 << 0;
const LOCAL_FLAG_GPS_ONLINE: u32 = 1 << 1;
const LOCAL_FLAG_GPS_FIX: u32 = 1 << 2;
const LOCAL_FLAG_LINK_UP: u32 = 1 << 3;

/// Temporary FAT timestamp source.
///
/// GOLFER does not yet have a validated wall clock at storage initialization,
/// so filesystem metadata is intentionally stamped with the FAT epoch
/// (1980-01-01 00:00:00) rather than pretending we know the current time.
///
/// Telemetry and diagnostics use monotonic milliseconds since this boot until
/// the real survey time model exists.
#[derive(Clone, Copy)]
struct PlaceholderTimeSource;

impl TimeSource for PlaceholderTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 10, // 1980
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}


/// Fast, bounded SD presence probe.
///
/// This is deliberately performed before invoking embedded-sdmmc's full card
/// initialization. A missing card should be recognized in milliseconds rather
/// than allowing the filesystem stack to dominate GOLFER boot.
///
/// Success means CMD0 returned R1=0x01 (SPI idle state). It does NOT mean the
/// filesystem is valid.
pub fn card_present(
    bus: &'static RefCell<Spi0Bus>,
    sd_cs: &mut Output<'static>,
) -> bool {
    info!("Probing for SD card");

    sd_cs.set_high();

    // SD SPI-mode entry requires >=74 clocks with CS high. 10 bytes = 80 clocks.
    if !spi0_bus::send_sd_startup_clocks(bus) {
        warn!("SD presence probe: startup clocks failed");
        return false;
    }

    for attempt in 1..=SD_PRESENCE_ATTEMPTS {
        sd_cs.set_low();

        let write_result = {
            let mut spi = bus.borrow_mut();

            if spi.blocking_write(&[0xFF]).is_err() {
                false
            } else {
                spi.blocking_write(&SD_CMD0).is_ok()
            }
        };

        if !write_result {
            sd_cs.set_high();
            warn!("SD presence probe: SPI write failed");
            return false;
        }

        let mut response = 0xFFu8;

        for _ in 0..16 {
            let mut byte = [0xFFu8];

            let read_ok = bus
                .borrow_mut()
                .blocking_transfer_in_place(&mut byte)
                .is_ok();

            if !read_ok {
                sd_cs.set_high();
                warn!("SD presence probe: SPI read failed");
                return false;
            }

            // R1 responses have bit 7 clear.
            if byte[0] & 0x80 == 0 {
                response = byte[0];
                break;
            }
        }

        sd_cs.set_high();

        // Extra clocks after deselect.
        let _ = bus.borrow_mut().blocking_write(&[0xFF]);

        if response == 0x01 {
            info!(
                "SD card detected on presence probe attempt {}",
                attempt
            );
            return true;
        }
    }

    warn!("SD card not detected");
    false
}

impl Storage {
    /// Initialize SD storage.
    ///
    /// When `active_survey_id` is Some, create a new per-boot/session directory:
    ///
    ///     /SURVEY/Axxxxxxx/Bxxxxxxx/
    ///
    /// containing RX.LOG, LOCAL.LOG, EVENT.LOG, DEBUG.LOG, and META.TXT.
    ///
    /// When no survey is active, diagnostics fall back to /DEBUG.LOG. The same
    /// global fallback is used if survey-session setup fails, so a healthy card
    /// can still capture why survey logging did not come online.
    pub fn init(
        bus: &'static RefCell<Spi0Bus>,
        sd_cs: Output<'static>,
        system_info: SystemInfo,
        active_survey_id: Option<u32>,
    ) -> Option<Self> {
        let total_started = Instant::now();
        info!("Initializing GOLFER storage");

        let card_init_started = Instant::now();

        if !spi0_bus::send_sd_startup_clocks(bus) {
            error!("SD startup clocks failed");
            return None;
        }

        let spi_device = match RefCellDevice::new(bus, sd_cs, Delay) {
            Ok(device) => device,
            Err(_) => {
                error!("Failed to create SD SPI device");
                return None;
            }
        };

        let sdcard = SdCard::new(spi_device, Delay);

        let card_size = match sdcard.num_bytes() {
            Ok(size) => size,
            Err(_) => {
                error!("SD card initialization failed");
                return None;
            }
        };

        let card_init_us = Instant::now()
            .duration_since(card_init_started)
            .as_micros();

        info!("SD card initialized: {} bytes", card_size);

        // SD cards require the low SPI clock only during SPI-mode/card
        // initialization. Once num_bytes() succeeds the card is initialized,
        // so immediately promote the shared bus to normal runtime speed before
        // doing any FAT/directory/file work. V2A originally left the entire
        // filesystem setup at 400 kHz, which made boot take several seconds.
        spi0_bus::set_frequency(bus, spi0_bus::RUN_FREQUENCY_HZ);
        info!(
            "SD card promoted to runtime SPI frequency: {} Hz",
            spi0_bus::RUN_FREQUENCY_HZ
        );

        let volume_manager = VolumeManager::new(sdcard, PlaceholderTimeSource);

        let syslog_started = Instant::now();
        if !initialize_system_log(&volume_manager) {
            warn!("SD card online, but system log initialization failed");
        }
        let syslog_us = Instant::now()
            .duration_since(syslog_started)
            .as_micros();

        let mount_started = Instant::now();
        let storage_volume = match volume_manager.open_raw_volume(VolumeIdx(0)) {
            Ok(volume) => volume,
            Err(_) => {
                error!("Failed to mount FAT volume for GOLFER storage");
                return None;
            }
        };
        let mount_us = Instant::now()
            .duration_since(mount_started)
            .as_micros();

        let session_start_ms = Instant::now().as_millis();
        let mode_setup_started = Instant::now();

        let (mode, debug_file) = match active_survey_id {
            Some(survey_id) => {
                match initialize_survey_session(
                    &volume_manager,
                    storage_volume,
                    survey_id,
                    system_info,
                    session_start_ms,
                ) {
                    Some((session_dir, rx_file, local_file, event_file, session_number, debug_file)) => {
                        info!(
                            "Logging V2 survey session online: survey={} boot/session={}",
                            survey_id,
                            session_number
                        );

                        (
                            StorageMode::Survey {
                                session_dir,
                                rx_file,
                                local_file,
                                event_file,
                                survey_id,
                                session_number,
                            },
                            debug_file,
                        )
                    }
                    None => {
                        error!(
                            "Survey-session logging initialization failed; falling back to global DEBUG.LOG"
                        );

                        let (root_dir, debug_file) =
                            match initialize_global_debug_log(&volume_manager, storage_volume) {
                                Some(handles) => handles,
                                None => {
                                    let _ = volume_manager.close_volume(storage_volume);
                                    return None;
                                }
                            };

                        (StorageMode::Global { root_dir }, Some(debug_file))
                    }
                }
            }
            None => {
                let (root_dir, debug_file) =
                    match initialize_global_debug_log(&volume_manager, storage_volume) {
                        Some(handles) => handles,
                        None => {
                            let _ = volume_manager.close_volume(storage_volume);
                            return None;
                        }
                    };

                info!("No active survey; persistent diagnostics using /DEBUG.LOG");
                (StorageMode::Global { root_dir }, Some(debug_file))
            }
        };

        let mode_setup_us = Instant::now()
            .duration_since(mode_setup_started)
            .as_micros();

        if debug_file.is_some() {
            info!(
                "Persistent DEBUG.LOG online at level {}",
                PERSISTENT_LOG_LEVEL.label()
            );
        } else {
            warn!("Persistent DEBUG.LOG unavailable; survey logging continues");
        }

        let total_us = Instant::now()
            .duration_since(total_started)
            .as_micros();

        info!(
            "SD_INIT_TIMING card_init_us={} syslog_us={} mount_us={} mode_setup_us={} total_us={}",
            card_init_us,
            syslog_us,
            mount_us,
            mode_setup_us,
            total_us
        );

        Some(Self {
            volume_manager,
            storage_volume,
            mode,
            debug_file,
            debug_buffer: String::new(),
            local_records_written: 0,
            utc_anchor: None,
        })
    }

    /// Generic gated persistent diagnostic event.
    ///
    /// DEBUG/TRACE/INFO are buffered in RAM. WARN/ERROR force the current
    /// diagnostic buffer to the card and flush DEBUG.LOG.
    pub fn diag(
        &mut self,
        level: PersistentLogLevel,
        uptime_ms: u64,
        originator: &'static str,
        event_id: &'static str,
        args: Arguments<'_>,
    ) {
        if level > PERSISTENT_LOG_LEVEL || self.debug_file.is_none() {
            return;
        }

        let mut line: String<DEBUG_LINE_CAPACITY> = String::new();

        // Persistent diagnostic envelope:
        //
        // <utc_unix_or_NA> <GPS|UNKNOWN> <mono_ms> <level> <originator>
        // <event_id> <message...>
        //
        // Monotonic time is always authoritative. Once GPS UTC becomes valid,
        // the current anchor provides a human/cross-device wall-clock millisecond.
        let utc = self.utc_for_mono_ms(uptime_ms);
        let time_write = match utc {
            Some(utc) => write!(
                line,
                "{} GPS {} {} {} {}",
                utc, uptime_ms, level.label(), originator, event_id
            ),
            None => write!(
                line,
                "NA UNKNOWN {} {} {} {}",
                uptime_ms, level.label(), originator, event_id
            ),
        };

        if time_write.is_err() {
            return;
        }

        // Many events carry key=value details, but an event may legitimately
        // have no additional message payload.
        let before_message_len = line.len();

        if write!(line, " ").is_err() {
            return;
        }

        if line.write_fmt(args).is_err() {
            return;
        }

        // Avoid leaving a meaningless trailing separator when the caller used
        // an empty message.
        if line.len() == before_message_len + 1 {
            line.truncate(before_message_len);
        }

        if writeln!(line).is_err() {
            return;
        }

        // If this line would overflow the RAM buffer, batch-write the previous
        // diagnostics first. This write is intentionally not a FAT flush.
        if self
            .debug_buffer
            .len()
            .saturating_add(line.len())
            > DEBUG_BUFFER_CAPACITY
        {
            self.flush_debug_buffer(false);
        }

        if self.debug_buffer.push_str(line.as_str()).is_err() {
            warn!("DEBUG.LOG RAM buffer overflow; dropping diagnostic line");
            return;
        }

        if level.forces_persistent_flush() {
            self.flush_debug_buffer(true);
        }
    }

    /// Update the monotonic -> GPS UTC anchor.
    ///
    /// Returns true the first time this boot acquires usable GPS UTC. The caller
    /// uses that transition to append one explicit TIME_SYNC event. Subsequent
    /// updates keep inferred UTC aligned with the latest RMC sentence without
    /// manufacturing a new event every second.
    pub fn update_time_anchor(
        &mut self,
        mono_ms: u64,
        unix_ms: u64,
    ) -> bool {
        let first_sync = self.utc_anchor.is_none();
        self.utc_anchor = Some(UtcAnchor {
            mono_ms,
            unix_ms,
        });
        first_sync
    }

    /// Append a parser-friendly event record with a local GPS snapshot.
    ///
    /// EVENT.LOG is append-only. Events that conceptually annotate an earlier
    /// RX record (for example LINK_LOST) carry references such as LAST_SEQ and
    /// LAST_RX_MONO_MS instead of rewriting already-persisted history.
    pub fn log_event(
        &mut self,
        mono_ms: u64,
        event_id: &'static str,
        gps: GpsState,
        args: Arguments<'_>,
    ) -> bool {
        let event_file = match self.mode {
            StorageMode::Survey { event_file, .. } => event_file,
            StorageMode::Global { .. } => return false,
        };

        let mut line: String<512> = String::new();

        if write!(line, "MONO_MS={},UTC_MS=", mono_ms).is_err() {
            return false;
        }
        if append_optional_u64(&mut line, self.utc_for_mono_ms(mono_ms)).is_err() {
            return false;
        }
        if write!(
            line,
            ",EVENT={},GPS_ONLINE={},GPS_FIX={},LAT_E7=",
            event_id,
            gps.online as u8,
            gps.fix as u8,
        )
        .is_err()
        {
            return false;
        }
        if append_optional_i32(&mut line, gps.latitude_e7).is_err() {
            return false;
        }
        if write!(line, ",LON_E7=").is_err()
            || append_optional_i32(&mut line, gps.longitude_e7).is_err()
        {
            return false;
        }

        let before_details = line.len();
        if write!(line, ",").is_err() || line.write_fmt(args).is_err() {
            return false;
        }
        if line.len() == before_details + 1 {
            line.truncate(before_details);
        }
        if writeln!(line).is_err() {
            return false;
        }

        if self.volume_manager.write(event_file, line.as_bytes()).is_err() {
            error!("EVENT.LOG write failed");
            return false;
        }

        true
    }

    /// Append the local GOLFER state once per second regardless of RF success.
    ///
    /// This is the survey's continuous ground-truth track. Missing radio frames
    /// therefore create gaps in RX.LOG while LOCAL.LOG continues through the
    /// dead zone. BME280/battery fields are reserved now and remain NA until
    /// those local subsystems are brought online.
    pub fn log_local_sample(
        &mut self,
        mono_ms: u64,
        gps: GpsState,
        link_up: bool,
        last_rssi: Option<i16>,
        last_snr: Option<i16>,
        received: u32,
        missed: u32,
        app_crc_failures: u32,
    ) -> bool {
        let (rx_file, local_file, event_file) = match self.mode {
            StorageMode::Survey {
                rx_file,
                local_file,
                event_file,
                ..
            } => (rx_file, local_file, event_file),
            StorageMode::Global { .. } => return false,
        };

        let mut flags = LOCAL_FLAG_SURVEY_ACTIVE;
        if gps.online {
            flags |= LOCAL_FLAG_GPS_ONLINE;
        }
        if gps.fix {
            flags |= LOCAL_FLAG_GPS_FIX;
        }
        if link_up {
            flags |= LOCAL_FLAG_LINK_UP;
        }

        let mut line: String<640> = String::new();
        if write!(line, "MONO_MS={},UTC_MS=", mono_ms).is_err()
            || append_optional_u64(&mut line, self.utc_for_mono_ms(mono_ms)).is_err()
            || write!(
                line,
                ",FLAGS={:08X},GPS_ONLINE={},GPS_FIX={},LAT_E7=",
                flags,
                gps.online as u8,
                gps.fix as u8,
            )
            .is_err()
            || append_optional_i32(&mut line, gps.latitude_e7).is_err()
            || write!(line, ",LON_E7=").is_err()
            || append_optional_i32(&mut line, gps.longitude_e7).is_err()
            || write!(line, ",ALT_HALF_M=").is_err()
            || append_optional_i16(&mut line, gps.altitude_half_m).is_err()
            || write!(line, ",SPEED_CM_S=").is_err()
            || append_optional_u16(&mut line, gps.speed_cm_s).is_err()
            || write!(line, ",COURSE_CDEG=").is_err()
            || append_optional_u16(&mut line, gps.course_cdeg).is_err()
            || write!(line, ",SATS=").is_err()
            || append_optional_u8(&mut line, gps.satellites).is_err()
            || write!(line, ",HDOP_TENTHS=").is_err()
            || append_optional_u8(&mut line, gps.hdop_tenths).is_err()
            || write!(
                line,
                ",LINK_UP={},LAST_RSSI=",
                link_up as u8,
            )
            .is_err()
            || append_optional_i16(&mut line, last_rssi).is_err()
            || write!(line, ",LAST_SNR=").is_err()
            || append_optional_i16(&mut line, last_snr).is_err()
            || writeln!(
                line,
                ",RX={},MISSED={},APP_CRC_FAIL={},TEMP_CENTI_C=NA,PRESSURE_10PA=NA,HUMIDITY_HALF_PCT=NA,BATTERY_SOC=NA",
                received,
                missed,
                app_crc_failures,
            )
            .is_err()
        {
            error!("LOCAL.LOG line formatting overflow");
            return false;
        }

        let append_started = Instant::now();
        if self.volume_manager.write(local_file, line.as_bytes()).is_err() {
            error!("LOCAL.LOG write failed");
            return false;
        }
        let append_us = Instant::now().duration_since(append_started).as_micros();

        self.local_records_written = self.local_records_written.saturating_add(1);

        self.diag(
            PersistentLogLevel::Trace,
            mono_ms,
            "STORAGE",
            "LOCAL_APPEND",
            format_args!("us={} bytes={}", append_us, line.len()),
        );

        // LOCAL.LOG is the 1 Hz heartbeat even when RF is absent. Every tenth
        // local sample checkpoints all survey data products together so a dead
        // RF zone still periodically makes RX/EVENT/DEBUG durable.
        if self.local_records_written % 10 == 0 {
            let checkpoint_started = Instant::now();
            let mut ok = true;

            if self.volume_manager.flush_file(local_file).is_err() {
                ok = false;
            }
            if self.volume_manager.flush_file(rx_file).is_err() {
                ok = false;
            }
            if self.volume_manager.flush_file(event_file).is_err() {
                ok = false;
            }
            self.flush_debug_buffer(true);

            let checkpoint_us = Instant::now()
                .duration_since(checkpoint_started)
                .as_micros();

            if ok {
                self.diag(
                    PersistentLogLevel::Debug,
                    mono_ms,
                    "STORAGE",
                    "SURVEY_CHECKPOINT",
                    format_args!("us={}", checkpoint_us),
                );
            } else {
                error!("Survey checkpoint flush failed");
                self.diag(
                    PersistentLogLevel::Error,
                    mono_ms,
                    "STORAGE",
                    "SURVEY_CHECKPOINT_FAILED",
                    format_args!("us={}", checkpoint_us),
                );
            }
        }

        true
    }

    /// Append one accepted native TelemetryV1 reception.
    ///
    /// RX.LOG records both sides of the observation: local position at receive
    /// time plus the remote telemetry carried by the packet. Application CRC,
    /// survey-context, and packet decoding have already succeeded before this
    /// function is called.
    pub fn log_receiver_packet(
        &mut self,
        mono_ms: u64,
        telemetry: TelemetryV1,
        rssi: i16,
        snr: i16,
        received: u32,
        missed: u32,
        app_crc_failures: u32,
        local_gps: GpsState,
    ) -> Option<TelemetryWriteStats> {
        let rx_file = match self.mode {
            StorageMode::Survey { rx_file, .. } => rx_file,
            StorageMode::Global { .. } => return None,
        };

        let mut line: String<960> = String::new();

        if write!(
            line,
            "MONO_MS={},UTC_MS=",
            mono_ms,
        )
        .is_err()
            || append_optional_u64(&mut line, self.utc_for_mono_ms(mono_ms)).is_err()
            || write!(
                line,
                ",SEQ={},SENDER={:016X},SURVEY={:08X},MODE={},RSSI={},SNR={},RX={},MISSED={},APP_CRC_FAIL={},LOCAL_GPS_FIX={},LOCAL_LAT_E7=",
                telemetry.sequence,
                telemetry.sender_system_id,
                telemetry.survey_id,
                telemetry.sender_mode,
                rssi,
                snr,
                received,
                missed,
                app_crc_failures,
                local_gps.fix as u8,
            )
            .is_err()
            || append_optional_i32(&mut line, local_gps.latitude_e7).is_err()
            || write!(line, ",LOCAL_LON_E7=").is_err()
            || append_optional_i32(&mut line, local_gps.longitude_e7).is_err()
            || write!(line, ",TX_UTC_S=").is_err()
            || append_optional_u32(&mut line, telemetry.gps_unix_time).is_err()
            || write!(line, ",TX_LAT_E7=").is_err()
            || append_optional_i32(&mut line, telemetry.latitude_e7).is_err()
            || write!(line, ",TX_LON_E7=").is_err()
            || append_optional_i32(&mut line, telemetry.longitude_e7).is_err()
            || write!(line, ",TX_ALT_HALF_M=").is_err()
            || append_optional_i16(&mut line, telemetry.altitude_half_m).is_err()
            || write!(line, ",TX_SPEED_CM_S=").is_err()
            || append_optional_u16(&mut line, telemetry.speed_cm_s).is_err()
            || write!(line, ",TX_COURSE_CDEG=").is_err()
            || append_optional_u16(&mut line, telemetry.course_cdeg).is_err()
            || write!(
                line,
                ",TX_FIX={},TX_SATS={},TX_HDOP_TENTHS=",
                telemetry.gps_fix_class as u8,
                telemetry.satellites,
            )
            .is_err()
            || append_optional_u8(&mut line, telemetry.hdop_tenths).is_err()
            || write!(line, ",TX_TEMP_CENTI_C=").is_err()
            || append_optional_i16(&mut line, telemetry.temperature_centi_c).is_err()
            || write!(line, ",TX_PRESSURE_10PA=").is_err()
            || append_optional_u16(&mut line, telemetry.pressure_10pa).is_err()
            || write!(line, ",TX_HUMIDITY_HALF_PCT=").is_err()
            || append_optional_u8(&mut line, telemetry.humidity_half_percent).is_err()
            || write!(line, ",TX_BATTERY_SOC=").is_err()
            || append_optional_u8(&mut line, telemetry.battery_soc_percent).is_err()
            || writeln!(line).is_err()
        {
            error!("RX.LOG line formatting overflow");
            return None;
        }

        let append_started = Instant::now();

        if self.volume_manager.write(rx_file, line.as_bytes()).is_err() {
            error!("RX.LOG write failed");
            self.diag(
                PersistentLogLevel::Error,
                mono_ms,
                "STORAGE",
                "RX_APPEND_FAILED",
                format_args!(""),
            );
            return None;
        }

        let append_us = Instant::now()
            .duration_since(append_started)
            .as_micros();


        self.diag(
            PersistentLogLevel::Trace,
            mono_ms,
            "STORAGE",
            "RX_APPEND",
            format_args!("us={} bytes={}", append_us, line.len()),
        );

        Some(TelemetryWriteStats {
            append_us,
            checkpoint_us: None,
            total_us: append_us,
        })
    }

    fn utc_for_mono_ms(&self, mono_ms: u64) -> Option<u64> {
        let anchor = self.utc_anchor?;

        if mono_ms >= anchor.mono_ms {
            Some(
                anchor.unix_ms
                    .saturating_add(mono_ms - anchor.mono_ms),
            )
        } else {
            anchor.unix_ms
                .checked_sub(anchor.mono_ms - mono_ms)
        }
    }

    /// Boot/session number for the active survey recording. Zero means there is
    /// currently no survey-local recording session.
    pub fn segment_number(&self) -> u32 {
        match self.mode {
            StorageMode::Survey { session_number, .. } => session_number,
            StorageMode::Global { .. } => 0,
        }
    }

    pub fn survey_logging_active(&self) -> bool {
        matches!(self.mode, StorageMode::Survey { .. })
    }

    pub fn active_survey_id(&self) -> Option<u32> {
        match self.mode {
            StorageMode::Survey { survey_id, .. } => Some(survey_id),
            StorageMode::Global { .. } => None,
        }
    }

    fn flush_debug_buffer(
        &mut self,
        flush_file: bool,
    ) {
        let Some(debug_file) = self.debug_file else {
            self.debug_buffer.clear();
            return;
        };

        if !self.debug_buffer.is_empty() {
            if self
                .volume_manager
                .write(
                    debug_file,
                    self.debug_buffer.as_bytes(),
                )
                .is_err()
            {
                warn!("Failed to append DEBUG.LOG");
                self.debug_buffer.clear();
                return;
            }

            self.debug_buffer.clear();
        }

        if flush_file
            && self
                .volume_manager
                .flush_file(debug_file)
                .is_err()
        {
            warn!("Failed to flush DEBUG.LOG");
        }
    }
}

impl Drop for Storage {
    fn drop(&mut self) {
        self.flush_debug_buffer(true);

        if let Some(debug_file) = self.debug_file {
            let _ = self.volume_manager.close_file(debug_file);
        }

        match self.mode {
            StorageMode::Survey {
                session_dir,
                rx_file,
                local_file,
                event_file,
                ..
            } => {
                let _ = self.volume_manager.flush_file(rx_file);
                let _ = self.volume_manager.flush_file(local_file);
                let _ = self.volume_manager.flush_file(event_file);
                let _ = self.volume_manager.close_file(rx_file);
                let _ = self.volume_manager.close_file(local_file);
                let _ = self.volume_manager.close_file(event_file);
                let _ = self.volume_manager.close_dir(session_dir);
            }
            StorageMode::Global { root_dir } => {
                let _ = self.volume_manager.close_dir(root_dir);
            }
        }

        let _ = self.volume_manager.close_volume(self.storage_volume);
    }
}

fn append_optional_u64<const N: usize>(line: &mut String<N>, value: Option<u64>) -> core::fmt::Result {
    match value {
        Some(value) => write!(line, "{}", value),
        None => write!(line, "NA"),
    }
}

fn append_optional_u32<const N: usize>(line: &mut String<N>, value: Option<u32>) -> core::fmt::Result {
    match value {
        Some(value) => write!(line, "{}", value),
        None => write!(line, "NA"),
    }
}

fn append_optional_i32<const N: usize>(line: &mut String<N>, value: Option<i32>) -> core::fmt::Result {
    match value {
        Some(value) => write!(line, "{}", value),
        None => write!(line, "NA"),
    }
}

fn append_optional_i16<const N: usize>(line: &mut String<N>, value: Option<i16>) -> core::fmt::Result {
    match value {
        Some(value) => write!(line, "{}", value),
        None => write!(line, "NA"),
    }
}

fn append_optional_u16<const N: usize>(line: &mut String<N>, value: Option<u16>) -> core::fmt::Result {
    match value {
        Some(value) => write!(line, "{}", value),
        None => write!(line, "NA"),
    }
}

fn append_optional_u8<const N: usize>(line: &mut String<N>, value: Option<u8>) -> core::fmt::Result {
    match value {
        Some(value) => write!(line, "{}", value),
        None => write!(line, "NA"),
    }
}

fn initialize_global_debug_log(
    volume_manager: &VolumeManagerImpl,
    volume: RawVolume,
) -> Option<(RawDirectory, RawFile)> {
    let root = volume_manager.open_root_dir(volume).ok()?;

    match open_or_create_append_file(volume_manager, root, DEBUG_LOG_FILENAME) {
        Some(file) => Some((root, file)),
        None => {
            let _ = volume_manager.close_dir(root);
            None
        }
    }
}

fn open_or_create_append_file(
    volume_manager: &VolumeManagerImpl,
    dir: RawDirectory,
    filename: &str,
) -> Option<RawFile> {
    let filename = ShortFileName::create_from_str(filename).ok()?;
    let mut already_exists = false;

    volume_manager
        .iterate_dir(dir, |entry| {
            if entry.name == filename {
                already_exists = true;
            }
        })
        .ok()?;

    let mode = if already_exists {
        Mode::ReadWriteAppend
    } else {
        Mode::ReadWriteCreate
    };

    volume_manager.open_file_in_dir(dir, &filename, mode).ok()
}

fn initialize_system_log(
    volume_manager: &VolumeManagerImpl,
) -> bool {
    let volume = match volume_manager.open_raw_volume(VolumeIdx(0)) {
        Ok(volume) => volume,
        Err(_) => {
            error!("Failed to mount FAT volume 0");
            return false;
        }
    };

    let root = match volume_manager.open_root_dir(volume) {
        Ok(root) => root,
        Err(_) => {
            error!("Failed to open SD root directory");
            let _ = volume_manager.close_volume(volume);
            return false;
        }
    };

    let filename =
        match ShortFileName::create_from_str(SYSTEM_LOG_FILENAME) {
            Ok(name) => name,
            Err(_) => {
                error!("Internal SYS_LOG filename is invalid");
                let _ = volume_manager.close_dir(root);
                let _ = volume_manager.close_volume(volume);
                return false;
            }
        };

    let mut already_exists = false;

    if volume_manager
        .iterate_dir(root, |entry| {
            if entry.name == filename {
                already_exists = true;
            }
        })
        .is_err()
    {
        error!("Failed to inspect SD root directory");
        let _ = volume_manager.close_dir(root);
        let _ = volume_manager.close_volume(volume);
        return false;
    }

    let mode = if already_exists {
        Mode::ReadWriteAppend
    } else {
        Mode::ReadWriteCreate
    };

    let file =
        match volume_manager.open_file_in_dir(root, &filename, mode) {
            Ok(file) => file,
            Err(_) => {
                error!("Failed to open SYS_LOG.TXT");
                let _ = volume_manager.close_dir(root);
                let _ = volume_manager.close_volume(volume);
                return false;
            }
        };

    if !already_exists {
        if volume_manager
            .write(file, b"Hello GOLFER!\r\n")
            .is_err()
        {
            error!("Failed to write SYS_LOG greeting");
            let _ = volume_manager.close_file(file);
            let _ = volume_manager.close_dir(root);
            let _ = volume_manager.close_volume(volume);
            return false;
        }

        info!("Created SYS_LOG.TXT");
    } else {
        info!("Found existing SYS_LOG.TXT");
    }

    if volume_manager
        .write(file, b"SYSTEM STARTUP\r\n")
        .is_err()
    {
        error!("Failed to append startup event to SYS_LOG.TXT");
        let _ = volume_manager.close_file(file);
        let _ = volume_manager.close_dir(root);
        let _ = volume_manager.close_volume(volume);
        return false;
    }

    if volume_manager.flush_file(file).is_err() {
        error!("Failed to flush SYS_LOG.TXT");
        let _ = volume_manager.close_file(file);
        let _ = volume_manager.close_dir(root);
        let _ = volume_manager.close_volume(volume);
        return false;
    }

    if volume_manager.close_file(file).is_err() {
        error!("Failed to close SYS_LOG.TXT");
        let _ = volume_manager.close_dir(root);
        let _ = volume_manager.close_volume(volume);
        return false;
    }

    if volume_manager.close_dir(root).is_err() {
        error!("Failed to close SD root directory");
        let _ = volume_manager.close_volume(volume);
        return false;
    }

    if volume_manager.close_volume(volume).is_err() {
        error!("Failed to close FAT volume");
        return false;
    }

    info!("SYSTEM STARTUP appended to SYS_LOG.TXT");
    true
}

/// Create one Logging V2 survey boot/session:
///
///     /SURVEY/Axxxxxxx/Bxxxxxxx/
///         RX.LOG
///         LOCAL.LOG
///         EVENT.LOG
///         DEBUG.LOG
///         META.TXT
///
/// Bxxxxxxx is an ergonomic recording/boot distinction only. The Survey ID is
/// canonical and survives reboot/power loss.
fn initialize_survey_session(
    volume_manager: &VolumeManagerImpl,
    volume: RawVolume,
    survey_id: u32,
    system_info: SystemInfo,
    start_mono_ms: u64,
) -> Option<(RawDirectory, RawFile, RawFile, RawFile, u32, Option<RawFile>)> {
    let root = match volume_manager.open_root_dir(volume) {
        Ok(root) => root,
        Err(_) => {
            error!("Failed to open root for survey session");
            return None;
        }
    };

    let survey_root = match ensure_directory(volume_manager, root, SURVEY_ROOT_DIR) {
        Some(dir) => dir,
        None => {
            error!("Failed to ensure /SURVEY");
            let _ = volume_manager.close_dir(root);
            return None;
        }
    };

    let _ = volume_manager.close_dir(root);

    let survey_name = match survey_directory_name(survey_id) {
        Some(name) => name,
        None => {
            error!("Failed to format survey directory name");
            let _ = volume_manager.close_dir(survey_root);
            return None;
        }
    };
    let survey_dir = match ensure_directory(volume_manager, survey_root, survey_name.as_str()) {
        Some(dir) => dir,
        None => {
            error!("Failed to ensure survey directory {}", survey_name.as_str());
            let _ = volume_manager.close_dir(survey_root);
            return None;
        }
    };

    let _ = volume_manager.close_dir(survey_root);

    // Allocate the next boot/session from a tiny persistent counter rather than
    // scanning every Bxxxxxxx directory on every boot. Existing V2A surveys do
    // not yet have BOOT.NXT, so the allocator performs one compatibility scan,
    // seeds the counter, and uses O(1) allocation on subsequent boots.
    //
    // The *following* number is flushed before the session directory is
    // created. A sudden power loss can therefore leave a harmless numbering
    // gap, but it cannot cause a later boot to reuse an already allocated
    // session number.
    let next_session = match allocate_boot_session(volume_manager, survey_dir) {
        Some(session) => session,
        None => {
            error!("Failed to allocate boot/session number");
            let _ = volume_manager.close_dir(survey_dir);
            return None;
        }
    };

    let session_name = match boot_session_directory_name(next_session) {
        Some(name) => name,
        None => {
            error!("Failed to format boot/session directory name");
            let _ = volume_manager.close_dir(survey_dir);
            return None;
        }
    };

    if volume_manager
        .make_dir_in_dir(survey_dir, session_name.as_str())
        .is_err()
    {
        error!("Failed to create boot/session directory {}", session_name.as_str());
        let _ = volume_manager.close_dir(survey_dir);
        return None;
    }

    let session_dir = match volume_manager.open_dir(survey_dir, session_name.as_str()) {
        Ok(dir) => dir,
        Err(_) => {
            error!("Failed to open boot/session directory {}", session_name.as_str());
            let _ = volume_manager.close_dir(survey_dir);
            return None;
        }
    };

    let _ = volume_manager.close_dir(survey_dir);

    // V2B keeps the three survey data products open for the lifetime of this
    // boot/session. All are append-only; a 10-second LOCAL.LOG heartbeat
    // checkpoints RX/LOCAL/EVENT/DEBUG together.
    let local_file = match open_or_create_append_file(
        volume_manager, session_dir, LOCAL_LOG_FILENAME
    ) {
        Some(file) => file,
        None => {
            error!("Failed to create LOCAL.LOG");
            let _ = volume_manager.close_dir(session_dir);
            return None;
        }
    };

    let event_file = match open_or_create_append_file(
        volume_manager, session_dir, EVENT_LOG_FILENAME
    ) {
        Some(file) => file,
        None => {
            error!("Failed to create EVENT.LOG");
            let _ = volume_manager.close_file(local_file);
            let _ = volume_manager.close_dir(session_dir);
            return None;
        }
    };

    if !create_metadata_file(
        volume_manager,
        session_dir,
        survey_id,
        next_session,
        system_info,
        start_mono_ms,
    ) {
        error!("Failed to create META.TXT");
        let _ = volume_manager.close_file(event_file);
        let _ = volume_manager.close_file(local_file);
        let _ = volume_manager.close_dir(session_dir);
        return None;
    }

    let rx_name = match ShortFileName::create_from_str(RX_LOG_FILENAME) {
        Ok(name) => name,
        Err(_) => {
            error!("Internal RX.LOG filename is invalid");
            let _ = volume_manager.close_file(event_file);
            let _ = volume_manager.close_file(local_file);
            let _ = volume_manager.close_dir(session_dir);
            return None;
        }
    };
    let rx_file = match volume_manager.open_file_in_dir(
        session_dir,
        &rx_name,
        Mode::ReadWriteCreate,
    ) {
        Ok(file) => file,
        Err(_) => {
            error!("Failed to create RX.LOG");
            let _ = volume_manager.close_file(event_file);
            let _ = volume_manager.close_file(local_file);
            let _ = volume_manager.close_dir(session_dir);
            return None;
        }
    };

    // DEBUG.LOG is intentionally optional. RX logging remains available if the
    // diagnostic file itself cannot be created.
    let debug_file = match open_or_create_append_file(
        volume_manager,
        session_dir,
        DEBUG_LOG_FILENAME,
    ) {
        Some(file) => Some(file),
        None => {
            warn!("Failed to create survey/session DEBUG.LOG");
            None
        }
    };

    info!(
        "Test survey active: /SURVEY/{}/{}",
        survey_name.as_str(),
        session_name.as_str()
    );

    Some((session_dir, rx_file, local_file, event_file, next_session, debug_file))
}

fn create_metadata_file(
    volume_manager: &VolumeManagerImpl,
    dir: RawDirectory,
    survey_id: u32,
    session_number: u32,
    system_info: SystemInfo,
    start_mono_ms: u64,
) -> bool {
    let name = match ShortFileName::create_from_str(META_FILENAME) {
        Ok(name) => name,
        Err(_) => return false,
    };

    let file = match volume_manager.open_file_in_dir(dir, &name, Mode::ReadWriteCreate) {
        Ok(file) => file,
        Err(_) => return false,
    };

    let mut metadata: String<512> = String::new();
    let session_name = match boot_session_directory_name(session_number) {
        Some(name) => name,
        None => {
            let _ = volume_manager.close_file(file);
            return false;
        }
    };

    if write!(
        metadata,
        "LOG_SCHEMA={}\r\nIMPLEMENTATION_STAGE=V2B1\r\nSURVEY_ID={:08X}\r\nSYSTEM_ID={:016X}\r\nBOOT_SESSION={}\r\nSESSION_DIR={}\r\nFIRMWARE={}\r\nRF_PROTOCOL={}\r\nCONFIG_SCHEMA={}\r\nSTART_MONO_MS={}\r\nTIME_MODEL=MONOTONIC_PLUS_GPS_UTC_ANCHOR\r\nLOCAL_SAMPLE_HZ=1\r\nRX_PACKET_FORMAT=TELEMETRY_V1\r\n",
        LOG_SCHEMA_VERSION,
        survey_id,
        system_info.system_id.value,
        session_number,
        session_name.as_str(),
        system_info.firmware_version.value,
        system_info.protocol_version.value,
        system_info.config_version.value,
        start_mono_ms,
    )
    .is_err()
    {
        let _ = volume_manager.close_file(file);
        return false;
    }

    if volume_manager.write(file, metadata.as_bytes()).is_err() {
        let _ = volume_manager.close_file(file);
        return false;
    }

    if volume_manager.flush_file(file).is_err() {
        let _ = volume_manager.close_file(file);
        return false;
    }

    volume_manager.close_file(file).is_ok()
}

fn survey_directory_name(survey_id: u32) -> Option<String<8>> {
    let mut name: String<8> = String::new();
    write!(name, "{:08X}", survey_id).ok()?;
    Some(name)
}

fn boot_session_directory_name(session: u32) -> Option<String<8>> {
    if session == 0 || session > MAX_BOOT_SESSION {
        return None;
    }

    let mut name: String<8> = String::new();
    write!(name, "B{:07}", session).ok()?;
    Some(name)
}

fn ensure_directory(
    volume_manager: &VolumeManagerImpl,
    parent: RawDirectory,
    name: &str,
) -> Option<RawDirectory> {
    if let Ok(dir) = volume_manager.open_dir(parent, name) {
        info!("Found directory {}", name);
        return Some(dir);
    }

    if volume_manager.make_dir_in_dir(parent, name).is_err() {
        error!("Failed to create directory {}", name);
        return None;
    }

    info!("Created directory {}", name);

    match volume_manager.open_dir(parent, name) {
        Ok(dir) => Some(dir),
        Err(_) => {
            error!("Failed to open newly-created directory {}", name);
            None
        }
    }
}

fn allocate_boot_session(
    volume_manager: &VolumeManagerImpl,
    survey_dir: RawDirectory,
) -> Option<u32> {
    let mut recovered_from_scan = false;

    let mut next_session = match read_boot_session_counter(volume_manager, survey_dir) {
        Some(value) if value >= 1 && value <= MAX_BOOT_SESSION => {
            info!("Boot/session counter loaded: next={}", value);
            value
        }
        Some(value) => {
            warn!(
                "Boot/session counter invalid/exhausted: value={}; recovering from directory scan",
                value
            );
            recovered_from_scan = true;
            find_highest_boot_session(volume_manager, survey_dir)?
                .saturating_add(1)
        }
        None => {
            info!(
                "Boot/session counter unavailable; performing one-time directory scan"
            );
            recovered_from_scan = true;
            find_highest_boot_session(volume_manager, survey_dir)?
                .saturating_add(1)
        }
    };

    if next_session == 0 || next_session > MAX_BOOT_SESSION {
        return None;
    }

    // If BOOT.NXT was stale but syntactically valid, avoid a collision. This
    // should be rare; recover with a full directory scan and repair the counter.
    if boot_session_directory_exists(volume_manager, survey_dir, next_session) {
        warn!(
            "Boot/session counter collision at {}; recovering from directory scan",
            next_session
        );
        recovered_from_scan = true;
        next_session = find_highest_boot_session(volume_manager, survey_dir)?
            .saturating_add(1);
    }

    if next_session == 0 || next_session > MAX_BOOT_SESSION {
        return None;
    }

    let following_session = next_session.saturating_add(1);

    if !write_boot_session_counter(volume_manager, survey_dir, following_session) {
        error!("Failed to persist BOOT.NXT");
        return None;
    }

    if recovered_from_scan {
        info!(
            "Boot/session counter recovered: allocated={} next={}",
            next_session,
            following_session
        );
    } else {
        info!(
            "Boot/session allocated: current={} next={}",
            next_session,
            following_session
        );
    }

    Some(next_session)
}

fn read_boot_session_counter(
    volume_manager: &VolumeManagerImpl,
    survey_dir: RawDirectory,
) -> Option<u32> {
    let name = ShortFileName::create_from_str(BOOT_COUNTER_FILENAME).ok()?;
    let file = volume_manager
        .open_file_in_dir(survey_dir, &name, Mode::ReadOnly)
        .ok()?;

    let mut bytes = [0u8; 16];
    let result = volume_manager.read(file, &mut bytes);
    let _ = volume_manager.close_file(file);

    let len = result.ok()?;
    if len == 0 {
        return None;
    }

    let mut value: u32 = 0;
    let mut saw_digit = false;

    for byte in &bytes[..len] {
        if byte.is_ascii_digit() {
            saw_digit = true;
            value = value
                .checked_mul(10)?
                .checked_add(u32::from(*byte - b'0'))?;
        } else if matches!(*byte, b'\r' | b'\n' | b' ' | b'\t') {
            if saw_digit {
                break;
            }
        } else {
            return None;
        }
    }

    saw_digit.then_some(value)
}

fn write_boot_session_counter(
    volume_manager: &VolumeManagerImpl,
    survey_dir: RawDirectory,
    next_session: u32,
) -> bool {
    let name = match ShortFileName::create_from_str(BOOT_COUNTER_FILENAME) {
        Ok(name) => name,
        Err(_) => return false,
    };

    let file = match volume_manager.open_file_in_dir(
        survey_dir,
        &name,
        Mode::ReadWriteCreateOrTruncate,
    ) {
        Ok(file) => file,
        Err(_) => return false,
    };

    let mut contents: String<16> = String::new();
    if writeln!(contents, "{:08}", next_session).is_err() {
        let _ = volume_manager.close_file(file);
        return false;
    }

    if volume_manager.write(file, contents.as_bytes()).is_err() {
        let _ = volume_manager.close_file(file);
        return false;
    }

    if volume_manager.flush_file(file).is_err() {
        let _ = volume_manager.close_file(file);
        return false;
    }

    volume_manager.close_file(file).is_ok()
}

fn boot_session_directory_exists(
    volume_manager: &VolumeManagerImpl,
    survey_dir: RawDirectory,
    session: u32,
) -> bool {
    let Some(name) = boot_session_directory_name(session) else {
        return false;
    };

    match volume_manager.open_dir(survey_dir, name.as_str()) {
        Ok(dir) => {
            let _ = volume_manager.close_dir(dir);
            true
        }
        Err(_) => false,
    }
}

fn find_highest_boot_session(
    volume_manager: &VolumeManagerImpl,
    survey_dir: RawDirectory,
) -> Option<u32> {
    let mut highest: u32 = 0;

    if volume_manager
        .iterate_dir(survey_dir, |entry| {
            if !entry.attributes.is_directory() {
                return;
            }

            let base = entry.name.base_name();
            if base.len() != 8 || base[0] != BOOT_SESSION_PREFIX {
                return;
            }

            let mut value: u32 = 0;
            for digit in &base[1..] {
                if !digit.is_ascii_digit() {
                    return;
                }

                value = value
                    .saturating_mul(10)
                    .saturating_add(u32::from(*digit - b'0'));
            }

            if value > highest {
                highest = value;
            }
        })
        .is_err()
    {
        return None;
    }

    Some(highest)
}
