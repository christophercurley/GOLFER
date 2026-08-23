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
    spi0_bus::{self, Spi0Bus},
};

const SYSTEM_LOG_FILENAME: &str = "SYS_LOG.TXT";
const DEBUG_LOG_FILENAME: &str = "DEBUG.LOG";

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
// Current time placeholders:
//
//   timestamp   = NA
//   time_source = UNKNOWN
//
// Example:
//
//   NA UNKNOWN 13761 DEBUG STORAGE FS_APPEND_CHECKPOINT us=6636 ...
//
// This shape is intentionally parser-friendly and leaves room for the future
// GOLFER time package without sacrificing the always-available monotonic clock.
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
const TEST_SURVEY_ID: &str = "A0000001";
const RECEIVER_LOG_PREFIX: u8 = b'R';
const RECEIVER_LOG_EXTENSION: &[u8] = b"LOG";

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

    // Kept open for the lifetime of this boot/test segment.
    survey_volume: RawVolume,
    survey_dir: RawDirectory,
    telemetry_file: RawFile,

    // DEBUG.LOG lives at the card root. Failure to create/open it does NOT take
    // survey telemetry offline.
    debug_root: Option<RawDirectory>,
    debug_file: Option<RawFile>,
    debug_buffer: String<DEBUG_BUFFER_CAPACITY>,

    segment_number: u32,
    records_written: u32,
}

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
    /// Initialize SD storage and the current primitive survey segment.
    ///
    /// Survey storage failure is non-fatal to GOLFER as a whole. DEBUG.LOG
    /// failure is additionally non-fatal to survey logging.
    pub fn init(
        bus: &'static RefCell<Spi0Bus>,
        sd_cs: Output<'static>,
    ) -> Option<Self> {
        info!("Initializing GOLFER storage");

        if !spi0_bus::send_sd_startup_clocks(bus) {
            error!("SD startup clocks failed");
            return None;
        }

        let spi_device =
            match RefCellDevice::new(bus, sd_cs, Delay) {
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

        info!("SD card initialized: {} bytes", card_size);

        let volume_manager =
            VolumeManager::new(sdcard, PlaceholderTimeSource);

        if !initialize_system_log(&volume_manager) {
            warn!("SD card online, but system log initialization failed");
        }

        let (
            survey_volume,
            survey_dir,
            telemetry_file,
            segment_number,
        ) = match initialize_test_survey_log(&volume_manager) {
            Some(handles) => handles,
            None => {
                error!("Telemetry survey-log initialization failed");
                return None;
            }
        };

        // DEBUG.LOG is deliberately optional. A diagnostic-log problem must not
        // make survey telemetry unavailable.
        let (debug_root, debug_file) =
            match initialize_debug_log(
                &volume_manager,
                survey_volume,
            ) {
                Some((root, file)) => {
                    info!(
                        "Persistent DEBUG.LOG online at level {}",
                        PERSISTENT_LOG_LEVEL.label()
                    );
                    (Some(root), Some(file))
                }

                None => {
                    warn!(
                        "Persistent DEBUG.LOG unavailable; survey logging continues"
                    );
                    (None, None)
                }
            };

        info!(
            "GOLFER storage ready; test receiver segment={}",
            segment_number
        );

        Some(Self {
            volume_manager,
            survey_volume,
            survey_dir,
            telemetry_file,
            debug_root,
            debug_file,
            debug_buffer: String::new(),
            segment_number,
            records_written: 0,
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
        // <timestamp> <time_source> <uptime_ms> <level> <originator>
        // <event_id> <message...>
        //
        // Real wall-clock time is not wired into the logger yet. "NA UNKNOWN"
        // is an intentional placeholder rather than fabricated UTC.
        if write!(
            line,
            "NA UNKNOWN {} {} {} {}",
            uptime_ms,
            level.label(),
            originator,
            event_id,
        )
        .is_err()
        {
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

    /// Append one received telemetry record and periodically checkpoint it.
    ///
    /// The returned timings measure ONLY the survey telemetry operation. Any
    /// later DEBUG.LOG batch write is outside these measurements.
    pub fn log_receiver_packet(
        &mut self,
        timestamp_ms: u64,
        sequence: u32,
        rssi: i16,
        snr: i16,
        received: u32,
        missed: u32,
        gps: GpsState,
    ) -> Option<TelemetryWriteStats> {
        let mut line: String<192> = String::new();

        if write!(
            line,
            "{},SEQ={},RSSI={},SNR={},RX={},MISSED={},GPS_ONLINE={},GPS_FIX={}",
            timestamp_ms,
            sequence,
            rssi,
            snr,
            received,
            missed,
            gps.online as u8,
            gps.fix as u8,
        )
        .is_err()
        {
            error!("Telemetry line formatting overflow");
            return None;
        }

        match gps.latitude_e7 {
            Some(latitude_e7) => {
                if write!(line, ",LAT_E7={}", latitude_e7).is_err() {
                    return None;
                }
            }

            None => {
                if write!(line, ",LAT_E7=NA").is_err() {
                    return None;
                }
            }
        }

        match gps.longitude_e7 {
            Some(longitude_e7) => {
                if write!(line, ",LON_E7={}", longitude_e7).is_err() {
                    return None;
                }
            }

            None => {
                if write!(line, ",LON_E7=NA").is_err() {
                    return None;
                }
            }
        }

        match gps.satellites {
            Some(satellites) => {
                if writeln!(line, ",SATS={}", satellites).is_err() {
                    return None;
                }
            }

            None => {
                if writeln!(line, ",SATS=NA").is_err() {
                    return None;
                }
            }
        }

        let append_started = Instant::now();

        if self
            .volume_manager
            .write(self.telemetry_file, line.as_bytes())
            .is_err()
        {
            error!("Telemetry SD write failed");

            self.diag(
                PersistentLogLevel::Error,
                timestamp_ms,
                "STORAGE",
                "FS_APPEND_FAILED",
                format_args!(""),
            );

            return None;
        }

        let append_us =
            Instant::now()
                .duration_since(append_started)
                .as_micros();

        self.records_written =
            self.records_written.saturating_add(1);

        let checkpointed = self.records_written % 10 == 0;

        let checkpoint_us = if checkpointed {
            let checkpoint_started = Instant::now();

            if self
                .volume_manager
                .flush_file(self.telemetry_file)
                .is_err()
            {
                error!("Telemetry checkpoint flush failed");

                self.diag(
                    PersistentLogLevel::Error,
                    timestamp_ms,
                    "STORAGE",
                    "FS_CHECKPOINT_FAILED",
                    format_args!(""),
                );

                return None;
            }

            Some(
                Instant::now()
                    .duration_since(checkpoint_started)
                    .as_micros()
            )
        } else {
            None
        };

        let total_us =
            append_us.saturating_add(checkpoint_us.unwrap_or(0));

        // Normal append timing is TRACE because it occurs every packet.
        self.diag(
            PersistentLogLevel::Trace,
            timestamp_ms,
            "STORAGE",
            "FS_APPEND",
            format_args!(
                "us={} bytes={}",
                append_us,
                line.len()
            ),
        );

        // Checkpoint timing is lower-volume and useful enough for DEBUG.
        if let Some(checkpoint_us) = checkpoint_us {
            self.diag(
                PersistentLogLevel::Debug,
                timestamp_ms,
                "STORAGE",
                "FS_APPEND_CHECKPOINT",
                format_args!(
                    "us={} append_us={} checkpoint_us={}",
                    total_us,
                    append_us,
                    checkpoint_us
                ),
            );

            // Persist the accumulated low-priority diagnostics once per normal
            // survey checkpoint rather than touching DEBUG.LOG every second.
            self.flush_debug_buffer(true);
        }

        Some(TelemetryWriteStats {
            append_us,
            checkpoint_us,
            total_us,
        })
    }

    pub fn segment_number(&self) -> u32 {
        self.segment_number
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

        if let Some(debug_root) = self.debug_root {
            let _ = self.volume_manager.close_dir(debug_root);
        }

        let _ = self.volume_manager.flush_file(self.telemetry_file);
        let _ = self.volume_manager.close_file(self.telemetry_file);
        let _ = self.volume_manager.close_dir(self.survey_dir);
        let _ = self.volume_manager.close_volume(self.survey_volume);
    }
}

fn initialize_debug_log(
    volume_manager: &VolumeManagerImpl,
    volume: RawVolume,
) -> Option<(RawDirectory, RawFile)> {
    let root = volume_manager.open_root_dir(volume).ok()?;

    let filename =
        ShortFileName::create_from_str(DEBUG_LOG_FILENAME).ok()?;

    let mut already_exists = false;

    if volume_manager
        .iterate_dir(root, |entry| {
            if entry.name == filename {
                already_exists = true;
            }
        })
        .is_err()
    {
        let _ = volume_manager.close_dir(root);
        return None;
    }

    let mode = if already_exists {
        Mode::ReadWriteAppend
    } else {
        Mode::ReadWriteCreate
    };

    let file =
        match volume_manager.open_file_in_dir(
            root,
            &filename,
            mode,
        ) {
            Ok(file) => file,

            Err(_) => {
                let _ = volume_manager.close_dir(root);
                return None;
            }
        };

    Some((root, file))
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

/// Create/open:
///
///     /SURVEY/A0000001/
///
/// Then scan for Rxxxxxxx.LOG and create the next segment.
///
/// The returned volume, survey directory and telemetry file intentionally stay
/// open for the duration of this boot.
fn initialize_test_survey_log(
    volume_manager: &VolumeManagerImpl,
) -> Option<(RawVolume, RawDirectory, RawFile, u32)> {
    let volume = match volume_manager.open_raw_volume(VolumeIdx(0)) {
        Ok(volume) => volume,
        Err(_) => {
            error!("Failed to mount FAT volume for test survey");
            return None;
        }
    };

    let root = match volume_manager.open_root_dir(volume) {
        Ok(root) => root,
        Err(_) => {
            error!("Failed to open root for test survey");
            let _ = volume_manager.close_volume(volume);
            return None;
        }
    };

    let survey_root =
        match ensure_directory(volume_manager, root, SURVEY_ROOT_DIR) {
            Some(dir) => dir,
            None => {
                error!("Failed to ensure /SURVEY");
                let _ = volume_manager.close_dir(root);
                let _ = volume_manager.close_volume(volume);
                return None;
            }
        };

    let _ = volume_manager.close_dir(root);

    let survey_dir =
        match ensure_directory(volume_manager, survey_root, TEST_SURVEY_ID) {
            Some(dir) => dir,
            None => {
                error!("Failed to ensure /SURVEY/A0000001");
                let _ = volume_manager.close_dir(survey_root);
                let _ = volume_manager.close_volume(volume);
                return None;
            }
        };

    let _ = volume_manager.close_dir(survey_root);

    let highest_segment =
        match find_highest_receiver_segment(
            volume_manager,
            survey_dir,
        ) {
            Some(segment) => segment,
            None => {
                error!("Failed to scan survey directory");
                let _ = volume_manager.close_dir(survey_dir);
                let _ = volume_manager.close_volume(volume);
                return None;
            }
        };

    let next_segment = highest_segment.saturating_add(1);

    if next_segment > 9_999_999 {
        error!("Receiver segment number exhausted");
        let _ = volume_manager.close_dir(survey_dir);
        let _ = volume_manager.close_volume(volume);
        return None;
    }

    let filename =
        match receiver_segment_filename(next_segment) {
            Some(name) => name,
            None => {
                error!("Failed to format receiver segment filename");
                let _ = volume_manager.close_dir(survey_dir);
                let _ = volume_manager.close_volume(volume);
                return None;
            }
        };

    let telemetry_file =
        match volume_manager.open_file_in_dir(
            survey_dir,
            &filename,
            Mode::ReadWriteCreate,
        ) {
            Ok(file) => file,

            Err(_) => {
                error!("Failed to create receiver telemetry segment");
                let _ = volume_manager.close_dir(survey_dir);
                let _ = volume_manager.close_volume(volume);
                return None;
            }
        };

    info!(
        "Test survey active: /SURVEY/A0000001 segment={}",
        next_segment
    );

    Some((
        volume,
        survey_dir,
        telemetry_file,
        next_segment,
    ))
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

fn find_highest_receiver_segment(
    volume_manager: &VolumeManagerImpl,
    survey_dir: RawDirectory,
) -> Option<u32> {
    let mut highest: u32 = 0;

    if volume_manager
        .iterate_dir(survey_dir, |entry| {
            if entry.attributes.is_directory() {
                return;
            }

            if entry.name.extension() != RECEIVER_LOG_EXTENSION {
                return;
            }

            let base = entry.name.base_name();

            if base.len() != 8 || base[0] != RECEIVER_LOG_PREFIX {
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

fn receiver_segment_filename(
    segment: u32,
) -> Option<ShortFileName> {
    let mut filename: String<13> = String::new();

    if write!(
        filename,
        "R{:07}.LOG",
        segment
    )
    .is_err()
    {
        return None;
    }

    ShortFileName::create_from_str(filename.as_str()).ok()
}
