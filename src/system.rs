use core::{fmt::Write as _, str};

use defmt::{error, info, warn};
use embassy_rp::{
    flash::{Blocking, Flash, ERASE_SIZE, PAGE_SIZE},
    peripherals::FLASH,
    Peri,
};
use heapless::String;

/// Firmware version from Cargo.toml's `[package].version`.
pub const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");

/// GOLFER-to-GOLFER protocol version.
///
/// Stubbed for now. This becomes meaningful when the USB/radio protocol is
/// formalized and versioned.
pub const PROTOCOL_VERSION: u16 = 1;

/// Persistent system-configuration schema version.
///
/// Version 1 currently contains only the user-editable GOLFER name.
pub const CONFIG_VERSION: u16 = 1;

/// Maximum UTF-8 byte length of a user-facing GOLFER name.
pub const MAX_NAME_BYTES: usize = 32;

/// Physical Pico 2 flash capacity: 4 MiB.
const FLASH_SIZE_BYTES: usize = 4 * 1024 * 1024;

/// The linker exposes only the first 4032 KiB to firmware. The upper 64 KiB is
/// reserved for GOLFER-owned persistent data and RP2350 end-of-flash safety.
///
/// Keep this in sync with memory.x.
const PERSISTENT_REGION_OFFSET: u32 = 4032 * 1024;

/// Two independent erase sectors are used as A/B config slots. A new config is
/// written to the inactive slot and verified before it becomes authoritative.
const CONFIG_SLOT_A_OFFSET: u32 = PERSISTENT_REGION_OFFSET;
const CONFIG_SLOT_B_OFFSET: u32 = PERSISTENT_REGION_OFFSET + ERASE_SIZE as u32;
const CONFIG_SLOT_SIZE: u32 = ERASE_SIZE as u32;

/// Fixed on-flash record layout. One page is plenty for today's config and
/// keeps writes simple/aligned even though embassy-rp supports smaller writes.
const CONFIG_RECORD_SIZE: usize = PAGE_SIZE;
const CONFIG_MAGIC: &[u8; 4] = b"GLFC";
const CONFIG_RECORD_FORMAT: u8 = 1;

const MAGIC_OFFSET: usize = 0;
const SCHEMA_OFFSET: usize = 4;
const NAME_LEN_OFFSET: usize = 6;
const FORMAT_OFFSET: usize = 7;
const GENERATION_OFFSET: usize = 8;
const NAME_OFFSET: usize = 12;
const CRC_OFFSET: usize = NAME_OFFSET + MAX_NAME_BYTES;
const CRC_END: usize = CRC_OFFSET + 4;

const _: () = assert!(CRC_END <= CONFIG_RECORD_SIZE);

type FlashDriver = Flash<'static, FLASH, Blocking, FLASH_SIZE_BYTES>;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum FieldAccess {
    ReadOnly,
    ReadWrite,
}

impl FieldAccess {
    pub const fn tag(self) -> &'static str {
        match self {
            Self::ReadOnly => "RO",
            Self::ReadWrite => "RW",
        }
    }
}

/// A system/config value plus its schema-level access policy.
///
/// The access flag is deliberately part of the data model so the upcoming USB
/// protocol can report which fields are mutable without duplicating that policy
/// in the CLI.
#[derive(Clone, Copy)]
pub struct SystemField<T> {
    pub value: T,
    pub access: FieldAccess,
}

impl<T> SystemField<T> {
    pub const fn read_only(value: T) -> Self {
        Self {
            value,
            access: FieldAccess::ReadOnly,
        }
    }

    pub const fn read_write(value: T) -> Self {
        Self {
            value,
            access: FieldAccess::ReadWrite,
        }
    }
}

/// Immutable identity/version information for this physical GOLFER.
///
/// `system_id` is the canonical GOLFER identity and comes directly from the
/// RP2350's random 64-bit chip ID stored in OTP. It never changes when the user
/// renames the device.
#[derive(Clone, Copy)]
pub struct SystemInfo {
    pub system_id: SystemField<u64>,
    pub firmware_version: SystemField<&'static str>,
    pub protocol_version: SystemField<u16>,
    pub config_version: SystemField<u16>,
}

/// User-editable system configuration.
#[derive(Clone)]
pub struct SystemConfig {
    pub name: SystemField<String<MAX_NAME_BYTES>>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ConfigSlot {
    A,
    B,
}

impl ConfigSlot {
    const fn offset(self) -> u32 {
        match self {
            Self::A => CONFIG_SLOT_A_OFFSET,
            Self::B => CONFIG_SLOT_B_OFFSET,
        }
    }

    const fn other(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::A => "A",
            Self::B => "B",
        }
    }
}

struct StoredConfig {
    slot: ConfigSlot,
    generation: u32,
    config: SystemConfig,
}

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum SystemError {
    NameEmpty,
    NameTooLong,
    NameContainsControl,
    FlashErase,
    FlashWrite,
    FlashVerify,
}

/// Owns GOLFER identity, editable configuration, and its flash persistence.
///
/// Callers can query `info()` / `config()`. Mutations go through methods such as
/// `set_name()` so RAM state and persistent state cannot silently diverge.
pub struct System {
    info: SystemInfo,
    config: SystemConfig,
    flash: FlashDriver,
    active_slot: Option<ConfigSlot>,
    generation: u32,
}

impl System {
    pub fn info(&self) -> SystemInfo {
        self.info
    }

    pub fn config(&self) -> &SystemConfig {
        &self.config
    }

    /// Change the user-facing GOLFER name and persist it atomically enough for
    /// field use: the previous valid A/B slot is retained until the new slot has
    /// been written and verified.
    #[allow(dead_code)]
    pub fn set_name(&mut self, name: &str) -> Result<(), SystemError> {
        let new_name = make_name(name)?;

        if self.config.name.value == new_name {
            return Ok(());
        }

        let previous = self.config.clone();
        self.config.name.value = new_name;

        if let Err(err) = self.persist_config() {
            self.config = previous;
            return Err(err);
        }

        info!("GOLFER name changed to: {}", self.config.name.value.as_str());
        Ok(())
    }

    fn persist_config(&mut self) -> Result<(), SystemError> {
        let target_slot = self.active_slot.map(ConfigSlot::other).unwrap_or(ConfigSlot::A);
        let next_generation = self.generation.wrapping_add(1).max(1);

        let record = encode_config_record(&self.config, next_generation);
        let offset = target_slot.offset();

        self.flash
            .blocking_erase(offset, offset + CONFIG_SLOT_SIZE)
            .map_err(|_| SystemError::FlashErase)?;

        self.flash
            .blocking_write(offset, &record)
            .map_err(|_| SystemError::FlashWrite)?;

        let verified = read_slot(&mut self.flash, target_slot).ok_or(SystemError::FlashVerify)?;

        if verified.generation != next_generation
            || verified.config.name.value != self.config.name.value
        {
            return Err(SystemError::FlashVerify);
        }

        self.active_slot = Some(target_slot);
        self.generation = next_generation;

        info!(
            "System config persisted: slot={} generation={} name={}",
            target_slot.label(),
            next_generation,
            self.config.name.value.as_str()
        );

        Ok(())
    }
}

/// Initialize GOLFER's immutable identity and load/create persistent config.
pub fn init(flash_peripheral: Peri<'static, FLASH>) -> System {
    let system_id = match embassy_rp::otp::get_chipid() {
        Ok(id) => id,
        Err(_) => {
            error!("Failed to read RP2350 hardware ID from OTP");
            panic!("RP2350 hardware ID unavailable");
        }
    };

    let info = SystemInfo {
        system_id: SystemField::read_only(system_id),
        firmware_version: SystemField::read_only(FIRMWARE_VERSION),
        protocol_version: SystemField::read_only(PROTOCOL_VERSION),
        config_version: SystemField::read_only(CONFIG_VERSION),
    };

    let mut flash = Flash::<_, Blocking, FLASH_SIZE_BYTES>::new_blocking(flash_peripheral);

    let slot_a = read_slot(&mut flash, ConfigSlot::A);
    let slot_b = read_slot(&mut flash, ConfigSlot::B);

    let stored = newest_slot(slot_a, slot_b);

    let (config, active_slot, generation) = if let Some(stored) = stored {
        info!(
            "Persistent system config loaded: slot={} generation={}",
            stored.slot.label(),
            stored.generation
        );

        (stored.config, Some(stored.slot), stored.generation)
    } else {
        let default_name = default_name(system_id);

        info!("No valid persistent system config found");
        info!("Creating default GOLFER name: {}", default_name.as_str());

        (
            SystemConfig {
                name: SystemField::read_write(default_name),
            },
            None,
            0,
        )
    };

    let mut system = System {
        info,
        config,
        flash,
        active_slot,
        generation,
    };

    // A factory-fresh device starts with the derived default name and immediately
    // commits it. Subsequent boots should load the same config without rewriting.
    if system.active_slot.is_none() {
        if system.persist_config().is_err() {
            warn!("Failed to persist initial system config; continuing with RAM default");
        }
    }

    log_system(&system);
    system
}

fn newest_slot(a: Option<StoredConfig>, b: Option<StoredConfig>) -> Option<StoredConfig> {
    match (a, b) {
        (Some(a), Some(b)) => {
            if b.generation > a.generation {
                Some(b)
            } else {
                Some(a)
            }
        }
        (Some(a), None) => Some(a),
        (None, Some(b)) => Some(b),
        (None, None) => None,
    }
}

fn read_slot(flash: &mut FlashDriver, slot: ConfigSlot) -> Option<StoredConfig> {
    let mut record = [0u8; CONFIG_RECORD_SIZE];

    if flash.blocking_read(slot.offset(), &mut record).is_err() {
        warn!("Failed to read system config slot {}", slot.label());
        return None;
    }

    decode_config_record(&record).map(|(generation, config)| StoredConfig {
        slot,
        generation,
        config,
    })
}

fn encode_config_record(
    config: &SystemConfig,
    generation: u32,
) -> [u8; CONFIG_RECORD_SIZE] {
    let mut record = [0xFFu8; CONFIG_RECORD_SIZE];
    let name = config.name.value.as_bytes();

    record[MAGIC_OFFSET..MAGIC_OFFSET + 4].copy_from_slice(CONFIG_MAGIC);
    record[SCHEMA_OFFSET..SCHEMA_OFFSET + 2].copy_from_slice(&CONFIG_VERSION.to_le_bytes());
    record[NAME_LEN_OFFSET] = name.len() as u8;
    record[FORMAT_OFFSET] = CONFIG_RECORD_FORMAT;
    record[GENERATION_OFFSET..GENERATION_OFFSET + 4]
        .copy_from_slice(&generation.to_le_bytes());
    record[NAME_OFFSET..NAME_OFFSET + name.len()].copy_from_slice(name);

    let crc = crc32(&record[..CRC_OFFSET]);
    record[CRC_OFFSET..CRC_END].copy_from_slice(&crc.to_le_bytes());

    record
}

fn decode_config_record(
    record: &[u8; CONFIG_RECORD_SIZE],
) -> Option<(u32, SystemConfig)> {
    if &record[MAGIC_OFFSET..MAGIC_OFFSET + 4] != CONFIG_MAGIC {
        return None;
    }

    let schema = u16::from_le_bytes([
        record[SCHEMA_OFFSET],
        record[SCHEMA_OFFSET + 1],
    ]);

    if schema != CONFIG_VERSION || record[FORMAT_OFFSET] != CONFIG_RECORD_FORMAT {
        return None;
    }

    let stored_crc = u32::from_le_bytes([
        record[CRC_OFFSET],
        record[CRC_OFFSET + 1],
        record[CRC_OFFSET + 2],
        record[CRC_OFFSET + 3],
    ]);

    if stored_crc != crc32(&record[..CRC_OFFSET]) {
        return None;
    }

    let name_len = record[NAME_LEN_OFFSET] as usize;

    if name_len == 0 || name_len > MAX_NAME_BYTES {
        return None;
    }

    let name_text = str::from_utf8(&record[NAME_OFFSET..NAME_OFFSET + name_len]).ok()?;
    let name = make_name(name_text).ok()?;

    let generation = u32::from_le_bytes([
        record[GENERATION_OFFSET],
        record[GENERATION_OFFSET + 1],
        record[GENERATION_OFFSET + 2],
        record[GENERATION_OFFSET + 3],
    ]);

    Some((
        generation,
        SystemConfig {
            name: SystemField::read_write(name),
        },
    ))
}

fn make_name(name: &str) -> Result<String<MAX_NAME_BYTES>, SystemError> {
    if name.is_empty() || name.chars().all(char::is_whitespace) {
        return Err(SystemError::NameEmpty);
    }

    if name.len() > MAX_NAME_BYTES {
        return Err(SystemError::NameTooLong);
    }

    if name.chars().any(char::is_control) {
        return Err(SystemError::NameContainsControl);
    }

    let mut value = String::new();
    value
        .push_str(name)
        .map_err(|_| SystemError::NameTooLong)?;

    Ok(value)
}

fn default_name(system_id: u64) -> String<MAX_NAME_BYTES> {
    let mut name = String::new();
    write!(&mut name, "{:08X}", system_id as u32).unwrap();
    name
}

/// Small dependency-free CRC-32 (IEEE) for detecting incomplete/corrupt config
/// records. This is integrity checking, not cryptographic authentication.
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;

    for &byte in data {
        crc ^= u32::from(byte);

        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }

    !crc
}

fn log_system(system: &System) {
    let mut system_id: String<16> = String::new();
    write!(&mut system_id, "{:016X}", system.info.system_id.value).unwrap();

    info!("GOLFER system initialized");
    info!(
        "Name:        {} [{}]",
        system.config.name.value.as_str(),
        system.config.name.access.tag()
    );
    info!(
        "System ID:   {} [{}]",
        system_id.as_str(),
        system.info.system_id.access.tag()
    );
    info!(
        "Firmware:    {} [{}]",
        system.info.firmware_version.value,
        system.info.firmware_version.access.tag()
    );
    info!(
        "Protocol:    {} [{}]",
        system.info.protocol_version.value,
        system.info.protocol_version.access.tag()
    );
    info!(
        "Config:      {} [{}]",
        system.info.config_version.value,
        system.info.config_version.access.tag()
    );
}
