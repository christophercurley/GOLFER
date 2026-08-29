// -----------------------------------------------------------------------------
// NATIVE GOLFER RF PACKET FORMAT
//
// TelemetryV1 is a fixed-width 48-byte binary packet designed to represent a
// realistic GOLFER measurement while keeping the current SF7/BW125/CR4/5 LoRa
// airtime below the project's ~100 ms target.
//
// IMPORTANT DESIGN RULES
//
// * Bytes are encoded explicitly. Never transmit a Rust struct's raw memory.
// * Multi-byte integers are little-endian.
// * Survey ID is the persistent survey identity. Boots/restarts do NOT change it.
// * Sequence is transport bookkeeping only and may restart after a reboot.
// * The application CRC32 is independent of the SX1262 PHY CRC and exists as
//   defense-in-depth, including against the current lora-phy CRC-error leak.
// * Missing sensor values use explicit on-air sentinels but appear as Option<T>
//   to the rest of the firmware.
// -----------------------------------------------------------------------------

pub const TELEMETRY_V1_LEN: usize = 48;
pub const TELEMETRY_V1_CRC_OFFSET: usize = 44;

pub const PROTOCOL_MARKER: u8 = b'G';
pub const PROTOCOL_VERSION: u8 = 1;

pub const PACKET_TYPE_TELEMETRY: u8 = 1;

/// Sender-mode nibble values known today.
///
/// The field is intentionally carried as a raw 4-bit value in TelemetryV1 so
/// future firmware can add modes without changing the packet layout.
pub mod sender_mode {
    pub const UNSPECIFIED: u8 = 0;
    pub const BEACON: u8 = 1;
    pub const RECEIVER: u8 = 2;
    pub const SURVEYING: u8 = 3;
}

pub const MAX_SEQUENCE: u32 = 0x00FF_FFFF;

const UNKNOWN_U32: u32 = u32::MAX;
const UNKNOWN_I32: i32 = i32::MIN;
const UNKNOWN_U16: u16 = u16::MAX;
const UNKNOWN_I16: i16 = i16::MIN;
const UNKNOWN_U8: u8 = u8::MAX;

const MAX_LATITUDE_E7: i32 = 900_000_000;
const MAX_LONGITUDE_E7: i32 = 1_800_000_000;
const MAX_COURSE_CDEG: u16 = 35_999;
const MAX_HUMIDITY_HALF_PERCENT: u8 = 200;
const MAX_BATTERY_SOC_PERCENT: u8 = 100;
const MAX_SATELLITES: u8 = 63;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum GpsFixClass {
    NoFix = 0,
    Standard = 1,
    Differential = 2,
    EnhancedOrOther = 3,
}

impl GpsFixClass {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0 => Self::NoFix,
            1 => Self::Standard,
            2 => Self::Differential,
            _ => Self::EnhancedOrOther,
        }
    }
}

/// Native GOLFER telemetry packet, version 1.
///
/// On-air layout:
///
///   00      protocol marker ('G')
///   01      protocol version (1)
///   02      packet type [7:4] | sender mode [3:0]
///   03..10  sender System ID, u64 LE
///   11..14  Survey ID, u32 LE
///   15..17  sequence, unsigned 24-bit LE
///   18..21  GPS UTC as Unix seconds, u32 LE
///   22..25  latitude, signed degrees * 1e7, i32 LE
///   26..29  longitude, signed degrees * 1e7, i32 LE
///   30..31  GPS altitude, signed 0.5 m units, i16 LE
///   32..33  GPS speed over ground, cm/s, u16 LE
///   34..35  GPS course over ground, degrees * 100, u16 LE
///   36      GPS fix class [7:6] | satellites [5:0]
///   37      HDOP * 10, u8
///   38..39  BME280 temperature, degrees C * 100, i16 LE
///   40..41  BME280 pressure, 10 Pa units, u16 LE
///   42      BME280 humidity, 0.5 %RH units, u8
///   43      battery state of charge, percent, u8
///   44..47  CRC-32/IEEE over bytes 0..43, u32 LE
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryV1 {
    pub sender_mode: u8,
    pub sender_system_id: u64,
    pub survey_id: u32,
    pub sequence: u32,

    pub gps_unix_time: Option<u32>,
    pub latitude_e7: Option<i32>,
    pub longitude_e7: Option<i32>,
    pub altitude_half_m: Option<i16>,
    pub speed_cm_s: Option<u16>,
    pub course_cdeg: Option<u16>,
    pub gps_fix_class: GpsFixClass,
    pub satellites: u8,
    pub hdop_tenths: Option<u8>,

    pub temperature_centi_c: Option<i16>,
    pub pressure_10pa: Option<u16>,
    pub humidity_half_percent: Option<u8>,

    pub battery_soc_percent: Option<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    SenderModeOutOfRange,
    SequenceOutOfRange,
    LatitudeOutOfRange,
    LongitudeOutOfRange,
    AltitudeUsesReservedSentinel,
    SpeedUsesReservedSentinel,
    CourseOutOfRange,
    TooManySatellites,
    HdopUsesReservedSentinel,
    TemperatureUsesReservedSentinel,
    PressureUsesReservedSentinel,
    HumidityOutOfRange,
    BatterySocOutOfRange,
    GpsTimeUsesReservedSentinel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    WrongLength,
    BadMarker,
    UnsupportedVersion,
    UnsupportedPacketType,
    CrcMismatch,
    InvalidField,
}

impl TelemetryV1 {
    pub fn encode(&self) -> Result<[u8; TELEMETRY_V1_LEN], EncodeError> {
        self.validate()?;

        let mut out = [0u8; TELEMETRY_V1_LEN];

        out[0] = PROTOCOL_MARKER;
        out[1] = PROTOCOL_VERSION;
        out[2] = (PACKET_TYPE_TELEMETRY << 4) | (self.sender_mode & 0x0F);

        out[3..11].copy_from_slice(&self.sender_system_id.to_le_bytes());
        out[11..15].copy_from_slice(&self.survey_id.to_le_bytes());

        let sequence = self.sequence.to_le_bytes();
        out[15..18].copy_from_slice(&sequence[..3]);

        out[18..22].copy_from_slice(
            &self.gps_unix_time.unwrap_or(UNKNOWN_U32).to_le_bytes(),
        );

        out[22..26].copy_from_slice(
            &self.latitude_e7.unwrap_or(UNKNOWN_I32).to_le_bytes(),
        );

        out[26..30].copy_from_slice(
            &self.longitude_e7.unwrap_or(UNKNOWN_I32).to_le_bytes(),
        );

        out[30..32].copy_from_slice(
            &self.altitude_half_m.unwrap_or(UNKNOWN_I16).to_le_bytes(),
        );

        out[32..34].copy_from_slice(
            &self.speed_cm_s.unwrap_or(UNKNOWN_U16).to_le_bytes(),
        );

        out[34..36].copy_from_slice(
            &self.course_cdeg.unwrap_or(UNKNOWN_U16).to_le_bytes(),
        );

        out[36] = ((self.gps_fix_class as u8) << 6) | (self.satellites & 0x3F);
        out[37] = self.hdop_tenths.unwrap_or(UNKNOWN_U8);

        out[38..40].copy_from_slice(
            &self.temperature_centi_c.unwrap_or(UNKNOWN_I16).to_le_bytes(),
        );

        out[40..42].copy_from_slice(
            &self.pressure_10pa.unwrap_or(UNKNOWN_U16).to_le_bytes(),
        );

        out[42] = self.humidity_half_percent.unwrap_or(UNKNOWN_U8);
        out[43] = self.battery_soc_percent.unwrap_or(UNKNOWN_U8);

        let crc = crc32_ieee(&out[..TELEMETRY_V1_CRC_OFFSET]);
        out[TELEMETRY_V1_CRC_OFFSET..TELEMETRY_V1_LEN]
            .copy_from_slice(&crc.to_le_bytes());

        Ok(out)
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() != TELEMETRY_V1_LEN {
            return Err(DecodeError::WrongLength);
        }

        if bytes[0] != PROTOCOL_MARKER {
            return Err(DecodeError::BadMarker);
        }

        if bytes[1] != PROTOCOL_VERSION {
            return Err(DecodeError::UnsupportedVersion);
        }

        let packet_type = bytes[2] >> 4;

        if packet_type != PACKET_TYPE_TELEMETRY {
            return Err(DecodeError::UnsupportedPacketType);
        }

        let expected_crc = read_u32_le(&bytes[44..48]);
        let actual_crc = crc32_ieee(&bytes[..TELEMETRY_V1_CRC_OFFSET]);

        if expected_crc != actual_crc {
            return Err(DecodeError::CrcMismatch);
        }

        let sequence = u32::from(bytes[15])
            | (u32::from(bytes[16]) << 8)
            | (u32::from(bytes[17]) << 16);

        let gps_meta = bytes[36];

        let packet = Self {
            sender_mode: bytes[2] & 0x0F,
            sender_system_id: read_u64_le(&bytes[3..11]),
            survey_id: read_u32_le(&bytes[11..15]),
            sequence,

            gps_unix_time: decode_optional_u32(read_u32_le(&bytes[18..22])),
            latitude_e7: decode_optional_i32(read_i32_le(&bytes[22..26])),
            longitude_e7: decode_optional_i32(read_i32_le(&bytes[26..30])),
            altitude_half_m: decode_optional_i16(read_i16_le(&bytes[30..32])),
            speed_cm_s: decode_optional_u16(read_u16_le(&bytes[32..34])),
            course_cdeg: decode_optional_u16(read_u16_le(&bytes[34..36])),
            gps_fix_class: GpsFixClass::from_bits(gps_meta >> 6),
            satellites: gps_meta & 0x3F,
            hdop_tenths: decode_optional_u8(bytes[37]),

            temperature_centi_c: decode_optional_i16(read_i16_le(&bytes[38..40])),
            pressure_10pa: decode_optional_u16(read_u16_le(&bytes[40..42])),
            humidity_half_percent: decode_optional_u8(bytes[42]),

            battery_soc_percent: decode_optional_u8(bytes[43]),
        };

        packet
            .validate()
            .map_err(|_| DecodeError::InvalidField)?;

        Ok(packet)
    }

    fn validate(&self) -> Result<(), EncodeError> {
        if self.sender_mode > 0x0F {
            return Err(EncodeError::SenderModeOutOfRange);
        }

        if self.sequence > MAX_SEQUENCE {
            return Err(EncodeError::SequenceOutOfRange);
        }

        if matches!(self.gps_unix_time, Some(UNKNOWN_U32)) {
            return Err(EncodeError::GpsTimeUsesReservedSentinel);
        }

        if let Some(latitude) = self.latitude_e7 {
            if !(-MAX_LATITUDE_E7..=MAX_LATITUDE_E7).contains(&latitude) {
                return Err(EncodeError::LatitudeOutOfRange);
            }
        }

        if let Some(longitude) = self.longitude_e7 {
            if !(-MAX_LONGITUDE_E7..=MAX_LONGITUDE_E7).contains(&longitude) {
                return Err(EncodeError::LongitudeOutOfRange);
            }
        }

        if matches!(self.altitude_half_m, Some(UNKNOWN_I16)) {
            return Err(EncodeError::AltitudeUsesReservedSentinel);
        }

        if matches!(self.speed_cm_s, Some(UNKNOWN_U16)) {
            return Err(EncodeError::SpeedUsesReservedSentinel);
        }

        if let Some(course) = self.course_cdeg {
            if course > MAX_COURSE_CDEG {
                return Err(EncodeError::CourseOutOfRange);
            }
        }

        if self.satellites > MAX_SATELLITES {
            return Err(EncodeError::TooManySatellites);
        }

        if matches!(self.hdop_tenths, Some(UNKNOWN_U8)) {
            return Err(EncodeError::HdopUsesReservedSentinel);
        }

        if matches!(self.temperature_centi_c, Some(UNKNOWN_I16)) {
            return Err(EncodeError::TemperatureUsesReservedSentinel);
        }

        if matches!(self.pressure_10pa, Some(UNKNOWN_U16)) {
            return Err(EncodeError::PressureUsesReservedSentinel);
        }

        if let Some(humidity) = self.humidity_half_percent {
            if humidity > MAX_HUMIDITY_HALF_PERCENT {
                return Err(EncodeError::HumidityOutOfRange);
            }
        }

        if let Some(soc) = self.battery_soc_percent {
            if soc > MAX_BATTERY_SOC_PERCENT {
                return Err(EncodeError::BatterySocOutOfRange);
            }
        }

        Ok(())
    }
}

/// Golden packet used to prove that independent implementations agree on the
/// exact byte-level format. The temporary nRF transmitter should eventually
/// reproduce this vector byte-for-byte in its own codec test.
pub const GOLDEN_TELEMETRY_V1: [u8; TELEMETRY_V1_LEN] = [
    0x47, 0x01, 0x11, 0x88, 0x77, 0x66, 0x55, 0x44,
    0x33, 0x22, 0x11, 0x42, 0x00, 0x00, 0xA0, 0x56,
    0x34, 0x12, 0x03, 0x02, 0x01, 0x69, 0xAA, 0xBF,
    0xC7, 0x14, 0xD4, 0x35, 0x0E, 0xCF, 0xF7, 0x00,
    0xD2, 0x04, 0xF3, 0x69, 0x4A, 0x0D, 0xE6, 0x09,
    0x94, 0x27, 0x5D, 0x48, 0x0A, 0x21, 0x1F, 0x98,
];

/// Lightweight on-device codec check used during protocol bring-up.
///
/// It verifies:
///   * encoding produces the canonical 48-byte golden vector
///   * decoding reconstructs the source model exactly
///   * one flipped payload bit is rejected by the application CRC32
pub fn golden_self_test() -> bool {
    let packet = TelemetryV1 {
        sender_mode: sender_mode::BEACON,
        sender_system_id: 0x1122_3344_5566_7788,
        survey_id: 0xA000_0042,
        sequence: 0x0012_3456,

        gps_unix_time: Some(0x6901_0203),
        latitude_e7: Some(348_635_050),
        longitude_e7: Some(-821_152_300),
        altitude_half_m: Some(247),
        speed_cm_s: Some(1_234),
        course_cdeg: Some(27_123),
        gps_fix_class: GpsFixClass::Standard,
        satellites: 10,
        hdop_tenths: Some(13),

        temperature_centi_c: Some(2_534),
        pressure_10pa: Some(10_132),
        humidity_half_percent: Some(93),

        battery_soc_percent: Some(72),
    };

    let encoded = match packet.encode() {
        Ok(encoded) => encoded,
        Err(_) => return false,
    };

    if encoded != GOLDEN_TELEMETRY_V1 {
        return false;
    }

    let decoded = match TelemetryV1::decode(&GOLDEN_TELEMETRY_V1) {
        Ok(decoded) => decoded,
        Err(_) => return false,
    };

    if decoded != packet {
        return false;
    }

    let mut corrupted = GOLDEN_TELEMETRY_V1;
    corrupted[22] ^= 0x01;

    matches!(
        TelemetryV1::decode(&corrupted),
        Err(DecodeError::CrcMismatch)
    )
}

/// CRC-32/IEEE, matching the integrity algorithm already used by GOLFER's
/// persistent system configuration. This is an integrity check, not a
/// cryptographic authentication mechanism.
pub fn crc32_ieee(data: &[u8]) -> u32 {
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

fn decode_optional_u8(value: u8) -> Option<u8> {
    (value != UNKNOWN_U8).then_some(value)
}

fn decode_optional_u16(value: u16) -> Option<u16> {
    (value != UNKNOWN_U16).then_some(value)
}

fn decode_optional_i16(value: i16) -> Option<i16> {
    (value != UNKNOWN_I16).then_some(value)
}

fn decode_optional_u32(value: u32) -> Option<u32> {
    (value != UNKNOWN_U32).then_some(value)
}

fn decode_optional_i32(value: i32) -> Option<i32> {
    (value != UNKNOWN_I32).then_some(value)
}

fn read_u16_le(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}

fn read_i16_le(bytes: &[u8]) -> i16 {
    i16::from_le_bytes([bytes[0], bytes[1]])
}

fn read_u32_le(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

fn read_i32_le(bytes: &[u8]) -> i32 {
    i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

fn read_u64_le(bytes: &[u8]) -> u64 {
    u64::from_le_bytes([
        bytes[0], bytes[1], bytes[2], bytes[3],
        bytes[4], bytes[5], bytes[6], bytes[7],
    ])
}
