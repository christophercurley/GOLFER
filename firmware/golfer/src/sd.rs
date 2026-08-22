use defmt::{info, warn};

use embassy_rp::{
    gpio::{Level, Output},
    peripherals::{PIN_16, PIN_17, PIN_18, PIN_19, PIN_22, SPI0},
    spi::{Config as SpiConfig, Spi},
    Peri,
};
use embassy_time::Timer;

// SD cards must be initialized at a slow SPI clock.
// 400 kHz is the conventional upper limit during card initialization.
const SD_INIT_SPI_HZ: u32 = 400_000;

// CMD0: GO_IDLE_STATE
//
// 0x40 = command index 0 with the SD SPI command prefix.
// Argument is zero.
// 0x95 is the required valid CRC for CMD0 while entering SPI mode.
const CMD0: [u8; 6] = [
    0x40,
    0x00,
    0x00,
    0x00,
    0x00,
    0x95,
];

/// Minimal SD-card electrical / SPI probe.
///
/// This intentionally does NOT initialize the filesystem or even fully
/// initialize the SD card. It proves only that:
///
///   Pico SPI0 -> SD card communication works
///   SD card -> Pico MISO communication works
///   SD chip select works
///   the card can enter SPI idle state via CMD0
///
/// Expected successful CMD0 R1 response: 0x01.
///
/// The TFT shares SPI0, so its CS is explicitly held HIGH for the entire probe.
pub async fn probe(
    spi0: Peri<'static, SPI0>,
    sck: Peri<'static, PIN_18>,
    mosi: Peri<'static, PIN_19>,
    miso: Peri<'static, PIN_16>,
    tft_cs_pin: Peri<'static, PIN_17>,
    sd_cs_pin: Peri<'static, PIN_22>,
) -> bool {
    info!("SD probe starting");

    // Both SPI devices must begin deselected.
    let _tft_cs = Output::new(tft_cs_pin, Level::High);
    let mut sd_cs = Output::new(sd_cs_pin, Level::High);

    let mut config = SpiConfig::default();
    config.frequency = SD_INIT_SPI_HZ;

    let mut spi = Spi::new_blocking(
        spi0,
        sck,
        mosi,
        miso,
        config,
    );

    // Give the powered card a moment before beginning the SPI-mode sequence.
    Timer::after_millis(5).await;

    // SD SPI-mode entry requires at least 74 clocks with CS HIGH and MOSI HIGH.
    // Ten 0xFF bytes = 80 clocks.
    let startup_clocks = [0xFFu8; 10];

    if spi.blocking_write(&startup_clocks).is_err() {
        warn!("SD probe: failed while sending startup clocks");
        return false;
    }

    // A few cards can be slightly stubborn at power-up, so try CMD0 more than
    // once before declaring the wiring/card dead.
    for attempt in 1..=8 {
        sd_cs.set_low();

        // One idle byte after asserting CS.
        if spi.blocking_write(&[0xFF]).is_err() {
            sd_cs.set_high();
            warn!("SD probe: SPI write failed");
            return false;
        }

        if spi.blocking_write(&CMD0).is_err() {
            sd_cs.set_high();
            warn!("SD probe: CMD0 write failed");
            return false;
        }

        // The card may take several byte-times before returning its R1 response.
        let mut response = 0xFFu8;

        for _ in 0..16 {
            let mut byte = [0xFFu8];

            if spi.blocking_transfer_in_place(&mut byte).is_err() {
                sd_cs.set_high();
                warn!("SD probe: SPI read failed");
                return false;
            }

            // An R1 response always has bit 7 clear.
            if byte[0] & 0x80 == 0 {
                response = byte[0];
                break;
            }
        }

        sd_cs.set_high();

        // Supply another 8 clocks with the card deselected.
        let _ = spi.blocking_write(&[0xFF]);

        info!(
            "SD probe CMD0 attempt {} response: {=u8:#04x}",
            attempt,
            response
        );

        if response == 0x01 {
            info!("SD CARD DETECTED: CMD0 entered SPI idle state");
            return true;
        }

        Timer::after_millis(2).await;
    }

    warn!("SD probe FAILED: no 0x01 response to CMD0");
    false
}
