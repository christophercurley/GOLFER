#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, DMA_CH1},
    spi::{Config as SpiConfig, Spi},
};
use embassy_time::{Delay, Timer};

use embedded_hal_bus::spi::ExclusiveDevice;

use lora_phy::{
    LoRa, RxMode,
    iv::GenericSx126xInterfaceVariant,
    mod_params::{Bandwidth, CodingRate, SpreadingFactor},
    sx126x::{self, Sx126x, Sx1262, TcxoCtrlVoltage},
};

use panic_probe as _;

use core::fmt::Write as _;

use embassy_rp::i2c::{Config as I2cConfig, I2c};

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use heapless::String;

use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

const LORA_FREQUENCY_HZ: u32 = 915_000_000;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Pico 2 onboard LED.
    // In RX mode we'll pulse this briefly whenever a packet arrives.
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

    let interface = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

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
    // These exactly match the nRF52840 transmitter:
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
    //
    // RX additionally specifies the maximum payload length.
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
        .prepare_for_rx(RxMode::Continuous, &modulation_params, &rx_packet_params)
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
    // Receive packets forever
    // -------------------------------------------------------------------------

    let mut last_sequence: Option<u32> = None;
    let mut received_packets: u32 = 0;
    let mut missed_packets: u32 = 0;

    loop {
        rx_buffer.fill(0);

        match lora.rx(&rx_packet_params, &mut rx_buffer).await {
            Ok((received_len, packet_status)) => {
                let len = received_len as usize;

                info!("RX packet!");
                info!("Length: {}", received_len);
                info!("RSSI: {} dBm", packet_status.rssi);
                info!("SNR: {} dB", packet_status.snr);

                if len >= 10 && &rx_buffer[0..4] == b"MRU1" {
                    let sequence = u32::from_le_bytes([
                        rx_buffer[6],
                        rx_buffer[7],
                        rx_buffer[8],
                        rx_buffer[9],
                    ]);

                    received_packets += 1;

                    if let Some(last) = last_sequence {
                        let expected = last.wrapping_add(1);

                        if sequence == expected {
                            // Perfectly sequential packet.
                        } else if sequence == last {
                            info!("Duplicate packet: seq={}", sequence);
                        } else if sequence > last {
                            let missed = sequence - last - 1;

                            missed_packets += missed;

                            info!(
                                "PACKET LOSS: missed {} packet(s) between seq={} and seq={}",
                                missed, last, sequence
                            );
                        } else {
                            // Most likely transmitter reboot/reset rather than billions
                            // of suddenly-lost packets.
                            info!(
                                "Sequence reset/out-of-order: last={} current={}",
                                last, sequence
                            );
                        }
                    }

                    last_sequence = Some(sequence);

                    // -------------------------------------------------------------------------
                    // Update OLED
                    // -------------------------------------------------------------------------

                    display.clear(BinaryColor::Off).unwrap();

                    let mut line: String<32> = String::new();

                    // Header
                    Text::with_baseline(
                        "LORAM RECEIVER",
                        Point::new(0, 0),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();

                    // Sequence
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

                    // RSSI
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

                    // SNR
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

                    // Packet stats
                    write!(&mut line, "RX {} MISS {}", received_packets, missed_packets).unwrap();

                    Text::with_baseline(
                        line.as_str(),
                        Point::new(0, 48),
                        text_style,
                        Baseline::Top,
                    )
                    .draw(&mut display)
                    .unwrap();

                    display.flush().unwrap();

                    info!(
                        "RX seq={} RSSI={} dBm SNR={} dB | received={} missed={}",
                        sequence,
                        packet_status.rssi,
                        packet_status.snr,
                        received_packets,
                        missed_packets
                    );
                } else {
                    error!("Invalid or unknown packet");
                }
            }

            Err(err) => {
                error!("RX error: {}", err);
            }
        }
    }
}
