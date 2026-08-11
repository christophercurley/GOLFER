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

    loop {
        rx_buffer.fill(0);

        match lora.rx(&rx_packet_params, &mut rx_buffer).await {
            Ok((received_len, packet_status)) => {
                let len = received_len as usize;

                info!("RX packet!");
                info!("Length: {}", received_len);
                info!("RSSI: {} dBm", packet_status.rssi);
                info!("SNR: {} dB", packet_status.snr);

                info!("Payload: {=[u8]:x}", &rx_buffer[..len]);

                // Visible RX indication.
                led.set_high();
                Timer::after_millis(100).await;
                led.set_low();
            }

            Err(err) => {
                error!("RX error: {}", err);
            }
        }
    }
}
