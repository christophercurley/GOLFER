#![no_std]
#![no_main]

use defmt::{error, info};
use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    spi::{Config as SpiConfig, Spi},
};
use embassy_time::{Delay, Timer};

use embassy_rp::peripherals::{DMA_CH0, DMA_CH1};
use embassy_rp::{bind_interrupts, dma};

use embedded_hal_bus::spi::ExclusiveDevice;

use lora_phy::{
    LoRa,
    iv::GenericSx126xInterfaceVariant,
    sx126x::{self, Sx126x, Sx1262, TcxoCtrlVoltage},
};

use panic_probe as _;

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>, dma::InterruptHandler<DMA_CH1>;
});

#[embassy_executor::main(
    executor = "embassy_rp::executor::Executor",
    entry = "cortex_m_rt::entry"
)]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Pico 2 onboard LED
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("LORAMv1 online");

    // ---------------------------------------------------------
    // Waveshare Pico-LoRa-SX1262 pin mapping
    // ---------------------------------------------------------

    // SX1262 control pins
    let nss = Output::new(p.PIN_3, Level::High);
    let reset = Output::new(p.PIN_15, Level::High);

    let dio1 = Input::new(p.PIN_20, Pull::None);
    let busy = Input::new(p.PIN_2, Pull::None);

    // ---------------------------------------------------------
    // SPI1
    //
    // GP10 = SCK
    // GP11 = MOSI
    // GP12 = MISO
    // ---------------------------------------------------------

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

    // Waveshare board does not require separate MCU-controlled
    // RXEN/TXEN GPIOs here.
    let interface = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    // ---------------------------------------------------------
    // SX1262 hardware configuration
    // ---------------------------------------------------------

    let radio_config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        rx_boost: false,
    };

    info!("Initializing SX1262...");

    let _lora = match LoRa::new(Sx126x::new(spi, interface, radio_config), false, Delay).await {
        Ok(lora) => {
            info!("SX1262 initialization successful");
            lora
        }

        Err(_err) => {
            error!("SX1262 initialization FAILED");

            // Make failure visually obvious:
            // rapid LED blink instead of normal 2-second cadence.
            loop {
                led.set_high();
                Timer::after_millis(100).await;

                led.set_low();
                Timer::after_millis(100).await;
            }
        }
    };

    info!("LORAMv1 radio online");

    // ---------------------------------------------------------
    // Normal heartbeat
    // ---------------------------------------------------------

    loop {
        led.set_high();
        Timer::after_millis(2000).await;

        led.set_low();
        Timer::after_millis(2000).await;
    }
}
