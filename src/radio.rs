use defmt::{error, info};

use embassy_rp::{
    bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull},
    peripherals::{
        DMA_CH0, DMA_CH1, PIN_2, PIN_3, PIN_10, PIN_11, PIN_12, PIN_15, PIN_20, SPI1,
    },
    spi::{Async, Config as SpiConfig, Spi},
    Peri,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Delay;

use embedded_hal_bus::spi::ExclusiveDevice;

use lora_phy::{
    iv::GenericSx126xInterfaceVariant,
    mod_params::{
        Bandwidth, CodingRate, ModulationParams, PacketParams, PacketStatus, RadioError,
        SpreadingFactor,
    },
    sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage},
    LoRa, RxMode,
};

pub const RX_BUFFER_SIZE: usize = 255;

const LORA_FREQUENCY_HZ: u32 = 915_000_000;

const RX_CHANNEL_DEPTH: usize = 4;

/// One received LoRa frame handed from the dedicated radio task to the
/// application layer.
///
/// Keeping the radio in its own task is intentional: lora-phy warns that
/// cancelling SX126x IRQ processing mid-flow can lock the radio. The task
/// therefore owns `Radio::receive().await` and never wraps it in a timeout.
pub struct RxPacket {
    pub len: u8,
    pub data: [u8; RX_BUFFER_SIZE],
    pub status: PacketStatus,
}

/// Application-facing RX queue.
///
/// The application is free to timeout/cancel `receive()` on this channel.
/// Doing so does not cancel the SX1262 driver's receive future.
pub static RX_CHANNEL: Channel<CriticalSectionRawMutex, RxPacket, RX_CHANNEL_DEPTH> =
    Channel::new();

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>;
});

type RadioSpi = ExclusiveDevice<Spi<'static, SPI1, Async>, Output<'static>, Delay>;
type RadioInterface = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioKind = Sx126x<RadioSpi, RadioInterface, Sx1262>;
type LoraPhy = LoRa<RadioKind, Delay>;

/// Owns the SX1262, its LoRa configuration, and the RX packet parameters.
///
/// For this first refactor the application still owns packet parsing, sequence
/// tracking, link-loss timing, display updates, etc. This module only owns the
/// actual radio hardware/configuration boundary.
pub struct Radio {
    lora: LoraPhy,
    _modulation_params: ModulationParams,
    rx_packet_params: PacketParams,
}

impl Radio {
    /// Initialize the Waveshare Pico-LoRa-SX1262 using the currently proven
    /// LORAM v1 SF7 baseline and enter continuous RX mode.
    pub async fn new(
        spi1: Peri<'static, SPI1>,
        sck: Peri<'static, PIN_10>,
        mosi: Peri<'static, PIN_11>,
        miso: Peri<'static, PIN_12>,
        dma_tx: Peri<'static, DMA_CH0>,
        dma_rx: Peri<'static, DMA_CH1>,
        nss_pin: Peri<'static, PIN_3>,
        reset_pin: Peri<'static, PIN_15>,
        dio1_pin: Peri<'static, PIN_20>,
        busy_pin: Peri<'static, PIN_2>,
    ) -> Result<Self, RadioError> {
        // ---------------------------------------------------------------------
        // Waveshare Pico-LoRa-SX1262
        //
        // GP2  = BUSY
        // GP3  = NSS / CS
        // GP10 = SPI1 SCK
        // GP11 = SPI1 MOSI
        // GP12 = SPI1 MISO
        // GP15 = RESET
        // GP20 = DIO1
        // ---------------------------------------------------------------------

        let nss = Output::new(nss_pin, Level::High);
        let reset = Output::new(reset_pin, Level::High);

        let dio1 = Input::new(dio1_pin, Pull::None);
        let busy = Input::new(busy_pin, Pull::None);

        // ---------------------------------------------------------------------
        // SPI1
        // ---------------------------------------------------------------------

        let spi = Spi::new(
            spi1,
            sck,
            mosi,
            miso,
            dma_tx,
            dma_rx,
            Irqs,
            SpiConfig::default(),
        );

        let spi = ExclusiveDevice::new(spi, nss, Delay).unwrap();

        // ---------------------------------------------------------------------
        // SX1262 interface
        // ---------------------------------------------------------------------

        let interface =
            GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

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
                return Err(err);
            }
        };

        // ---------------------------------------------------------------------
        // Proven LORAM v1 baseline:
        //
        // 915 MHz
        // SF7
        // BW 125 kHz
        // Coding rate 4/5
        // ---------------------------------------------------------------------

        let modulation_params = match lora.create_modulation_params(
            SpreadingFactor::_7,
            Bandwidth::_125KHz,
            CodingRate::_4_5,
            LORA_FREQUENCY_HZ,
        ) {
            Ok(params) => params,
            Err(err) => {
                error!("Failed to create modulation params: {}", err);
                return Err(err);
            }
        };

        // ---------------------------------------------------------------------
        // RX packet parameters:
        //
        // 8-symbol preamble
        // explicit header
        // CRC enabled
        // normal IQ
        // ---------------------------------------------------------------------

        let rx_packet_params = match lora.create_rx_packet_params(
            8,
            false,
            RX_BUFFER_SIZE as u8,
            true,
            false,
            &modulation_params,
        ) {
            Ok(params) => params,
            Err(err) => {
                error!("Failed to create RX packet params: {}", err);
                return Err(err);
            }
        };

        // ---------------------------------------------------------------------
        // Enter continuous RX mode
        // ---------------------------------------------------------------------

        info!("Preparing continuous RX...");

        if let Err(err) = lora
            .prepare_for_rx(
                RxMode::Continuous,
                &modulation_params,
                &rx_packet_params,
            )
            .await
        {
            error!("Failed to prepare RX: {}", err);
            return Err(err);
        }

        info!("LORAMv1 RX READY");

        Ok(Self {
            lora,
            _modulation_params: modulation_params,
            rx_packet_params,
        })
    }

    /// Wait for and receive the next LoRa packet.
    pub async fn receive(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<(u8, PacketStatus), RadioError> {
        self.lora.rx(&self.rx_packet_params, buffer).await
    }
}

/// Dedicated SX1262 receive task.
///
/// IMPORTANT: do not wrap `radio.receive()` in `with_timeout`, `select`, or any
/// other cancellation mechanism. lora-phy's SX126x IRQ processing must be
/// allowed to finish once entered.
#[embassy_executor::task]
pub async fn receive_task(mut radio: Radio) {
    loop {
        let mut buffer = [0u8; RX_BUFFER_SIZE];

        match radio.receive(&mut buffer).await {
            Ok((len, status)) => {
                RX_CHANNEL
                    .send(RxPacket {
                        len,
                        data: buffer,
                        status,
                    })
                    .await;
            }

            Err(err) => {
                error!("RX error: {}", err);
            }
        }
    }
}

