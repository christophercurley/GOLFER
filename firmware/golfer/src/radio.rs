use defmt::{debug, error, info, warn};

use embassy_rp::{
    Peri, bind_interrupts, dma,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, DMA_CH1, PIN_2, PIN_3, PIN_10, PIN_11, PIN_12, PIN_15, PIN_20, SPI1},
    spi::{Async, Config as SpiConfig, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Delay, Instant, Timer};

use embedded_hal_bus::spi::ExclusiveDevice;

use lora_phy::{
    LoRa, RxMode,
    iv::GenericSx126xInterfaceVariant,
    mod_params::{
        Bandwidth, CodingRate, ModulationParams, PacketParams, PacketStatus, RadioError,
        SpreadingFactor,
    },
    sx126x::{self, Sx126x, Sx1262, TcxoCtrlVoltage},
};

pub const RX_BUFFER_SIZE: usize = 255;
pub const TX_BUFFER_SIZE: usize = 255;

pub const LORA_FREQUENCY_HZ: u32 = 915_000_000;
pub const DEFAULT_TX_POWER_DBM: i32 = 14;

// lora-phy 3.0.1 explicitly warns against cancelling SX126x IRQ processing.
// That means a radio-owner task cannot safely select! between an indefinitely
// pending continuous-RX future and an application TX/mode command.
//
// Instead, the manager receives in bounded SX1262 Single windows. The SX1262
// symbol timeout ends an idle window naturally; if a preamble is detected the
// timeout stops and the in-flight packet is allowed to finish. Commands are
// serviced only at those safe boundaries, so the lora-phy RX future is never
// cancelled mid-flow.
//
// 248 is the SX126x driver's maximum LoRa symbol timeout. At the current
// SF7/BW125 baseline one symbol is ~1.024 ms, so an idle command waits at most
// roughly 254 ms before the radio owner can service it. This deliberately
// minimizes RX re-arm gaps for the first regression pass. We can tune this
// later if interactive modes need lower command latency.
pub const RX_SLICE_SYMBOLS: u16 = 248;

const RX_CHANNEL_DEPTH: usize = 4;
const COMMAND_CHANNEL_DEPTH: usize = 4;
const TX_EVENT_CHANNEL_DEPTH: usize = 4;

/// One received LoRa frame handed from the authoritative radio-owner task to
/// the application layer.
pub struct RxPacket {
    pub len: u8,
    pub data: [u8; RX_BUFFER_SIZE],
    pub status: PacketStatus,
}

/// Application-facing RX queue.
///
/// The application is free to timeout/cancel `receive()` on this channel. That
/// never cancels the SX1262 driver's receive future; only radio.rs owns that.
pub static RX_CHANNEL: Channel<CriticalSectionRawMutex, RxPacket, RX_CHANNEL_DEPTH> =
    Channel::new();

/// A fully-owned TX request. Packet parsing/semantics remain above radio.rs;
/// this layer only serializes safe physical-radio operations.
#[derive(Clone, Copy)]
pub struct TxRequest {
    pub token: u32,
    pub len: u8,
    pub data: [u8; TX_BUFFER_SIZE],
    pub power_dbm: i32,
    pub resume_receive: bool,
}

impl TxRequest {
    /// Build a TX request using GOLFER's current default output power.
    pub fn new_default(token: u32, payload: &[u8], resume_receive: bool) -> Option<Self> {
        Self::new(token, payload, DEFAULT_TX_POWER_DBM, resume_receive)
    }

    /// Build a TX request without allocation. Returns None for payloads larger
    /// than the SX1262/LoRa packet buffer.
    pub fn new(
        token: u32,
        payload: &[u8],
        power_dbm: i32,
        resume_receive: bool,
    ) -> Option<Self> {
        if payload.len() > TX_BUFFER_SIZE {
            return None;
        }

        let mut data = [0u8; TX_BUFFER_SIZE];
        data[..payload.len()].copy_from_slice(payload);

        Some(Self {
            token,
            len: payload.len() as u8,
            data,
            power_dbm,
            resume_receive,
        })
    }
}

/// Commands accepted by the radio owner.
///
/// Commands are intentionally transport-oriented rather than survey-oriented.
/// Discovery, joining, survey membership, response policy, beacon schedules,
/// etc. belong to higher application layers.
pub enum RadioCommand {
    SetReceiveEnabled(bool),
    Transmit(TxRequest),
}

/// Completion notification for a queued TX operation.
#[derive(Clone, Copy, defmt::Format)]
pub struct TxEvent {
    pub token: u32,
    pub len: u8,
    pub success: bool,
    /// Duration of `lora.tx().await` only. This is firmware/radio operation
    /// duration, not the theoretical RF airtime calculation.
    pub tx_duration_us: u64,
}

pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, RadioCommand, COMMAND_CHANNEL_DEPTH> =
    Channel::new();

pub static TX_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, TxEvent, TX_EVENT_CHANNEL_DEPTH> =
    Channel::new();

/// Queue a transmit operation for the authoritative radio owner.
pub async fn transmit(request: TxRequest) {
    COMMAND_CHANNEL.send(RadioCommand::Transmit(request)).await;
}

/// Enable or disable receive operation. A disable request is honored at the
/// next cancellation-safe RX boundary; it never aborts an in-flight RX future.
pub async fn set_receive_enabled(enabled: bool) {
    COMMAND_CHANNEL
        .send(RadioCommand::SetReceiveEnabled(enabled))
        .await;
}

bind_interrupts!(struct Irqs {
    DMA_IRQ_0 =>
        dma::InterruptHandler<DMA_CH0>,
        dma::InterruptHandler<DMA_CH1>;
});

type RadioSpi = ExclusiveDevice<Spi<'static, SPI1, Async>, Output<'static>, Delay>;
type RadioInterface = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioKind = Sx126x<RadioSpi, RadioInterface, Sx1262>;
type LoraPhy = LoRa<RadioKind, Delay>;

/// Sole owner of the SX1262 and all physical-radio configuration.
///
/// Application code never touches lora-phy directly. The owner task below
/// serializes RX/TX/mode transitions and preserves the SX126x cancellation
/// safety rule.
pub struct Radio {
    lora: LoraPhy,
    modulation_params: ModulationParams,
    rx_packet_params: PacketParams,
    tx_packet_params: PacketParams,
}

impl Radio {
    /// Initialize the Waveshare Pico-LoRa-SX1262 using the currently proven
    /// GOLFER baseline. Starting/continuing RX is owned by `task()`.
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
        // Waveshare Pico-LoRa-SX1262:
        // GP2 BUSY, GP3 NSS, GP10/11/12 SPI1, GP15 RESET, GP20 DIO1.
        let nss = Output::new(nss_pin, Level::High);
        let reset = Output::new(reset_pin, Level::High);
        let dio1 = Input::new(dio1_pin, Pull::None);
        let busy = Input::new(busy_pin, Pull::None);

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
                return Err(err);
            }
        };

        // Proven GOLFER baseline: 915 MHz / SF7 / BW125 / CR4/5.
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

        // RX: 8-symbol preamble, explicit header, PHY CRC on, normal IQ.
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

        // TX uses the same packet framing and modulation baseline.
        let tx_packet_params = match lora.create_tx_packet_params(
            8,
            false,
            true,
            false,
            &modulation_params,
        ) {
            Ok(params) => params,
            Err(err) => {
                error!("Failed to create TX packet params: {}", err);
                return Err(err);
            }
        };

        let mut radio = Self {
            lora,
            modulation_params,
            rx_packet_params,
            tx_packet_params,
        };

        // Preserve the old boot-time acceptance boundary: LORA init is not
        // considered complete until the proven RX profile has successfully
        // been programmed into the SX1262 at least once. task() will re-arm
        // the first live window after the boot-status hold.
        info!("Preparing managed RX...");
        if let Err(err) = radio.prepare_receive_window().await {
            error!("Failed to prepare managed RX: {}", err);
            return Err(err);
        }

        info!(
            "GOLFER RADIO READY: managed RX/TX owner frequency={} rx_slice_symbols={}",
            LORA_FREQUENCY_HZ,
            RX_SLICE_SYMBOLS
        );

        Ok(radio)
    }

    async fn prepare_receive_window(&mut self) -> Result<(), RadioError> {
        self.lora
            .prepare_for_rx(
                RxMode::Single(RX_SLICE_SYMBOLS),
                &self.modulation_params,
                &self.rx_packet_params,
            )
            .await
    }

    async fn receive_window(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<(u8, PacketStatus), RadioError> {
        self.lora.rx(&self.rx_packet_params, buffer).await
    }

    /// Prepare and execute one TX operation. Returned time measures only the
    /// `tx().await` phase so it remains directly comparable to the MRU trace.
    async fn transmit_packet(
        &mut self,
        payload: &[u8],
        power_dbm: i32,
    ) -> Result<u64, RadioError> {
        self.lora
            .prepare_for_tx(
                &self.modulation_params,
                &mut self.tx_packet_params,
                power_dbm,
                payload,
            )
            .await?;

        let started = Instant::now();
        self.lora.tx().await?;
        Ok(Instant::now().duration_since(started).as_micros())
    }
}

async fn handle_command(radio: &mut Radio, rx_enabled: &mut bool, command: RadioCommand) {
    match command {
        RadioCommand::SetReceiveEnabled(enabled) => {
            *rx_enabled = enabled;
            info!("Radio receive enabled={}", enabled);
        }

        RadioCommand::Transmit(request) => {
            let len = request.len as usize;
            let payload = &request.data[..len];

            match radio.transmit_packet(payload, request.power_dbm).await {
                Ok(tx_duration_us) => {
                    // This is intentionally DEBUG instrumentation. It will become
                    // runtime-configurable later alongside the broader trace policy.
                    debug!(
                        "RADIO TX_DURATION token={} len={} power_dbm={} us={}",
                        request.token,
                        request.len,
                        request.power_dbm,
                        tx_duration_us
                    );

                    TX_EVENT_CHANNEL
                        .send(TxEvent {
                            token: request.token,
                            len: request.len,
                            success: true,
                            tx_duration_us,
                        })
                        .await;
                }

                Err(err) => {
                    error!(
                        "Radio TX failed: token={} len={} power_dbm={} error={}",
                        request.token,
                        request.len,
                        request.power_dbm,
                        err
                    );

                    TX_EVENT_CHANNEL
                        .send(TxEvent {
                            token: request.token,
                            len: request.len,
                            success: false,
                            tx_duration_us: 0,
                        })
                        .await;
                }
            }

            *rx_enabled = request.resume_receive;
        }
    }
}

/// Authoritative SX1262 owner task.
///
/// The task never cancels an SX1262 receive future. RX runs to either a real
/// packet completion or the radio's own symbol timeout, then queued commands
/// are serviced at that known-safe boundary. This is the core architectural
/// change that allows future TX->RX, RX->reply, beacon, discovery, join, and
/// multi-GOLFER behavior without giving hardware ownership to application code.
#[embassy_executor::task]
pub async fn task(mut radio: Radio) {
    let mut rx_enabled = true;

    info!(
        "Radio manager online: rx_enabled=true rx_slice_symbols={} default_tx_power_dbm={}",
        RX_SLICE_SYMBOLS,
        DEFAULT_TX_POWER_DBM
    );

    loop {
        // Drain commands only at a cancellation-safe radio boundary.
        while let Ok(command) = COMMAND_CHANNEL.try_receive() {
            handle_command(&mut radio, &mut rx_enabled, command).await;
        }

        // In standby/application-controlled mode there is no physical RX future
        // to cancel, so block efficiently until the next command arrives.
        if !rx_enabled {
            let command = COMMAND_CHANNEL.receive().await;
            handle_command(&mut radio, &mut rx_enabled, command).await;
            continue;
        }

        if let Err(err) = radio.prepare_receive_window().await {
            error!("RX prepare error: {}", err);
            Timer::after_millis(10).await;
            continue;
        }

        let mut buffer = [0u8; RX_BUFFER_SIZE];

        match radio.receive_window(&mut buffer).await {
            Ok((len, status)) => {
                RX_CHANNEL
                    .send(RxPacket {
                        len,
                        data: buffer,
                        status,
                    })
                    .await;
            }

            // Expected idle-window completion. Do not log this as a fault: its
            // entire purpose is to create a safe command-processing boundary.
            Err(RadioError::ReceiveTimeout) => {}

            Err(err) => {
                warn!("RX operation error: {}", err);
                Timer::after_millis(5).await;
            }
        }
    }
}
