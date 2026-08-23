use defmt::{debug, info};

use embassy_rp::{
    clocks::clk_sys_freq,
    peripherals::{PIN_26, PWM_SLICE5},
    pwm::{Config, Pwm},
    Peri,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    signal::Signal,
};
use embassy_time::Timer;

// -----------------------------------------------------------------------------
// GOLFER AUDIO SUBSYSTEM
//
// Owns:
//
//   * PWM slice 5
//   * GP26 audio output
//   * PAM8302A input drive
//   * synthesized UI sound sequencing
//   * routine-vs-important audio priority
//
// Routine audio (currently held-button tones, later receiver sonification) is
// stored as latest state. Important UI sounds temporarily preempt that state,
// then the newest routine tone automatically resumes when the UI sound ends.
// -----------------------------------------------------------------------------

const UI_SOUND_QUEUE_DEPTH: usize = 4;
const AUDIO_SERVICE_INTERVAL_MS: u64 = 5;
const UI_DRIVE_PERCENT: u8 = 10;
const MAX_DRIVE_PERCENT: u8 = 50;

#[derive(Clone, Copy)]
struct Tone {
    frequency_hz: u32,
    drive_percent: u8,
}

#[derive(Clone, Copy)]
pub enum UiSound {
    Startup,
    LinkLost,
    LinkReacquired,
}

/// Latest routine tone request.
///
/// Signal is intentional: routine audio is state, not an event stream. If a
/// button changes while a UI sound is playing, only the newest requested tone
/// matters when that sound finishes.
static ROUTINE_TONE: Signal<
    CriticalSectionRawMutex,
    Option<Tone>,
> = Signal::new();

/// Important UI sounds are events and therefore use a queue.
static UI_SOUNDS: Channel<
    CriticalSectionRawMutex,
    UiSound,
    UI_SOUND_QUEUE_DEPTH,
> = Channel::new();

/// Request a continuous routine tone.
///
/// This layer is intentionally lower priority than synthesized UI sounds.
pub async fn play_tone(
    frequency_hz: u32,
    drive_percent: u8,
) {
    if frequency_hz == 0 || drive_percent == 0 {
        stop().await;
        return;
    }

    ROUTINE_TONE.signal(Some(Tone {
        frequency_hz,
        drive_percent: drive_percent.min(MAX_DRIVE_PERCENT),
    }));
}

/// Stop routine audio.
pub async fn stop() {
    ROUTINE_TONE.signal(None);
}

/// Queue an important synthesized UI sound.
///
/// UI sounds preempt routine audio. The latest routine tone automatically
/// resumes after the UI sound completes.
pub async fn play_ui_sound(sound: UiSound) {
    UI_SOUNDS.send(sound).await;
}

/// Dedicated audio hardware / synthesis task.
///
/// This task is the sole owner of PWM5 / GP26.
#[embassy_executor::task]
pub async fn task(
    pwm_slice: Peri<'static, PWM_SLICE5>,
    audio_pin: Peri<'static, PIN_26>,
) {
    let mut pwm = Pwm::new_output_a(
        pwm_slice,
        audio_pin,
        silent_config(),
    );

    let mut routine_tone: Option<Tone> = None;

    info!("Audio subsystem online: PWM5A on GP26");

    loop {
        // Important UI sounds always win over routine audio.
        if let Ok(sound) = UI_SOUNDS.try_receive() {
            play_ui_sequence(&mut pwm, sound).await;

            // A button may have changed while the UI sound was playing. Signal
            // stores only the newest routine request, which is exactly what we
            // want to resume now.
            if let Some(latest) = ROUTINE_TONE.try_take() {
                routine_tone = latest;
            }

            apply_optional_tone(&mut pwm, routine_tone);
            continue;
        }

        if let Some(latest) = ROUTINE_TONE.try_take() {
            routine_tone = latest;
            apply_optional_tone(&mut pwm, routine_tone);
        }

        Timer::after_millis(AUDIO_SERVICE_INTERVAL_MS).await;
    }
}

async fn play_ui_sequence(
    pwm: &mut Pwm<'static>,
    sound: UiSound,
) {
    match sound {
        UiSound::Startup => {
            info!("Audio UI sound: STARTUP");

            // Bright ascending four-note boot signature.
            note(pwm, 523, 90).await;
            gap(pwm, 25).await;
            note(pwm, 659, 90).await;
            gap(pwm, 25).await;
            note(pwm, 784, 100).await;
            gap(pwm, 30).await;
            note(pwm, 1047, 170).await;
        }

        UiSound::LinkLost => {
            info!("Audio UI sound: LINK LOST");

            // Deliberately descending / defeated.
            note(pwm, 659, 130).await;
            gap(pwm, 35).await;
            note(pwm, 440, 180).await;
            gap(pwm, 40).await;
            note(pwm, 294, 280).await;
        }

        UiSound::LinkReacquired => {
            info!("Audio UI sound: LINK REACQUIRED");

            // Fast upward fanfare with a held high finish.
            note(pwm, 440, 85).await;
            gap(pwm, 20).await;
            note(pwm, 659, 85).await;
            gap(pwm, 20).await;
            note(pwm, 880, 100).await;
            gap(pwm, 20).await;
            note(pwm, 1175, 220).await;
        }
    }

    apply_optional_tone(pwm, None);
}

async fn note(
    pwm: &mut Pwm<'static>,
    frequency_hz: u32,
    duration_ms: u64,
) {
    apply_optional_tone(
        pwm,
        Some(Tone {
            frequency_hz,
            drive_percent: UI_DRIVE_PERCENT,
        }),
    );

    Timer::after_millis(duration_ms).await;
}

async fn gap(
    pwm: &mut Pwm<'static>,
    duration_ms: u64,
) {
    apply_optional_tone(pwm, None);
    Timer::after_millis(duration_ms).await;
}

fn apply_optional_tone(
    pwm: &mut Pwm<'static>,
    tone: Option<Tone>,
) {
    match tone {
        Some(tone) => {
            let config = tone_config(
                tone.frequency_hz,
                tone.drive_percent,
            );

            pwm.set_config(&config);

            debug!(
                "Audio tone: {} Hz drive={}%",
                tone.frequency_hz,
                tone.drive_percent
            );
        }

        None => {
            let config = silent_config();
            pwm.set_config(&config);
            debug!("Audio stopped");
        }
    }
}

/// Build a PWM configuration for an audible square-wave tone.
fn tone_config(
    frequency_hz: u32,
    drive_percent: u8,
) -> Config {
    let system_clock_hz = clk_sys_freq();
    let frequency_hz = frequency_hz.max(1);

    let mut divider: u32 = 1;
    let mut period_counts =
        system_clock_hz / frequency_hz / divider;

    while period_counts > 65_536 && divider < 255 {
        divider += 1;
        period_counts =
            system_clock_hz / frequency_hz / divider;
    }

    period_counts = period_counts.clamp(2, 65_536);

    let top = (period_counts - 1) as u16;
    let compare = (
        period_counts * u32::from(drive_percent) / 100
    ) as u16;

    let mut config = Config::default();

    config.divider = (divider as u8).into();
    config.top = top;
    config.compare_a = compare;
    config.compare_b = 0;
    config.phase_correct = false;
    config.enable = true;

    config
}

fn silent_config() -> Config {
    let mut config = Config::default();

    config.compare_a = 0;
    config.compare_b = 0;
    config.enable = true;

    config
}
