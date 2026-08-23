use defmt::info;

use embassy_rp::{
    gpio::{Input, Pull},
    peripherals::{PIN_4, PIN_5, PIN_6, PIN_7},
    Peri,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
};
use embassy_time::Timer;

// -----------------------------------------------------------------------------
// GOLFER BUTTON SUBSYSTEM
//
// Hardware allocation:
//
//   GP4 -> Button 1
//   GP5 -> Button 2
//   GP6 -> Button 3
//   GP7 -> Button 4
//
// All four buttons are active-low and use the RP2350's internal pull-ups.
// buttons.rs owns only input sampling/debouncing and publishes clean state
// changes. It intentionally knows nothing about audio, UI policy, or radio
// behavior.
// -----------------------------------------------------------------------------

const POLL_INTERVAL_MS: u64 = 5;
const DEBOUNCE_SAMPLES: u8 = 4;
const EVENT_CHANNEL_DEPTH: usize = 8;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Button {
    One,
    Two,
    Three,
    Four,
}

impl Button {
    pub const ALL: [Self; 4] = [
        Self::One,
        Self::Two,
        Self::Three,
        Self::Four,
    ];

    pub const fn index(self) -> usize {
        match self {
            Self::One => 0,
            Self::Two => 1,
            Self::Three => 2,
            Self::Four => 3,
        }
    }

    pub const fn number(self) -> u8 {
        self.index() as u8 + 1
    }
}

#[derive(Clone, Copy)]
pub struct ButtonEvent {
    pub button: Button,
    pub pressed: bool,
}

/// Debounced button-state changes for the application layer.
pub static EVENTS: Channel<
    CriticalSectionRawMutex,
    ButtonEvent,
    EVENT_CHANNEL_DEPTH,
> = Channel::new();

/// Own GP4-GP7 and publish debounced press/release events.
///
/// A short polling loop is intentional here. Four human-operated switches do
/// not justify four separate interrupt tasks, and 5 ms polling with four stable
/// samples gives roughly 20 ms of software debounce while remaining effectively
/// instantaneous to the user.
#[embassy_executor::task]
pub async fn task(
    button_1: Peri<'static, PIN_4>,
    button_2: Peri<'static, PIN_5>,
    button_3: Peri<'static, PIN_6>,
    button_4: Peri<'static, PIN_7>,
) {
    let inputs = [
        Input::new(button_1, Pull::Up),
        Input::new(button_2, Pull::Up),
        Input::new(button_3, Pull::Up),
        Input::new(button_4, Pull::Up),
    ];

    // Start from the normal released state. If GOLFER starts while a button is
    // already held, the normal debounce path will publish that press shortly
    // after this task begins.
    let mut stable_pressed = [false; 4];
    let mut differing_samples = [0u8; 4];

    info!("Button subsystem online: GP4-GP7 active-low");

    loop {
        for (index, input) in inputs.iter().enumerate() {
            let sampled_pressed = input.is_low();

            if sampled_pressed == stable_pressed[index] {
                differing_samples[index] = 0;
                continue;
            }

            differing_samples[index] = differing_samples[index].saturating_add(1);

            if differing_samples[index] < DEBOUNCE_SAMPLES {
                continue;
            }

            stable_pressed[index] = sampled_pressed;
            differing_samples[index] = 0;

            let button = Button::ALL[index];

            if sampled_pressed {
                info!("Button {} pressed", button.number());
            } else {
                info!("Button {} released", button.number());
            }

            EVENTS
                .send(ButtonEvent {
                    button,
                    pressed: sampled_pressed,
                })
                .await;
        }

        Timer::after_millis(POLL_INTERVAL_MS).await;
    }
}
