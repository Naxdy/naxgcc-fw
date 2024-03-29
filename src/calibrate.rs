use defmt::Format;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};
use embassy_time::Timer;

use crate::{gcc_hid::GcReport, input::CHANNEL_GCC_STATE};

const CALIBRATION_ENTRY_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Start,
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Y,
];

/// This doesn't need to be super fast, since it's only used
/// in calibration mode.
const BUTTON_POLL_INTERVAL_MILLIS: u64 = 20;

#[derive(Clone, Copy, Debug, Format)]
enum AwaitableButtons {
    A,
    B,
    X,
    Y,
    Up,
    Down,
    Left,
    Right,
    Start,
    L,
    R,
}

trait WaitForButtonPress {
    /// Wait for a single button press.
    async fn wait_for_button_press(&mut self, button_to_wait_for: &AwaitableButtons);

    /// Wait for multiple buttons to be pressed simultaneously.
    async fn wait_for_simultaneous_button_presses<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    );

    /// Wait for a single button press of specified buttons, and return the button that was pressed.
    async fn wait_and_filter_button_press<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) -> AwaitableButtons;

    /// Wait for multiple possible button combinations to be pressed simultaneously, and return the index of the combination that was pressed.
    async fn wait_and_filter_simultaneous_button_presses<const N: usize, const M: usize>(
        &mut self,
        buttons_to_wait_for: &[[AwaitableButtons; N]; M],
    ) -> usize;
}

impl<'a, const I: usize, const J: usize, const K: usize> WaitForButtonPress
    for Subscriber<'a, CriticalSectionRawMutex, GcReport, I, J, K>
{
    async fn wait_for_button_press(&mut self, button_to_wait_for: &AwaitableButtons) {
        loop {
            if match self.next_message_pure().await {
                report => is_awaitable_button_pressed(&report, button_to_wait_for),
            } {
                break;
            }
            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_for_simultaneous_button_presses<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) {
        loop {
            if match self.next_message_pure().await {
                report => buttons_to_wait_for
                    .iter()
                    .all(|button| is_awaitable_button_pressed(&report, button)),
            } {
                break;
            }
            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_and_filter_button_press<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) -> AwaitableButtons {
        loop {
            match self.next_message_pure().await {
                report => {
                    for button in buttons_to_wait_for {
                        if is_awaitable_button_pressed(&report, button) {
                            return *button;
                        }
                    }
                }
            }
            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_and_filter_simultaneous_button_presses<const N: usize, const M: usize>(
        &mut self,
        buttons_to_wait_for: &[[AwaitableButtons; N]; M],
    ) -> usize {
        loop {
            match self.next_message_pure().await {
                report => {
                    for (i, buttons) in buttons_to_wait_for.iter().enumerate() {
                        if buttons
                            .iter()
                            .all(|button| is_awaitable_button_pressed(&report, button))
                        {
                            return i;
                        }
                    }
                }
            }
            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }
}

fn is_awaitable_button_pressed(report: &GcReport, button_to_wait_for: &AwaitableButtons) -> bool {
    match button_to_wait_for {
        AwaitableButtons::A => report.buttons_1.button_a,
        AwaitableButtons::B => report.buttons_1.button_b,
        AwaitableButtons::X => report.buttons_1.button_x,
        AwaitableButtons::Y => report.buttons_1.button_y,
        AwaitableButtons::Up => report.buttons_1.dpad_up,
        AwaitableButtons::Down => report.buttons_1.dpad_down,
        AwaitableButtons::Left => report.buttons_1.dpad_left,
        AwaitableButtons::Right => report.buttons_1.dpad_right,
        AwaitableButtons::Start => report.buttons_2.button_start,
        AwaitableButtons::L => report.buttons_2.button_l,
        AwaitableButtons::R => report.buttons_2.button_r,
    }
}

pub async fn calibration_loop() {
    let mut gcc_subscriber = CHANNEL_GCC_STATE.subscriber().unwrap();

    loop {
        gcc_subscriber
            .wait_for_simultaneous_button_presses(&CALIBRATION_ENTRY_COMBO)
            .await;
    }
}
