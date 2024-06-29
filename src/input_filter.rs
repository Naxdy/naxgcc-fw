use defmt::warn;

use crate::{
    config::{is_awaitable_button_pressed, AwaitableButtons},
    gcc_hid::GcReport,
};

/**
 * Houses functionality for modifying GCC state before it is sent to the console.
 *
 * General info for implementing filters on the sticks:
 * X and Y values of a stick go each from 0 to 255.
 * 127.5 is the middle value and when both X and Y are 127.5 the stick is in neutral position.
 */

pub trait InputFilter: Sized {
    fn apply_filter(&mut self, gcc_state: &mut GcReport);
}

/// Presses a single button if another button is pressed.
pub struct SingleButtonMacroFilter {
    /// The button that will trigger the macro.
    pub btn_instigator: AwaitableButtons,
    /// The button that will be pressed alongside the instigator.
    pub btn_to_press: AwaitableButtons,
}

impl InputFilter for SingleButtonMacroFilter {
    fn apply_filter(&mut self, gcc_state: &mut GcReport) {
        if is_awaitable_button_pressed(gcc_state, &self.btn_instigator) {
            match self.btn_to_press {
                AwaitableButtons::A => {
                    gcc_state.buttons_1.button_a = true;
                }
                AwaitableButtons::B => {
                    gcc_state.buttons_1.button_b = true;
                }
                AwaitableButtons::X => {
                    gcc_state.buttons_1.button_x = true;
                }
                AwaitableButtons::Y => {
                    gcc_state.buttons_1.button_y = true;
                }
                AwaitableButtons::L => {
                    gcc_state.trigger_l = 255;
                    gcc_state.buttons_2.button_l = true;
                }
                AwaitableButtons::R => {
                    gcc_state.trigger_r = 255;
                    gcc_state.buttons_2.button_r = true;
                }
                AwaitableButtons::Z => {
                    gcc_state.buttons_2.button_z = true;
                }
                AwaitableButtons::Start => {
                    gcc_state.buttons_2.button_start = true;
                }
                AwaitableButtons::Up => {
                    gcc_state.buttons_1.dpad_up = true;
                }
                AwaitableButtons::Down => {
                    gcc_state.buttons_1.dpad_down = true;
                }
                AwaitableButtons::Left => {
                    gcc_state.buttons_1.dpad_left = true;
                }
                AwaitableButtons::Right => {
                    gcc_state.buttons_1.dpad_right = true;
                }
                b => {
                    warn!(
                        "Awaitable button {} is not supported by SingleButtonMacroFilter",
                        b
                    );
                }
            }
        }
    }
}

/// Improves hitting turnaround up & down tilt at the cost
/// of making it harder to hit up/down angled forward tilt.
pub struct CStickUpTiltFilter;

impl InputFilter for CStickUpTiltFilter {
    fn apply_filter(&mut self, gcc_state: &mut GcReport) {
        if gcc_state.cstick_y > 157 {
            if (137..=201).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 201;
                gcc_state.cstick_y = 255;
            } else if (53..=117).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 53;
                gcc_state.cstick_y = 255;
            }
        } else if gcc_state.cstick_y < 97 {
            if (137..=221).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 221;
                gcc_state.cstick_y = 0;
            } else if (53..=117).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 33;
                gcc_state.cstick_y = 0;
            }
        }
    }
}

/// Improves hitting up/down angled forward tilt at the cost
/// of making it impossible to hit turnaround up & down tilt
/// and making it slightly harder to hit regular forward tilt.
pub struct CStickAngledFTiltFilter;

impl InputFilter for CStickAngledFTiltFilter {
    fn apply_filter(&mut self, gcc_state: &mut GcReport) {
        if gcc_state.cstick_y > 147 {
            if (147..=225).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 205;
                gcc_state.cstick_y = 205;
            } else if (30..=107).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 50;
                gcc_state.cstick_y = 205;
            }
        } else if gcc_state.cstick_y < 107 {
            if (147..=225).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 205;
                gcc_state.cstick_y = 50;
            } else if (30..=107).contains(&gcc_state.cstick_x) {
                gcc_state.cstick_x = 50;
                gcc_state.cstick_y = 50;
            }
        }
    }
}

/// Does nothing.
#[derive(Default)]
pub struct DummyFilter;

impl InputFilter for DummyFilter {
    fn apply_filter(&mut self, _gcc_state: &mut GcReport) {}
}
