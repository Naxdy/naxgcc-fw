use defmt::warn;

use crate::{
    config::{is_awaitable_button_pressed, AwaitableButtons},
    input::ControllerState,
};

/**
 * Houses functionality for modifying GCC state before it is sent to the console.
 *
 * General info for implementing filters on the sticks:
 * X and Y values of a stick go each from 0 to 255.
 * 127.5 is the middle value and when both X and Y are 127.5 the stick is in neutral position.
 */

pub trait InputFilter: Sized {
    fn apply_filter(&mut self, controller_state: &mut ControllerState);
}

/// Presses a single button if another button is pressed.
pub struct SingleButtonMacroFilter {
    /// The button that will trigger the macro.
    pub btn_instigator: AwaitableButtons,
    /// The button that will be pressed alongside the instigator.
    pub btn_to_press: AwaitableButtons,
}

impl InputFilter for SingleButtonMacroFilter {
    fn apply_filter(&mut self, controller_state: &mut ControllerState) {
        if is_awaitable_button_pressed(controller_state, &self.btn_instigator) {
            match self.btn_to_press {
                AwaitableButtons::A => {
                    controller_state.button_a = true;
                }
                AwaitableButtons::B => {
                    controller_state.button_b = true;
                }
                AwaitableButtons::X => {
                    controller_state.button_x = true;
                }
                AwaitableButtons::Y => {
                    controller_state.button_y = true;
                }
                AwaitableButtons::L => {
                    controller_state.trigger_l = true;
                }
                AwaitableButtons::R => {
                    controller_state.trigger_r = true;
                }
                AwaitableButtons::Z => {
                    controller_state.trigger_zr = true;
                }
                AwaitableButtons::Start => {
                    controller_state.button_start = true;
                }
                AwaitableButtons::Up => {
                    controller_state.dpad_up = true;
                }
                AwaitableButtons::Down => {
                    controller_state.dpad_down = true;
                }
                AwaitableButtons::Left => {
                    controller_state.dpad_left = true;
                }
                AwaitableButtons::Right => {
                    controller_state.dpad_right = true;
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
    fn apply_filter(&mut self, controller_state: &mut ControllerState) {
        if controller_state.stick_state.cy > 157 {
            if (137..=201).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 201;
                controller_state.stick_state.cy = 255;
            } else if (53..=117).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 53;
                controller_state.stick_state.cy = 255;
            }
        } else if controller_state.stick_state.cy < 97 {
            if (137..=221).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 221;
                controller_state.stick_state.cy = 0;
            } else if (53..=117).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 33;
                controller_state.stick_state.cy = 0;
            }
        }
    }
}

/// Improves hitting up/down angled forward tilt at the cost
/// of making it impossible to hit turnaround up & down tilt
/// and making it slightly harder to hit regular forward tilt.
pub struct CStickAngledFTiltFilter;

impl InputFilter for CStickAngledFTiltFilter {
    fn apply_filter(&mut self, controller_state: &mut ControllerState) {
        if controller_state.stick_state.cy > 147 {
            if (147..=225).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 205;
                controller_state.stick_state.cy = 205;
            } else if (30..=107).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 50;
                controller_state.stick_state.cy = 205;
            }
        } else if controller_state.stick_state.cy < 107 {
            if (147..=225).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 205;
                controller_state.stick_state.cy = 50;
            } else if (30..=107).contains(&controller_state.stick_state.cx) {
                controller_state.stick_state.cx = 50;
                controller_state.stick_state.cy = 50;
            }
        }
    }
}

/// Does nothing.
#[derive(Default)]
pub struct DummyFilter;

impl InputFilter for DummyFilter {
    fn apply_filter(&mut self, _gcc_state: &mut ControllerState) {}
}
