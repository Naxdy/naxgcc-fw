use defmt::info;
use embedded_hal::digital::v2::InputPin;

use crate::gcc_hid::{Buttons1, Buttons2, GcReport};

pub static mut GCC_STATE: GcReport = GcReport {
    buttons_1: Buttons1 {
        button_a: false,
        button_b: false,
        button_x: false,
        button_y: false,
        dpad_left: false,
        dpad_right: false,
        dpad_down: false,
        dpad_up: false,
    },
    buttons_2: Buttons2 {
        button_start: false,
        button_z: false,
        button_r: false,
        button_l: false,
        blank1: 0,
    },
    stick_x: 0,
    stick_y: 0,
    cstick_x: 0,
    cstick_y: 0,
    trigger_l: 0,
    trigger_r: 0,
};

macro_rules! pin_inputs {
    ($x:tt {$($f:tt: $g:tt),*}) => {
        pub struct $x<$($g,)*>
        where
            $(
                $g: InputPin,
            )*
        {
            $(
                pub $f: $g,
            )*
        }
    };
}

macro_rules! assign_pins {
    ($gcc:expr, $inputs:tt, {$($p:tt.$c:tt),*}) => {
        $(
            $gcc.$p.$c = $inputs.$c.is_low().map_err(|_| "").unwrap();
        )*
    };
}

pin_inputs!(BasicInputs {
    button_a: A,
    button_b: B,
    button_x: X,
    button_y: Y,
    dpad_left: Dl,
    dpad_right: Dr,
    dpad_down: Dd,
    dpad_up: Du,
    button_start: S,
    button_z: Z,
    button_r: R,
    button_l: L
});

pub fn input_loop<
    A: InputPin,
    B: InputPin,
    X: InputPin,
    Y: InputPin,
    Dl: InputPin,
    Dr: InputPin,
    Dd: InputPin,
    Du: InputPin,
    S: InputPin,
    Z: InputPin,
    R: InputPin,
    L: InputPin,
>(
    basic_inputs: BasicInputs<A, B, X, Y, Dl, Dr, Dd, Du, S, Z, R, L>,
) -> ! {
    info!("Input loop started.");

    let update_gcc_state = || unsafe {
        // simple booleans
        assign_pins!(GCC_STATE, basic_inputs, {
            buttons_1.button_a,
            buttons_1.button_b,
            buttons_1.button_x,
            buttons_1.button_y,
            buttons_1.dpad_left,
            buttons_1.dpad_right,
            buttons_1.dpad_down,
            buttons_1.dpad_up,
            buttons_2.button_start,
            buttons_2.button_z,
            buttons_2.button_r,
            buttons_2.button_l
        });

        // TODO: sticks
        GCC_STATE.cstick_x = 127;
        GCC_STATE.cstick_y = 127;
        GCC_STATE.stick_x = 127;
        GCC_STATE.stick_y = 127;
    };

    loop {
        update_gcc_state();
    }
}
