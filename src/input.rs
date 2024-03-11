use defmt::{debug, info};
use embassy_rp::{gpio::Input, peripherals::PIN_15, Peripherals};

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

#[embassy_executor::task]
pub async fn input_loop(btn_z: Input<'static, PIN_15>) -> ! {
    loop {
        unsafe {
            GCC_STATE.buttons_2.button_z = btn_z.is_low();
        }
    }
}
