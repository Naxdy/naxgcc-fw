//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]
mod gcc_hid;
mod input;

use defmt::info;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    bind_interrupts,
    gpio::{self, Input},
    multicore::{spawn_core1, Stack},
    peripherals::USB,
    usb::{Driver, InterruptHandler},
};
use embassy_time::Timer;
use gcc_hid::usb_transfer_loop;
use gpio::{Level, Output};
use input::input_loop;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);

    info!("Initializing");

    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| spawner.spawn(usb_transfer_loop(driver)).unwrap());
    });

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner
            .spawn(input_loop(Input::new(p.PIN_15, gpio::Pull::Up)))
            .unwrap()
    });
}
