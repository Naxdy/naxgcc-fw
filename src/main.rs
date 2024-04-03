//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]
mod config;
mod filter;
mod gcc_hid;
mod helpers;
mod input;
mod stick;

use config::config_task;
use defmt::{debug, info};
use embassy_executor::Executor;
use embassy_rp::{
    bind_interrupts,
    flash::{Async, Flash},
    gpio::{self, AnyPin, Input},
    multicore::{spawn_core1, Stack},
    peripherals::USB,
    spi::{self, Spi},
    usb::{Driver, InterruptHandler},
};
use gcc_hid::usb_transfer_task;
use gpio::{Level, Output};

use input::{update_button_state_task, update_stick_states_task};
use static_cell::StaticCell;

use crate::config::enter_config_mode_task;
use crate::gcc_hid::rumble_task;

use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

const FLASH_SIZE: usize = 2 * 1024 * 1024;
const ADDR_OFFSET: u32 = 0x100000;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Initializing");

    let p = embassy_rp::init(Default::default());

    let driver = Driver::new(p.USB, Irqs);

    // reading and writing from flash has to be done on the main thread, else funny things happen.

    let mut flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH0);
    let mut uid = [0u8; 8];
    flash.blocking_unique_id(&mut uid).unwrap();

    let mosi = p.PIN_7;
    let miso = p.PIN_4;
    let spi_clk = p.PIN_6;

    let p_acs = p.PIN_24;
    let p_ccs = p.PIN_23;

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 3000 * 1000;

    let spi = Spi::new_blocking(p.SPI0, spi_clk, mosi, miso, spi_cfg);

    let spi_acs = Output::new(AnyPin::from(p_acs), Level::High); // active low
    let spi_ccs = Output::new(AnyPin::from(p_ccs), Level::High); // active low

    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        debug!("Mana");
        executor1.run(|spawner| {
            spawner.spawn(usb_transfer_task(uid, driver)).unwrap();
            spawner.spawn(enter_config_mode_task()).unwrap();
            spawner
                .spawn(rumble_task(p.PIN_25, p.PIN_29, p.PWM_CH4, p.PWM_CH6))
                .unwrap();
            // spawner.spawn(input_integrity_benchmark()).unwrap();
            spawner
                .spawn(update_button_state_task(
                    Input::new(AnyPin::from(p.PIN_20), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_17), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_16), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_11), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_9), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_10), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_8), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_22), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_21), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_18), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_19), gpio::Pull::Up),
                    Input::new(AnyPin::from(p.PIN_5), gpio::Pull::Up),
                ))
                .unwrap()
        });
    });

    // Stick loop has to run on core0 because it makes use of SPI0.
    // Perhaps in the future we can rewire the board to have it make use of SPI1 instead.
    // This way it could be the sole task running on core1, and everything else could happen on core0.
    // Also, it needs to run on a higher prio executor to ensure consistent polling.
    // interrupt::SWI_IRQ_1.set_priority(interrupt::Priority::P0);
    // let spawner_high = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    // spawner_high
    //     .spawn(update_stick_states_task(
    //         spi,
    //         spi_acs,
    //         spi_ccs,
    //         controller_config.clone(),
    //     ))
    //     .unwrap();

    let executor0 = EXECUTOR0.init(Executor::new());
    info!("Initialized.");

    executor0.run(|spawner| {
        // Config task has to run on core0 because it reads and writes to flash.
        spawner.spawn(config_task(flash)).unwrap();

        spawner
            .spawn(update_stick_states_task(spi, spi_acs, spi_ccs))
            .unwrap();
    });
}
