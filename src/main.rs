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
use config::ControllerConfig;
use defmt::{debug, info};
use embassy_executor::Executor;
use embassy_futures::join::join;
use embassy_rp::{
    bind_interrupts,
    flash::{Async, Flash},
    gpio::{self, AnyPin, Input},
    multicore::{spawn_core1, Stack},
    peripherals::{SPI0, USB},
    pwm::Pwm,
    spi::{self, Spi},
    usb::{Driver, InterruptHandler},
};
use gcc_hid::usb_transfer_task;
use gpio::{Level, Output};

use input::{update_button_state_task, update_stick_states_task};
use static_cell::StaticCell;

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

    let controller_config = ControllerConfig::from_flash_memory(&mut flash).unwrap();

    debug!("Read unique id: {:02X}", uid);

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
            spawner.spawn(usb_transfer_task(driver, uid)).unwrap();
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

    let executor0 = EXECUTOR0.init(Executor::new());
    info!("Initialized.");

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 255;
    pwm_config.enable = true;
    pwm_config.compare_b = 255;

    // let pwm_rumble = Pwm::new_output_b(p.PWM_CH4, p.PIN_25, pwm_config.clone());
    // let pwm_brake = Pwm::new_output_b(p.PWM_CH6, p.PIN_29, pwm_config.clone());

    // pwm_rumble.set_counter(0);
    // pwm_brake.set_counter(255);

    executor0.run(|spawner| {
        spawner.spawn(config_task()).unwrap();
        spawner
            .spawn(update_stick_states_task(
                spi,
                spi_acs,
                spi_ccs,
                controller_config,
            ))
            .unwrap()
    });
}
