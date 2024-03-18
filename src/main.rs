//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]
mod gcc_hid;
mod input;

use defmt::{debug, info};
use embassy_executor::Executor;
use embassy_rp::{
    bind_interrupts,
    flash::{Async, Flash, ERASE_SIZE},
    gpio::{self, Input},
    multicore::{spawn_core1, Stack},
    peripherals::USB,
    spi::{self, Spi},
    usb::{Driver, InterruptHandler},
};
use gcc_hid::usb_transfer_loop;
use gpio::{Level, Output};
use input::input_loop;
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

    let mut flash = Flash::<_, Async, FLASH_SIZE>::new(p.FLASH, p.DMA_CH0);

    let mut uid = [0u8; 8];
    flash.blocking_unique_id(&mut uid).unwrap();

    flash
        .blocking_erase(ADDR_OFFSET, ADDR_OFFSET + ERASE_SIZE as u32)
        .unwrap();

    flash.blocking_write(ADDR_OFFSET, &[0xAB]).unwrap();

    debug!("Read unique id: {:02X}", uid);

    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| spawner.spawn(usb_transfer_loop(driver, uid)).unwrap());
    });

    let executor0 = EXECUTOR0.init(Executor::new());
    info!("Initialized.");

    let mosi = p.PIN_7;
    let miso = p.PIN_4;
    let spi_clk = p.PIN_6;

    let p_acs = p.PIN_24;
    let p_ccs = p.PIN_23;

    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = 3000 * 1000;

    let spi = Spi::new_blocking(p.SPI0, spi_clk, mosi, miso, spi_cfg);

    let spi_acs = Output::new(p_acs, Level::High); // active low
    let spi_ccs = Output::new(p_ccs, Level::High); // active low

    executor0.run(|spawner| {
        spawner
            .spawn(input_loop(
                flash,
                Input::new(p.PIN_20, gpio::Pull::Up),
                Input::new(p.PIN_17, gpio::Pull::Up),
                Input::new(p.PIN_16, gpio::Pull::Up),
                Input::new(p.PIN_11, gpio::Pull::Up),
                Input::new(p.PIN_9, gpio::Pull::Up),
                Input::new(p.PIN_10, gpio::Pull::Up),
                Input::new(p.PIN_8, gpio::Pull::Up),
                Input::new(p.PIN_22, gpio::Pull::Up),
                Input::new(p.PIN_21, gpio::Pull::Up),
                Input::new(p.PIN_18, gpio::Pull::Up),
                Input::new(p.PIN_19, gpio::Pull::Up),
                Input::new(p.PIN_5, gpio::Pull::Up),
                Input::new(p.PIN_25, gpio::Pull::Up),
                Input::new(p.PIN_29, gpio::Pull::Up),
                spi,
                spi_acs,
                spi_ccs,
            ))
            .unwrap()
    });
}
