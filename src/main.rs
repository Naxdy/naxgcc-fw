//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]
mod filter;
mod gcc_hid;
mod input;
mod stick;

use core::ops::Deref;

use defmt::{debug, info, Format};
use embassy_executor::Executor;
use embassy_rp::{
    bind_interrupts,
    flash::{Async, Flash, ERASE_SIZE},
    gpio::{self, Input},
    multicore::{spawn_core1, Stack},
    peripherals::USB,
    pwm::Pwm,
    spi::{self, Spi},
    usb::{Driver, InterruptHandler},
};
use gcc_hid::usb_transfer_loop;
use gpio::{Level, Output};
use input::input_loop;
use packed_struct::PackedStruct;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

const FLASH_SIZE: usize = 2 * 1024 * 1024;
const ADDR_OFFSET: u32 = 0x100000;

/// wrapper type because packed_struct doesn't implement float
/// packing by default for some reason
#[derive(Debug, Format, Clone, Default)]
pub struct PackedFloat(f32);

impl PackedStruct for PackedFloat {
    type ByteArray = [u8; 4];

    fn pack(&self) -> packed_struct::PackingResult<Self::ByteArray> {
        Ok(self.to_be_bytes())
    }

    fn unpack(src: &Self::ByteArray) -> packed_struct::PackingResult<Self> {
        Ok(Self(f32::from_be_bytes(*src)))
    }
}

impl Deref for PackedFloat {
    type Target = f32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

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

    let mut pwm_config: embassy_rp::pwm::Config = Default::default();
    pwm_config.top = 255;
    pwm_config.enable = true;
    pwm_config.compare_b = 255;

    let pwm_rumble = Pwm::new_output_b(p.PWM_CH4, p.PIN_25, pwm_config.clone());
    let pwm_brake = Pwm::new_output_b(p.PWM_CH6, p.PIN_29, pwm_config.clone());

    pwm_rumble.set_counter(255);
    pwm_brake.set_counter(0);

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
                pwm_rumble,
                pwm_brake,
                spi,
                spi_acs,
                spi_ccs,
            ))
            .unwrap()
    });
}
