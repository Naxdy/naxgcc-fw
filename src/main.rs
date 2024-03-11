//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]
mod flash_mem;
mod gcc_hid;
mod input;

use defmt::{error, info};
use gcc_hid::GcConfig;

use fugit::{ExtU32, RateExtU32};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use defmt_rtt as _;
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{
    gpio::FunctionSpi,
    multicore::{Multicore, Stack},
    pac, Spi,
};

// Some traits we need
use embedded_hal::{
    blocking::spi::Transfer,
    digital::v2::OutputPin,
    spi::{FullDuplex, MODE_0},
    timer::CountDown,
};

use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_human_interface_device::usb_class::UsbHidClassBuilder;

use crate::{
    flash_mem::{read_from_flash, write_to_flash},
    gcc_hid::usb_transfer_loop,
    input::{input_loop, BasicInputs},
};

static mut CORE1_STACK: Stack<4096> = Stack::new();

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let mut sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // usb parts
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    unsafe {
        let some_byte: u8 = 0xAB;
        info!("Byte to be written is {:02X}", some_byte);
        write_to_flash(some_byte);
        let r = read_from_flash();
        info!("Byte read from flash is {:02X}", r);
    }

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let _transfer_loop = core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            let mut poll_timer = timer.count_down();
            poll_timer.start(1.millis());

            usb_transfer_loop(usb_bus, poll_timer)
        })
        .unwrap();

    info!("Initialized");

    let mut ccs = pins.gpio23.into_push_pull_output();
    let mut acs = pins.gpio24.into_push_pull_output();

    ccs.set_low();
    acs.set_low();

    let spi_device = pac.SPI0;

    let clk = pins.gpio6.into_function::<FunctionSpi>();
    let tx = pins.gpio7.into_function::<FunctionSpi>();
    let rx = pins.gpio4.into_function::<FunctionSpi>();

    let spi_pin_layout = (tx, rx, clk);

    let mut spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
        &mut pac.RESETS,
        3_000_000u32.Hz(),
        3_000_000u32.Hz(),
        MODE_0,
    );

    let mut w = [0b11010000u8; 3];

    info!("W is {}", w);

    let r = spi.transfer(&mut w);

    match r {
        Ok(t) => {
            info!("T is {}", t)
        }
        Err(e) => {
            error!("SPI transfer failed: {}", e);
        }
    }

    input_loop(BasicInputs {
        button_a: pins.gpio17.into_pull_up_input(),
        button_b: pins.gpio16.into_pull_up_input(),
        button_x: pins.gpio18.into_pull_up_input(),
        button_y: pins.gpio19.into_pull_up_input(),
        button_z: pins.gpio20.into_pull_up_input(),
        button_r: pins.gpio21.into_pull_up_input(),
        button_l: pins.gpio22.into_pull_up_input(),
        dpad_left: pins.gpio8.into_pull_up_input(),
        dpad_up: pins.gpio9.into_pull_up_input(),
        dpad_down: pins.gpio10.into_pull_up_input(),
        dpad_right: pins.gpio11.into_pull_up_input(),
        button_start: pins.gpio5.into_pull_up_input(),
    });
}
