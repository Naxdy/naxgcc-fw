//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]
mod gcc_hid;

use core::fmt::Write;
use defmt::{error, info, Debug2Format};
use gcc_hid::{GcConfig, GcReport};

use fugit::ExtU32;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use defmt_rtt as _;
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{
    gpio::FunctionUart,
    pac,
    uart::{UartConfig, UartPeripheral},
};

// Some traits we need
use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin, timer::CountDown};
use rp2040_hal::Clock;
use usb_device::{
    bus::UsbBusAllocator,
    device::{UsbDeviceBuilder, UsbVidPid},
};
use usbd_human_interface_device::{usb_class::UsbHidClassBuilder, UsbHidError};

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

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut poll_timer = timer.count_down();
    poll_timer.start(10.millis());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut gcc_state = GcReport::default();

    // usb parts
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut gcc = UsbHidClassBuilder::new()
        .add_device(GcConfig::default())
        .build(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x057e, 0x0337))
        .manufacturer("Naxdy")
        .product("NaxGCC")
        .serial_number("fleeb")
        .device_class(0)
        .device_protocol(0)
        .device_sub_class(0)
        .self_powered(false)
        .max_power(500)
        .max_packet_size_0(64)
        .build();

    let mut uart = UartPeripheral::new(
        pac.UART0,
        (
            pins.gpio0.into_mode::<FunctionUart>(),
            pins.gpio1.into_mode(),
        ),
        &mut pac.RESETS,
    )
    .enable(UartConfig::default(), clocks.peripheral_clock.freq())
    .unwrap();

    gcc_state.buttons_1.button_a = true;

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    info!("Bleg");
    let _ = uart.write_str("FLAR");
    loop {
        if poll_timer.wait().is_ok() {
            match gcc.device().write_report(&gcc_state) {
                Err(UsbHidError::WouldBlock) => {}
                Ok(_) => {}
                Err(e) => {
                    led_pin.set_high().unwrap();
                    error!("Error: {:?}", Debug2Format(&e));
                    panic!();
                }
            }
        }

        if usb_dev.poll(&mut [&mut gcc]) {}
    }
}
