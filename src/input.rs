use defmt::{debug, info};
use embassy_rp::{
    clocks::RoscRng,
    flash::{Async, Flash, ERASE_SIZE},
    gpio::{Input, Output, Pin},
    peripherals::{
        DMA_CH0, FLASH, PIN_10, PIN_11, PIN_15, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21,
        PIN_22, PIN_23, PIN_24, PIN_25, PIN_29, PIN_5, PIN_8, PIN_9, SPI0,
    },
    spi::Spi,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use rand::RngCore;

use crate::{gcc_hid::GcReport, ADDR_OFFSET, FLASH_SIZE};

pub static GCC_SIGNAL: Signal<CriticalSectionRawMutex, GcReport> = Signal::new();

#[derive(PartialEq, Eq)]
enum Stick {
    ControlStick,
    CStick,
}

#[derive(PartialEq, Eq)]
enum StickAxis {
    XAxis,
    YAxis,
}

fn read_ext_adc<'a, Acs: Pin, Ccs: Pin>(
    which_stick: Stick,
    which_axis: StickAxis,
    spi: &mut Spi<'a, SPI0, embassy_rp::spi::Blocking>,
    spi_acs: &mut Output<'a, Acs>,
    spi_ccs: &mut Output<'a, Ccs>,
) -> u16 {
    let mut buf = [0b11010000; 3];

    if which_axis == StickAxis::YAxis {
        buf = [0b11110000; 3];
    }

    if which_stick == Stick::CStick {
        spi_acs.set_low();
    } else {
        spi_ccs.set_low();
    }

    spi.blocking_transfer_in_place(&mut buf).unwrap();

    let temp_value =
        (((buf[0] & 0b00000111) as u16) << 9) | (buf[1] as u16) << 1 | (buf[2] as u16) >> 7;

    if which_stick == Stick::ControlStick {
        spi_acs.set_high();
    } else {
        spi_ccs.set_high();
    }

    return temp_value;
}

#[embassy_executor::task]
pub async fn input_loop(
    mut flash: Flash<'static, FLASH, Async, FLASH_SIZE>,
    btn_z: Input<'static, PIN_20>,
    btn_a: Input<'static, PIN_17>,
    btn_b: Input<'static, PIN_16>,
    btn_dright: Input<'static, PIN_11>,
    btn_dup: Input<'static, PIN_9>,
    btn_ddown: Input<'static, PIN_10>,
    btn_dleft: Input<'static, PIN_8>,
    btn_l: Input<'static, PIN_22>,
    btn_r: Input<'static, PIN_21>,
    btn_x: Input<'static, PIN_18>,
    btn_y: Input<'static, PIN_19>,
    btn_start: Input<'static, PIN_5>,
    btn_rumble: Input<'static, PIN_25>,
    btn_brake: Input<'static, PIN_29>,
    mut spi: Spi<'static, SPI0, embassy_rp::spi::Blocking>,
    mut spi_acs: Output<'static, PIN_24>,
    mut spi_ccs: Output<'static, PIN_23>,
) -> ! {
    let mut gcc_state = GcReport::default();

    let mut rng = RoscRng;

    let mut uid = [0u8; 1];
    flash.blocking_read(ADDR_OFFSET, &mut uid).unwrap();

    debug!("Read from flash: {:02X}", uid);

    loop {
        gcc_state.buttons_1.button_a = btn_z.is_low();
        gcc_state.stick_x = rng.next_u32() as u8;
        gcc_state.stick_y = rng.next_u32() as u8;

        GCC_SIGNAL.signal(gcc_state);
    }
}
