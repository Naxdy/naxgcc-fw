use core::task::Poll;

use defmt::{debug, Format};
use embassy_futures::{join::join, yield_now};
use embassy_rp::{
    flash::{Async, Flash},
    gpio::{Input, Output, Pin},
    peripherals::{
        FLASH, PIN_10, PIN_11, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_23,
        PIN_24, PIN_5, PIN_8, PIN_9, PWM_CH4, PWM_CH6, SPI0,
    },
    pwm::Pwm,
    spi::Spi,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Instant, Timer};
use packed_struct::derive::PackedStruct;

use crate::{
    filter::{run_waveshaping, WaveshapingValues},
    gcc_hid::GcReport,
    stick::{linearize, run_kalman, FilterGains, StickParams},
    PackedFloat, ADDR_OFFSET, FLASH_SIZE,
};

pub static GCC_SIGNAL: Signal<CriticalSectionRawMutex, GcReport> = Signal::new();

static STICK_SIGNAL: Signal<CriticalSectionRawMutex, StickState> = Signal::new();

#[derive(Debug, Clone, Default, Format, PackedStruct)]
#[packed_struct(endian = "msb")]
pub struct ControllerConfig {
    #[packed_field(size_bits = "8")]
    pub config_version: u8,
    #[packed_field(size_bits = "8")]
    pub ax_waveshaping: u8,
    #[packed_field(size_bits = "8")]
    pub ay_waveshaping: u8,
    #[packed_field(size_bits = "8")]
    pub cx_waveshaping: u8,
    #[packed_field(size_bits = "8")]
    pub cy_waveshaping: u8,
}

struct StickState {
    ax: u8,
    ay: u8,
    cx: u8,
    cy: u8,
}

struct StickPositions {
    x: f32,
    y: f32,
    cx: f32,
    cy: f32,
}

struct RawStickValues {
    ax_linearized: f32,
    ay_linearized: f32,
    cx_linearized: f32,
    cy_linearized: f32,
    ax_raw: f32,
    ay_raw: f32,
    cx_raw: f32,
    cy_raw: f32,
    ax_unfiltered: f32,
    ay_unfiltered: f32,
    cx_unfiltered: f32,
    cy_unfiltered: f32,
}

#[derive(PartialEq, Eq)]
pub enum Stick {
    ControlStick,
    CStick,
}

#[derive(PartialEq, Eq)]
pub enum StickAxis {
    XAxis,
    YAxis,
}

fn read_ext_adc<'a, Acs: Pin, Ccs: Pin, I: embassy_rp::spi::Instance, M: embassy_rp::spi::Mode>(
    which_stick: Stick,
    which_axis: StickAxis,
    spi: &mut Spi<'a, I, M>,
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

/// Gets the average stick state over a 1ms interval in a non-blocking fashion.
async fn update_stick_states<
    'a,
    Acs: Pin,
    Ccs: Pin,
    I: embassy_rp::spi::Instance,
    M: embassy_rp::spi::Mode,
>(
    mut spi: &mut Spi<'a, I, M>,
    mut spi_acs: &mut Output<'a, Acs>,
    mut spi_ccs: &mut Output<'a, Ccs>,
    controlstick_params: &StickParams,
    cstick_params: &StickParams,
    controller_config: &ControllerConfig,
    filter_gains: &FilterGains,
    controlstick_waveshaping_values: &mut WaveshapingValues,
    cstick_waveshaping_values: &mut WaveshapingValues,
    old_stick_pos: &mut StickPositions,
    raw_stick_values: &mut RawStickValues,
) {
    let mut adc_count = 0u32;
    let mut ax_sum = 0u32;
    let mut ay_sum = 0u32;
    let mut cx_sum = 0u32;
    let mut cy_sum = 0u32;

    // TODO: lower interval possible?
    let mut timer = Timer::at(Instant::now() + embassy_time::Duration::from_millis(1));

    while embassy_futures::poll_once(&mut timer) != Poll::Ready(()) {
        adc_count += 1;
        ax_sum += read_ext_adc(
            Stick::ControlStick,
            StickAxis::XAxis,
            &mut spi,
            &mut spi_acs,
            &mut spi_ccs,
        ) as u32;
        ay_sum += read_ext_adc(
            Stick::ControlStick,
            StickAxis::YAxis,
            &mut spi,
            &mut spi_acs,
            &mut spi_ccs,
        ) as u32;
        cx_sum += read_ext_adc(
            Stick::CStick,
            StickAxis::XAxis,
            &mut spi,
            &mut spi_acs,
            &mut spi_ccs,
        ) as u32;
        cy_sum += read_ext_adc(
            Stick::CStick,
            StickAxis::YAxis,
            &mut spi,
            &mut spi_acs,
            &mut spi_ccs,
        ) as u32;

        // with this, we can poll the sticks at 1000Hz (ish), while updating
        // the rest of the controller (the buttons) much faster, to ensure
        // better input integrity for button inputs.
        yield_now().await;
    }

    timer.await;

    let raw_controlstick_x = (ax_sum as f32) / (adc_count as f32) / 4096.0f32;
    let raw_controlstick_y = (ay_sum as f32) / (adc_count as f32) / 4096.0f32;
    let raw_cstick_x = (cx_sum as f32) / (adc_count as f32) / 4096.0f32;
    let raw_cstick_y = (cy_sum as f32) / (adc_count as f32) / 4096.0f32;

    raw_stick_values.ax_raw = raw_controlstick_x;
    raw_stick_values.ay_raw = raw_controlstick_y;
    raw_stick_values.cx_raw = raw_cstick_x;
    raw_stick_values.cy_raw = raw_cstick_y;

    let x_z = linearize(raw_controlstick_x, &controlstick_params.fit_coeffs_x);
    let y_z = linearize(raw_controlstick_y, &controlstick_params.fit_coeffs_y);

    let pos_cx = linearize(raw_cstick_x, &cstick_params.fit_coeffs_x);
    let pos_cy = linearize(raw_cstick_y, &cstick_params.fit_coeffs_y);

    raw_stick_values.ax_linearized = x_z;
    raw_stick_values.ay_linearized = y_z;
    raw_stick_values.cx_linearized = pos_cx;
    raw_stick_values.cy_linearized = pos_cy;

    let (x_pos_filt, y_pos_filt) = run_kalman(x_z, y_z, controller_config, filter_gains);

    let (shaped_x, shaped_y) = run_waveshaping(
        x_pos_filt,
        y_pos_filt,
        Stick::ControlStick,
        controlstick_waveshaping_values,
        controller_config,
        filter_gains,
    );

    let pos_x =
        filter_gains.x_smoothing * shaped_x + (1.0 - filter_gains.x_smoothing) * old_stick_pos.x;
    let pos_y =
        filter_gains.y_smoothing * shaped_y + (1.0 - filter_gains.y_smoothing) * old_stick_pos.y;
    old_stick_pos.x = pos_x;
    old_stick_pos.y = pos_y;

    let (shaped_cx, shaped_cy) = run_waveshaping(
        pos_cx,
        pos_cy,
        Stick::CStick,
        cstick_waveshaping_values,
        controller_config,
        filter_gains,
    );

    let old_cx_pos = old_stick_pos.cx;
    let old_cy_pos = old_stick_pos.cy;
    old_stick_pos.cx = shaped_cx;
    old_stick_pos.cy = shaped_cy;

    let x_weight_1 = filter_gains.c_xsmoothing;
    let x_weight_2 = 1.0 - x_weight_1;
    let y_weight_1 = filter_gains.c_ysmoothing;
    let y_weight_2 = 1.0 - y_weight_1;

    let pos_cx_filt = x_weight_1 * shaped_cx + x_weight_2 * old_cx_pos;
    let pos_cy_filt = y_weight_1 * shaped_cy + y_weight_2 * old_cy_pos;

    // phob optionally runs a median filter here, but we leave it for now

    STICK_SIGNAL.signal(StickState {
        ax: 127,
        ay: 127,
        cx: 127,
        cy: 127,
    })
}

fn update_button_states<
    A: Pin,
    B: Pin,
    X: Pin,
    Y: Pin,
    Start: Pin,
    L: Pin,
    R: Pin,
    Z: Pin,
    DLeft: Pin,
    DRight: Pin,
    DUp: Pin,
    DDown: Pin,
>(
    gcc_state: &mut GcReport,
    btn_a: &Input<'_, A>,
    btn_b: &Input<'_, B>,
    btn_x: &Input<'_, X>,
    btn_y: &Input<'_, Y>,
    btn_start: &Input<'_, Start>,
    btn_l: &Input<'_, L>,
    btn_r: &Input<'_, R>,
    btn_z: &Input<'_, Z>,
    btn_dleft: &Input<'_, DLeft>,
    btn_dright: &Input<'_, DRight>,
    btn_dup: &Input<'_, DUp>,
    btn_ddown: &Input<'_, DDown>,
) {
    gcc_state.buttons_1.button_a = btn_a.is_low();
    gcc_state.buttons_1.button_b = btn_b.is_low();
    gcc_state.buttons_1.button_x = btn_x.is_low();
    gcc_state.buttons_1.button_y = btn_y.is_low();
    gcc_state.buttons_2.button_z = btn_z.is_low();
    gcc_state.buttons_2.button_start = btn_start.is_low();
    gcc_state.buttons_2.button_l = btn_l.is_low();
    gcc_state.buttons_2.button_r = btn_r.is_low();
    gcc_state.buttons_1.dpad_left = btn_dleft.is_low();
    gcc_state.buttons_1.dpad_right = btn_dright.is_low();
    gcc_state.buttons_1.dpad_up = btn_dup.is_low();
    gcc_state.buttons_1.dpad_down = btn_ddown.is_low();
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
    pwm_rumble: Pwm<'static, PWM_CH4>,
    pwm_brake: Pwm<'static, PWM_CH6>,
    mut spi: Spi<'static, SPI0, embassy_rp::spi::Blocking>,
    mut spi_acs: Output<'static, PIN_24>,
    mut spi_ccs: Output<'static, PIN_23>,
) {
    let mut gcc_state = GcReport::default();

    // Set the stick states to the center
    gcc_state.stick_x = 127;
    gcc_state.stick_y = 127;
    gcc_state.cstick_x = 127;
    gcc_state.cstick_y = 127;

    let mut uid = [0u8; 1];
    flash.blocking_read(ADDR_OFFSET, &mut uid).unwrap();

    debug!("Read from flash: {:02X}", uid);

    // TODO: load controller config here

    let stick_state_fut = async {
        loop {
            // update_stick_states(&mut spi, &mut spi_acs, &mut spi_ccs, 1.0).await;
        }
    };

    let input_fut = async {
        loop {
            update_button_states(
                &mut gcc_state,
                &btn_a,
                &btn_b,
                &btn_x,
                &btn_y,
                &btn_start,
                &btn_l,
                &btn_r,
                &btn_z,
                &btn_dleft,
                &btn_dright,
                &btn_dup,
                &btn_ddown,
            );

            yield_now().await;

            // not every loop pass is going to update the stick state
            match STICK_SIGNAL.try_take() {
                Some(stick_state) => {
                    gcc_state.stick_x = stick_state.ax;
                    gcc_state.stick_y = stick_state.ay;
                    gcc_state.cstick_x = stick_state.cx;
                    gcc_state.cstick_y = stick_state.cy;
                }
                None => (),
            }

            GCC_SIGNAL.signal(gcc_state);
        }
    };

    join(input_fut, stick_state_fut).await;
}
