use defmt::{debug, info, trace, Format};
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
use embassy_time::{Duration, Instant, Timer};
use libm::{fmaxf, fminf};

use crate::{
    config::ControllerConfig,
    filter::{run_waveshaping, FilterGains, KalmanState, WaveshapingValues, FILTER_GAINS},
    gcc_hid::GcReport,
    helpers::XyValuePair,
    stick::{linearize, notch_remap, StickParams},
    FLASH_SIZE,
};

pub static GCC_SIGNAL: Signal<CriticalSectionRawMutex, GcReport> = Signal::new();

static STICK_SIGNAL: Signal<CriticalSectionRawMutex, StickState> = Signal::new();
const STICK_HYST_VAL: f32 = 0.3;
const FLOAT_ORIGIN: f32 = 127.5;

#[derive(Clone, Debug, Default)]
struct StickState {
    ax: u8,
    ay: u8,
    cx: u8,
    cy: u8,
}

#[derive(Clone, Debug, Default)]
struct StickPositions {
    x: f32,
    y: f32,
    cx: f32,
    cy: f32,
}

#[derive(Clone, Debug, Default, Format)]
struct RawStickValues {
    a_linearized: XyValuePair<f32>,
    c_linearized: XyValuePair<f32>,
    a_raw: XyValuePair<f32>,
    c_raw: XyValuePair<f32>,
    a_unfiltered: XyValuePair<f32>,
    c_unfiltered: XyValuePair<f32>,
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

    if which_stick == Stick::ControlStick {
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
    current_stick_state: &StickState,
    controlstick_params: &StickParams,
    cstick_params: &StickParams,
    controller_config: &ControllerConfig,
    filter_gains: &FilterGains,
    controlstick_waveshaping_values: &mut WaveshapingValues,
    cstick_waveshaping_values: &mut WaveshapingValues,
    old_stick_pos: &mut StickPositions,
    raw_stick_values: &mut RawStickValues,
    kalman_state: &mut KalmanState,
) -> StickState {
    let mut adc_count = 0u32;
    let mut ax_sum = 0u32;
    let mut ay_sum = 0u32;
    let mut cx_sum = 0u32;
    let mut cy_sum = 0u32;

    // TODO: lower interval possible?

    let end_time = Instant::now() + embassy_time::Duration::from_micros(500);
    let mut loop_time = Duration::from_millis(0);

    while Instant::now() < end_time - loop_time {
        let loop_start = Instant::now();

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
        loop_time = Instant::now() - loop_start;
    }

    let raw_controlstick = XyValuePair {
        x: (ax_sum as f32) / (adc_count as f32) / 4096.0f32,
        y: (ay_sum as f32) / (adc_count as f32) / 4096.0f32,
    };
    let raw_cstick = XyValuePair {
        x: (cx_sum as f32) / (adc_count as f32) / 4096.0f32,
        y: (cy_sum as f32) / (adc_count as f32) / 4096.0f32,
    };

    trace!("Raw Control Stick: {}", raw_controlstick);

    raw_stick_values.a_raw = raw_controlstick;
    raw_stick_values.c_raw = raw_cstick;

    let x_z = linearize(raw_controlstick.x, &controlstick_params.fit_coeffs.x);
    let y_z = linearize(raw_controlstick.y, &controlstick_params.fit_coeffs.y);

    let pos_cx = linearize(raw_cstick.x, &cstick_params.fit_coeffs.x);
    let pos_cy = linearize(raw_cstick.y, &cstick_params.fit_coeffs.y);

    raw_stick_values.a_linearized.x = x_z;
    raw_stick_values.a_linearized.y = y_z;
    raw_stick_values.c_linearized.x = pos_cx;
    raw_stick_values.c_linearized.y = pos_cy;

    trace!("Raw Stick Values 001: {:?}", raw_stick_values);

    let (x_pos_filt, y_pos_filt) =
        kalman_state.run_kalman(x_z, y_z, &controller_config.astick_config, &filter_gains);

    let shaped_astick = match run_waveshaping(
        x_pos_filt,
        y_pos_filt,
        controller_config.astick_config.x_waveshaping,
        controller_config.astick_config.y_waveshaping,
        controlstick_waveshaping_values,
        &filter_gains,
    ) {
        (x, y) => XyValuePair { x, y },
    };

    trace!("Shaped Controlstick: {}", shaped_astick);

    let pos_x: f32 = filter_gains.smoothing.x * shaped_astick.x
        + (1.0 - filter_gains.smoothing.x) * old_stick_pos.x;
    let pos_y = filter_gains.smoothing.y * shaped_astick.y
        + (1.0 - filter_gains.smoothing.y) * old_stick_pos.y;
    old_stick_pos.x = pos_x;
    old_stick_pos.y = pos_y;

    let shaped_cstick = match run_waveshaping(
        pos_cx,
        pos_cy,
        controller_config.cstick_config.x_waveshaping,
        controller_config.cstick_config.y_waveshaping,
        cstick_waveshaping_values,
        &filter_gains,
    ) {
        (x, y) => XyValuePair { x, y },
    };

    let old_c_pos = XyValuePair {
        x: old_stick_pos.cx,
        y: old_stick_pos.cy,
    };
    old_stick_pos.cx = shaped_cstick.x;
    old_stick_pos.cy = shaped_cstick.y;

    let x_weight_1 = filter_gains.c_smoothing.x;
    let x_weight_2 = 1.0 - x_weight_1;
    let y_weight_1 = filter_gains.c_smoothing.y;
    let y_weight_2 = 1.0 - y_weight_1;

    let pos_cx_filt = x_weight_1 * shaped_cstick.x + x_weight_2 * old_c_pos.x;
    let pos_cy_filt = y_weight_1 * shaped_cstick.y + y_weight_2 * old_c_pos.y;

    // phob optionally runs a median filter here, but we leave it for now

    trace!("Controlstick position: {}, {}", pos_x, pos_y);

    let mut remapped = match notch_remap(
        pos_x,
        pos_y,
        controlstick_params,
        controller_config,
        Stick::ControlStick,
        false,
    ) {
        (x, y) => XyValuePair { x, y },
    };
    let mut remapped_c = match notch_remap(
        pos_cx_filt,
        pos_cy_filt,
        cstick_params,
        controller_config,
        Stick::CStick,
        false,
    ) {
        (x, y) => XyValuePair { x, y },
    };
    let remapped_unfiltered = match notch_remap(
        raw_stick_values.a_linearized.x,
        raw_stick_values.a_linearized.y,
        controlstick_params,
        controller_config,
        Stick::ControlStick,
        false,
    ) {
        (x, y) => XyValuePair { x, y },
    };
    let remapped_c_unfiltered = match notch_remap(
        raw_stick_values.c_linearized.x,
        raw_stick_values.c_linearized.y,
        cstick_params,
        controller_config,
        Stick::CStick,
        false,
    ) {
        (x, y) => XyValuePair { x, y },
    };

    trace!("Remapped Control Stick: {}", remapped);

    remapped = XyValuePair {
        x: fminf(125., fmaxf(-125., remapped.x)),
        y: fminf(125., fmaxf(-125., remapped.y)),
    };
    remapped_c = XyValuePair {
        x: fminf(125., fmaxf(-125., remapped_c.x)),
        y: fminf(125., fmaxf(-125., remapped_c.y)),
    };
    raw_stick_values.a_unfiltered.x = fminf(125., fmaxf(-125., remapped_unfiltered.x));
    raw_stick_values.a_unfiltered.y = fminf(125., fmaxf(-125., remapped_unfiltered.y));
    raw_stick_values.c_unfiltered.x = fminf(125., fmaxf(-125., remapped_c_unfiltered.x));
    raw_stick_values.c_unfiltered.y = fminf(125., fmaxf(-125., remapped_c_unfiltered.y));

    let mut out_stick_state = current_stick_state.clone();

    let diff_x = (remapped.x + FLOAT_ORIGIN) - current_stick_state.ax as f32;
    if (diff_x > (1.0 + STICK_HYST_VAL)) || (diff_x < -STICK_HYST_VAL) {
        out_stick_state.ax = (remapped.x + FLOAT_ORIGIN) as u8;
    }
    let diff_y = (remapped.y + FLOAT_ORIGIN) - current_stick_state.ay as f32;
    if (diff_y > (1.0 + STICK_HYST_VAL)) || (diff_y < -STICK_HYST_VAL) {
        out_stick_state.ay = (remapped.y + FLOAT_ORIGIN) as u8;
    }

    let diff_cx = (remapped_c.x + FLOAT_ORIGIN) - current_stick_state.cx as f32;
    if (diff_cx > (1.0 + STICK_HYST_VAL)) || (diff_cx < -STICK_HYST_VAL) {
        out_stick_state.cx = (remapped_c.x + FLOAT_ORIGIN) as u8;
    }
    let diff_cy = (remapped_c.y + FLOAT_ORIGIN) - current_stick_state.cy as f32;
    if (diff_cy > (1.0 + STICK_HYST_VAL)) || (diff_cy < -STICK_HYST_VAL) {
        out_stick_state.cy = (remapped_c.y + FLOAT_ORIGIN) as u8;
    }

    trace!(
        "Control stick: {}, {}, C-stick: {}, {}",
        out_stick_state.ax,
        out_stick_state.ay,
        out_stick_state.cx,
        out_stick_state.cy
    );

    out_stick_state
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
    // pwm_rumble: Pwm<'static, PWM_CH4>,
    // pwm_brake: Pwm<'static, PWM_CH6>,
    mut spi: Spi<'static, SPI0, embassy_rp::spi::Blocking>,
    mut spi_acs: Output<'static, PIN_24>,
    mut spi_ccs: Output<'static, PIN_23>,
) {
    if btn_a.is_low() && btn_x.is_low() && btn_y.is_low() {
        info!("Detected reset button press, booting into flash.");
        embassy_rp::rom_data::reset_to_usb_boot(0, 0);
        loop {}
    }

    let mut gcc_state = GcReport::default();

    // Set the stick states to the center
    gcc_state.stick_x = 127;
    gcc_state.stick_y = 127;
    gcc_state.cstick_x = 127;
    gcc_state.cstick_y = 127;

    let controller_config = ControllerConfig::from_flash_memory(&mut flash).unwrap();

    let controlstick_params = StickParams::from_stick_config(&controller_config.astick_config);
    let cstick_params = StickParams::from_stick_config(&controller_config.cstick_config);

    let filter_gains = FILTER_GAINS.get_normalized_gains(&controller_config);

    let stick_state_fut = async {
        let mut current_stick_state = StickState {
            ax: 127,
            ay: 127,
            cx: 127,
            cy: 127,
        };
        let mut raw_stick_values = RawStickValues::default();
        let mut old_stick_pos = StickPositions::default();
        let mut cstick_waveshaping_values = WaveshapingValues::default();
        let mut controlstick_waveshaping_values = WaveshapingValues::default();
        let mut kalman_state = KalmanState::default();

        let mut last_loop_time = Instant::now();

        loop {
            let timer = Timer::after_micros(1000);

            current_stick_state = update_stick_states(
                &mut spi,
                &mut spi_acs,
                &mut spi_ccs,
                &current_stick_state,
                &controlstick_params,
                &cstick_params,
                &controller_config,
                &filter_gains,
                &mut controlstick_waveshaping_values,
                &mut cstick_waveshaping_values,
                &mut old_stick_pos,
                &mut raw_stick_values,
                &mut kalman_state,
            )
            .await;

            timer.await;

            match (Instant::now() - last_loop_time).as_micros() {
                a if a > 1100 => {
                    debug!("Loop took {} us", a);
                }
                _ => {}
            };

            last_loop_time = Instant::now();

            STICK_SIGNAL.signal(current_stick_state.clone());
        }
    };

    let input_fut = async {
        loop {
            let timer = Timer::after_micros(500);

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

            timer.await;

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
