use defmt::info;
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

#[derive(Clone, Debug, Default)]
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

    let end_time = Instant::now() + embassy_time::Duration::from_millis(1);
    let timer = Timer::at(end_time);

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

        loop_time = Instant::now() - loop_start;

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

    let (x_pos_filt, y_pos_filt) =
        kalman_state.run_kalman(x_z, y_z, &controller_config.astick_config, &filter_gains);

    let (shaped_x, shaped_y) = run_waveshaping(
        x_pos_filt,
        y_pos_filt,
        controller_config.astick_config.x_waveshaping,
        controller_config.astick_config.y_waveshaping,
        controlstick_waveshaping_values,
        &filter_gains,
    );

    let pos_x: f32 =
        filter_gains.smoothing.x * shaped_x + (1.0 - filter_gains.smoothing.x) * old_stick_pos.x;
    let pos_y =
        filter_gains.smoothing.y * shaped_y + (1.0 - filter_gains.smoothing.y) * old_stick_pos.y;
    old_stick_pos.x = pos_x;
    old_stick_pos.y = pos_y;

    let (shaped_cx, shaped_cy) = run_waveshaping(
        pos_cx,
        pos_cy,
        controller_config.cstick_config.x_waveshaping,
        controller_config.cstick_config.y_waveshaping,
        cstick_waveshaping_values,
        &filter_gains,
    );

    let old_cx_pos = old_stick_pos.cx;
    let old_cy_pos = old_stick_pos.cy;
    old_stick_pos.cx = shaped_cx;
    old_stick_pos.cy = shaped_cy;

    let x_weight_1 = filter_gains.c_smoothing.x;
    let x_weight_2 = 1.0 - x_weight_1;
    let y_weight_1 = filter_gains.c_smoothing.y;
    let y_weight_2 = 1.0 - y_weight_1;

    let pos_cx_filt = x_weight_1 * shaped_cx + x_weight_2 * old_cx_pos;
    let pos_cy_filt = y_weight_1 * shaped_cy + y_weight_2 * old_cy_pos;

    // phob optionally runs a median filter here, but we leave it for now

    let (mut remapped_x, mut remapped_y) = notch_remap(
        pos_x,
        pos_y,
        controlstick_params,
        controller_config,
        Stick::ControlStick,
    );
    let (mut remapped_cx, mut remapped_cy) = notch_remap(
        pos_cx_filt,
        pos_cy_filt,
        cstick_params,
        controller_config,
        Stick::CStick,
    );
    let (remapped_x_unfiltered, remapped_y_unfiltered) = notch_remap(
        raw_stick_values.ax_linearized,
        raw_stick_values.ay_linearized,
        controlstick_params,
        controller_config,
        Stick::ControlStick,
    );
    let (remapped_cx_unfiltered, remapped_cy_unfiltered) = notch_remap(
        raw_stick_values.cx_linearized,
        raw_stick_values.cy_linearized,
        cstick_params,
        controller_config,
        Stick::CStick,
    );

    remapped_x = fminf(125., fmaxf(-125., remapped_x));
    remapped_y = fminf(125., fmaxf(-125., remapped_y));
    remapped_cx = fminf(125., fmaxf(-125., remapped_cx));
    remapped_cy = fminf(125., fmaxf(-125., remapped_cy));
    raw_stick_values.ax_unfiltered = fminf(125., fmaxf(-125., remapped_x_unfiltered));
    raw_stick_values.ay_unfiltered = fminf(125., fmaxf(-125., remapped_y_unfiltered));
    raw_stick_values.cx_unfiltered = fminf(125., fmaxf(-125., remapped_cx_unfiltered));
    raw_stick_values.cy_unfiltered = fminf(125., fmaxf(-125., remapped_cy_unfiltered));

    let mut out_stick_state = current_stick_state.clone();

    let diff_x = (remapped_x + FLOAT_ORIGIN) - current_stick_state.ax as f32;
    if (diff_x > (1.0 + STICK_HYST_VAL)) || (diff_x < -STICK_HYST_VAL) {
        out_stick_state.ax = (remapped_x + FLOAT_ORIGIN) as u8;
    }
    let diff_y = (remapped_y + FLOAT_ORIGIN) - current_stick_state.ay as f32;
    if (diff_y > (1.0 + STICK_HYST_VAL)) || (diff_y < -STICK_HYST_VAL) {
        out_stick_state.ay = (remapped_y + FLOAT_ORIGIN) as u8;
    }

    let diff_cx = (remapped_cx + FLOAT_ORIGIN) - current_stick_state.cx as f32;
    if (diff_cx > (1.0 + STICK_HYST_VAL)) || (diff_cx < -STICK_HYST_VAL) {
        out_stick_state.cx = (remapped_cx + FLOAT_ORIGIN) as u8;
    }
    let diff_cy = (remapped_cy + FLOAT_ORIGIN) - current_stick_state.cy as f32;
    if (diff_cy > (1.0 + STICK_HYST_VAL)) || (diff_cy < -STICK_HYST_VAL) {
        out_stick_state.cy = (remapped_cy + FLOAT_ORIGIN) as u8;
    }

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
    pwm_rumble: Pwm<'static, PWM_CH4>,
    pwm_brake: Pwm<'static, PWM_CH6>,
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

        loop {
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

            STICK_SIGNAL.signal(current_stick_state.clone());
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
