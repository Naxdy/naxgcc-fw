use defmt::{debug, info, trace, Format};
use embassy_futures::{join::join, yield_now};
use embassy_rp::{
    flash::{Async, Flash},
    gpio::{AnyPin, Input, Output, Pin},
    peripherals::{
        FLASH, PIN_10, PIN_11, PIN_16, PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_22, PIN_23,
        PIN_24, PIN_5, PIN_8, PIN_9, PWM_CH4, PWM_CH6, SPI0, SPI1,
    },
    pwm::Pwm,
    spi::{Blocking, Spi},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
    pubsub::PubSubChannel,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Timer};
use libm::{fmaxf, fminf};

use crate::{
    config::{ControllerConfig, SIGNAL_IS_CALIBRATING, SIGNAL_OVERRIDE_GCC_STATE},
    filter::{run_waveshaping, FilterGains, KalmanState, WaveshapingValues, FILTER_GAINS},
    gcc_hid::GcReport,
    helpers::XyValuePair,
    stick::{linearize, notch_remap, StickParams},
    FLASH_SIZE,
};

/// Used to send the button state to the usb task and the calibration task
pub static CHANNEL_GCC_STATE: PubSubChannel<CriticalSectionRawMutex, GcReport, 1, 2, 1> =
    PubSubChannel::new();

/// Used to send the stick state from the stick task to the main input task
static SIGNAL_STICK_STATE: Signal<CriticalSectionRawMutex, StickState> = Signal::new();

/// Used to send the raw stick values for the calibration task
static SIGNAL_RAW_STICK_VALUES: Signal<CriticalSectionRawMutex, RawStickValues> = Signal::new();

pub static SPI_SHARED: Mutex<ThreadModeRawMutex, Option<Spi<'static, SPI0, Blocking>>> =
    Mutex::new(None);
pub static SPI_ACS_SHARED: Mutex<ThreadModeRawMutex, Option<Output<'static, AnyPin>>> =
    Mutex::new(None);
pub static SPI_CCS_SHARED: Mutex<ThreadModeRawMutex, Option<Output<'static, AnyPin>>> =
    Mutex::new(None);

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

#[derive(PartialEq, Eq, Debug, Clone, Format, Copy)]
pub enum Stick {
    ControlStick,
    CStick,
}

#[derive(PartialEq, Eq)]
pub enum StickAxis {
    XAxis,
    YAxis,
}

#[link_section = ".time_critical.read_ext_adc"]
pub fn read_ext_adc<
    'a,
    Acs: Pin,
    Ccs: Pin,
    I: embassy_rp::spi::Instance,
    M: embassy_rp::spi::Mode,
>(
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
/// Will wait until end_time is reached before continuing after reading the ADCs.
#[link_section = ".time_critical.update_stick_states"]
async fn update_stick_states(
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
    is_calibrating: bool,
) -> StickState {
    let mut adc_count = 0u32;
    let mut ax_sum = 0u32;
    let mut ay_sum = 0u32;
    let mut cx_sum = 0u32;
    let mut cy_sum = 0u32;

    let end_time = Instant::now() + Duration::from_micros(300); // this seems kinda magic, and it is, but

    let mut spi_unlocked = SPI_SHARED.lock().await;
    let mut spi_acs_unlocked = SPI_ACS_SHARED.lock().await;
    let mut spi_ccs_unlocked = SPI_CCS_SHARED.lock().await;

    let spi = spi_unlocked.as_mut().unwrap();
    let spi_acs = spi_acs_unlocked.as_mut().unwrap();
    let spi_ccs = spi_ccs_unlocked.as_mut().unwrap();

    // "do-while at home"
    while {
        let loop_start = Instant::now();

        adc_count += 1;
        ax_sum += read_ext_adc(Stick::ControlStick, StickAxis::XAxis, spi, spi_acs, spi_ccs) as u32;
        ay_sum += read_ext_adc(Stick::ControlStick, StickAxis::YAxis, spi, spi_acs, spi_ccs) as u32;
        cx_sum += read_ext_adc(Stick::CStick, StickAxis::XAxis, spi, spi_acs, spi_ccs) as u32;
        cy_sum += read_ext_adc(Stick::CStick, StickAxis::YAxis, spi, spi_acs, spi_ccs) as u32;

        let loop_end = Instant::now();
        loop_end < end_time - (loop_end - loop_start)
    } {}

    trace!("ADC Count: {}", adc_count);

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
        is_calibrating,
    ) {
        (x, y) => XyValuePair { x, y },
    };
    let mut remapped_c = match notch_remap(
        pos_cx_filt,
        pos_cy_filt,
        cstick_params,
        controller_config,
        Stick::CStick,
        is_calibrating,
    ) {
        (x, y) => XyValuePair { x, y },
    };
    let remapped_unfiltered = match notch_remap(
        raw_stick_values.a_linearized.x,
        raw_stick_values.a_linearized.y,
        controlstick_params,
        controller_config,
        Stick::ControlStick,
        is_calibrating,
    ) {
        (x, y) => XyValuePair { x, y },
    };
    let remapped_c_unfiltered = match notch_remap(
        raw_stick_values.c_linearized.x,
        raw_stick_values.c_linearized.y,
        cstick_params,
        controller_config,
        Stick::CStick,
        is_calibrating,
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

/// Task responsible for updating the button states.
/// Publishes the result to CHANNEL_GCC_STATE.
#[embassy_executor::task]
pub async fn update_button_state_task(
    btn_z: Input<'static, AnyPin>,
    btn_a: Input<'static, AnyPin>,
    btn_b: Input<'static, AnyPin>,
    btn_dright: Input<'static, AnyPin>,
    btn_dup: Input<'static, AnyPin>,
    btn_ddown: Input<'static, AnyPin>,
    btn_dleft: Input<'static, AnyPin>,
    btn_l: Input<'static, AnyPin>,
    btn_r: Input<'static, AnyPin>,
    btn_x: Input<'static, AnyPin>,
    btn_y: Input<'static, AnyPin>,
    btn_start: Input<'static, AnyPin>,
) {
    // upon loop entry, we check for the reset combo once
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

    let gcc_publisher = CHANNEL_GCC_STATE.publisher().unwrap();

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

        // not every loop pass is going to update the stick state
        if let Some(stick_state) = SIGNAL_STICK_STATE.try_take() {
            gcc_state.stick_x = stick_state.ax;
            gcc_state.stick_y = stick_state.ay;
            gcc_state.cstick_x = stick_state.cx;
            gcc_state.cstick_y = stick_state.cy;
        }

        // check for a gcc state override (usually coming from the config task)
        if let Some(override_gcc_state) = SIGNAL_OVERRIDE_GCC_STATE.try_take() {
            gcc_publisher.publish_immediate(override_gcc_state.report);
            Timer::after_millis(override_gcc_state.duration_ms).await;
        };

        gcc_publisher.publish_immediate(gcc_state);

        // give other tasks a chance to do something
        yield_now().await;
    }
}

/// Task responsible for updating the stick states.
/// Publishes the result to STICK_SIGNAL.
///
/// Has to run on core0 because it makes use of SPI0.
#[embassy_executor::task]
pub async fn update_stick_states_task(
    mut spi: Spi<'static, SPI0, embassy_rp::spi::Blocking>,
    mut spi_acs: Output<'static, AnyPin>,
    mut spi_ccs: Output<'static, AnyPin>,
    controller_config: ControllerConfig,
) {
    let controlstick_params = StickParams::from_stick_config(&controller_config.astick_config);
    let cstick_params = StickParams::from_stick_config(&controller_config.cstick_config);

    let filter_gains = FILTER_GAINS.get_normalized_gains(&controller_config);

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

    debug!("Entering stick update loop.");

    // the time at which the current loop iteration should end
    let mut end_time = Instant::now() + Duration::from_micros(1000);

    let mut is_calibrating = false;

    loop {
        let timer = Timer::at(end_time);

        current_stick_state = update_stick_states(
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
            is_calibrating,
        )
        .await;

        timer.await;
        end_time += Duration::from_micros(1000);

        if let Some(new_calibrating) = SIGNAL_IS_CALIBRATING.try_take() {
            is_calibrating = new_calibrating;
        }

        match Instant::now() {
            n => {
                match (n - last_loop_time).as_micros() {
                    a if a > 1999 => debug!("Loop took {} us", a),
                    _ => {}
                };
                last_loop_time = n;
            }
        };

        SIGNAL_STICK_STATE.signal(current_stick_state.clone());
    }
}
