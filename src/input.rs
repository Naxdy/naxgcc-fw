use defmt::{debug, info, trace, Format};
use embassy_futures::yield_now;
use embassy_rp::{
    gpio::{Input, Output},
    peripherals::SPI0,
    spi::{Blocking, Spi},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
    pubsub::PubSubChannel,
    signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use libm::{fmaxf, fminf};

use crate::{
    config::{
        ControllerConfig, ControllerMode, InputConsistencyMode, OverrideGcReportInstruction,
        OverrideStickState, SIGNAL_CONFIG_CHANGE, SIGNAL_IS_CALIBRATING,
        SIGNAL_OVERRIDE_CONTROLLER_STATE, SIGNAL_OVERRIDE_STICK_STATE,
    },
    filter::{run_waveshaping, FilterGains, KalmanState, WaveshapingValues, FILTER_GAINS},
    helpers::XyValuePair,
    hid::gcc::GcState,
    input_filter::{DummyFilter, InputFilter},
    stick::{linearize, notch_remap, StickParams},
    usb_comms::{MUTEX_CONTROLLER_MODE, MUTEX_INPUT_CONSISTENCY_MODE},
};

/// Used to send the button state to the usb task and the calibration task
pub static CHANNEL_CONTROLLER_STATE: PubSubChannel<
    CriticalSectionRawMutex,
    ControllerState,
    1,
    4,
    1,
> = PubSubChannel::new();

/// Used to send the stick state from the stick task to the main input task
static SIGNAL_STICK_STATE: Signal<CriticalSectionRawMutex, StickState> = Signal::new();

pub static SPI_SHARED: Mutex<ThreadModeRawMutex, Option<Spi<'static, SPI0, Blocking>>> =
    Mutex::new(None);
pub static SPI_ACS_SHARED: Mutex<ThreadModeRawMutex, Option<Output<'static>>> = Mutex::new(None);
pub static SPI_CCS_SHARED: Mutex<ThreadModeRawMutex, Option<Output<'static>>> = Mutex::new(None);

const STICK_HYST_VAL: f32 = 0.3;
pub const FLOAT_ORIGIN: f32 = 127.5;

#[derive(Clone, Copy, Debug, Format, PartialEq, Eq)]
pub struct StickState {
    pub ax: u8,
    pub ay: u8,
    pub cx: u8,
    pub cy: u8,
}

impl Default for StickState {
    fn default() -> Self {
        Self {
            ax: 127,
            ay: 127,
            cx: 127,
            cy: 127,
        }
    }
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

#[derive(Clone, Copy, Debug, Default, Format, PartialEq, Eq)]
pub struct ControllerState {
    pub button_a: bool,
    pub button_b: bool,
    pub button_x: bool,
    pub button_y: bool,
    pub trigger_zr: bool,
    pub trigger_zl: bool,
    pub trigger_l: bool,
    pub trigger_r: bool,
    pub button_start: bool,
    pub dpad_up: bool,
    pub dpad_down: bool,
    pub dpad_left: bool,
    pub dpad_right: bool,
    pub stick_state: StickState,
}

/// This is only implemented for backwards-compatibility purposes
impl From<GcState> for ControllerState {
    fn from(value: GcState) -> Self {
        Self {
            button_a: value.buttons_1.button_a,
            button_b: value.buttons_1.button_b,
            button_x: value.buttons_1.button_x,
            button_y: value.buttons_1.button_y,
            trigger_zr: value.buttons_2.button_z,
            trigger_zl: false,
            trigger_l: value.trigger_l > 170,
            trigger_r: value.trigger_r > 170,
            button_start: value.buttons_2.button_start,
            dpad_up: value.buttons_1.dpad_up,
            dpad_down: value.buttons_1.dpad_down,
            dpad_left: value.buttons_1.dpad_left,
            dpad_right: value.buttons_1.dpad_right,
            stick_state: StickState {
                ax: value.stick_x,
                ay: value.stick_y,
                cx: value.stick_x,
                cy: value.stick_y,
            },
        }
    }
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

#[inline(never)]
#[link_section = ".time_critical.read_ext_adc"]
pub fn read_ext_adc<'a, I: embassy_rp::spi::Instance, M: embassy_rp::spi::Mode>(
    which_stick: Stick,
    which_axis: StickAxis,
    spi: &mut Spi<'a, I, M>,
    spi_acs: &mut Output<'a>,
    spi_ccs: &mut Output<'a>,
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

    temp_value
}

/// Gets the average stick state over a 1ms interval in a non-blocking fashion.
/// Will wait until end_time is reached before continuing after reading the ADCs.
#[allow(clippy::too_many_arguments)]
#[inline(never)]
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

    let end_time = Instant::now() + Duration::from_micros(200); // this seems kinda magic, and it is, but

    let mut spi_unlocked = SPI_SHARED.lock().await;
    let mut spi_acs_unlocked = SPI_ACS_SHARED.lock().await;
    let mut spi_ccs_unlocked = SPI_CCS_SHARED.lock().await;

    let spi = spi_unlocked.as_mut().unwrap();
    let spi_acs = spi_acs_unlocked.as_mut().unwrap();
    let spi_ccs = spi_ccs_unlocked.as_mut().unwrap();

    let mut done = false;

    while !done {
        let loop_start = Instant::now();

        adc_count += 1;
        ax_sum += read_ext_adc(Stick::ControlStick, StickAxis::XAxis, spi, spi_acs, spi_ccs) as u32;
        ay_sum += read_ext_adc(Stick::ControlStick, StickAxis::YAxis, spi, spi_acs, spi_ccs) as u32;
        cx_sum += read_ext_adc(Stick::CStick, StickAxis::XAxis, spi, spi_acs, spi_ccs) as u32;
        cy_sum += read_ext_adc(Stick::CStick, StickAxis::YAxis, spi, spi_acs, spi_ccs) as u32;

        let loop_end = Instant::now();

        done = loop_end >= end_time - (loop_end - loop_start);
    }

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

    trace!("Raw CSTICK: {:?}", raw_cstick);

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
        kalman_state.run_kalman(x_z, y_z, &controller_config.astick_config, filter_gains);

    let shaped_astick = {
        let (x, y) = run_waveshaping(
            x_pos_filt,
            y_pos_filt,
            controller_config.astick_config.x_waveshaping,
            controller_config.astick_config.y_waveshaping,
            controlstick_waveshaping_values,
            filter_gains,
        );
        XyValuePair { x, y }
    };

    trace!("Shaped Controlstick: {}", shaped_astick);

    let pos_x: f32 = filter_gains.smoothing.x * shaped_astick.x
        + (1.0 - filter_gains.smoothing.x) * old_stick_pos.x;
    let pos_y = filter_gains.smoothing.y * shaped_astick.y
        + (1.0 - filter_gains.smoothing.y) * old_stick_pos.y;
    old_stick_pos.x = pos_x;
    old_stick_pos.y = pos_y;

    let shaped_cstick = {
        let (x, y) = run_waveshaping(
            pos_cx,
            pos_cy,
            controller_config.cstick_config.x_waveshaping,
            controller_config.cstick_config.y_waveshaping,
            cstick_waveshaping_values,
            filter_gains,
        );
        XyValuePair { x, y }
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

    trace!("Cstick position: {}, {}", pos_cx, pos_cy);

    let mut remapped = {
        let (x, y) = notch_remap(
            pos_x,
            pos_y,
            controlstick_params,
            &controller_config.astick_config,
            is_calibrating,
        );
        XyValuePair { x, y }
    };
    let mut remapped_c = {
        let (x, y) = notch_remap(
            pos_cx_filt,
            pos_cy_filt,
            cstick_params,
            &controller_config.cstick_config,
            is_calibrating,
        );
        XyValuePair { x, y }
    };
    let remapped_unfiltered = {
        let (x, y) = notch_remap(
            raw_stick_values.a_linearized.x,
            raw_stick_values.a_linearized.y,
            controlstick_params,
            &controller_config.astick_config,
            is_calibrating,
        );
        XyValuePair { x, y }
    };
    let remapped_c_unfiltered = {
        let (x, y) = notch_remap(
            raw_stick_values.c_linearized.x,
            raw_stick_values.c_linearized.y,
            cstick_params,
            &controller_config.cstick_config,
            is_calibrating,
        );
        XyValuePair { x, y }
    };

    trace!(
        "Remapped Control Stick: {}; C stick: {}",
        remapped,
        remapped_c
    );

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

    let mut out_stick_state = *current_stick_state;

    let diff_x = (remapped.x + FLOAT_ORIGIN) - current_stick_state.ax as f32;
    if !(-STICK_HYST_VAL..=(1.0 + STICK_HYST_VAL)).contains(&diff_x) {
        out_stick_state.ax = (remapped.x + FLOAT_ORIGIN) as u8;
    }
    let diff_y = (remapped.y + FLOAT_ORIGIN) - current_stick_state.ay as f32;
    if !(-STICK_HYST_VAL..=(1.0 + STICK_HYST_VAL)).contains(&diff_y) {
        out_stick_state.ay = (remapped.y + FLOAT_ORIGIN) as u8;
    }

    let diff_cx = (remapped_c.x + FLOAT_ORIGIN) - current_stick_state.cx as f32;
    if !(-STICK_HYST_VAL..=(1.0 + STICK_HYST_VAL)).contains(&diff_cx) {
        out_stick_state.cx = (remapped_c.x + FLOAT_ORIGIN) as u8;
    }
    let diff_cy = (remapped_c.y + FLOAT_ORIGIN) - current_stick_state.cy as f32;
    if !(-STICK_HYST_VAL..=(1.0 + STICK_HYST_VAL)).contains(&diff_cy) {
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

#[allow(clippy::too_many_arguments)]
fn update_button_states(
    controller_state: &mut ControllerState,
    btn_a: &Input<'_>,
    btn_b: &Input<'_>,
    btn_x: &Input<'_>,
    btn_y: &Input<'_>,
    btn_start: &Input<'_>,
    btn_l: &Input<'_>,
    btn_r: &Input<'_>,
    btn_zr: &Input<'_>,
    btn_zl: &Input<'_>,
    btn_dleft: &Input<'_>,
    btn_dright: &Input<'_>,
    btn_dup: &Input<'_>,
    btn_ddown: &Input<'_>,
) {
    controller_state.button_a = btn_a.is_low();
    controller_state.button_b = btn_b.is_low();
    controller_state.button_x = btn_x.is_low();
    controller_state.button_y = btn_y.is_low();
    controller_state.trigger_zr = btn_zr.is_low();
    controller_state.trigger_zl = btn_zl.is_low();
    controller_state.button_start = btn_start.is_low();
    controller_state.trigger_l = btn_l.is_low();
    controller_state.trigger_r = btn_r.is_low();
    controller_state.dpad_left = btn_dleft.is_low();
    controller_state.dpad_right = btn_dright.is_low();
    controller_state.dpad_up = btn_dup.is_low();
    controller_state.dpad_down = btn_ddown.is_low();
}

#[embassy_executor::task]
pub async fn input_integrity_benchmark() {
    loop {
        SIGNAL_OVERRIDE_CONTROLLER_STATE.signal(OverrideGcReportInstruction {
            report: {
                ControllerState {
                    dpad_up: true,
                    ..Default::default()
                }
            },
            duration_ms: 100,
        });

        Timer::after_millis(200).await;
    }
}

/// Task responsible for updating the button states.
/// Publishes the result to CHANNEL_GCC_STATE.
#[allow(clippy::too_many_arguments)]
#[embassy_executor::task]
pub async fn update_button_state_task(
    btn_zr: Input<'static>,
    btn_zl: Input<'static>,
    btn_a: Input<'static>,
    btn_b: Input<'static>,
    btn_dright: Input<'static>,
    btn_dup: Input<'static>,
    btn_ddown: Input<'static>,
    btn_dleft: Input<'static>,
    btn_l: Input<'static>,
    btn_r: Input<'static>,
    btn_x: Input<'static>,
    btn_y: Input<'static>,
    btn_start: Input<'static>,
) {
    // upon loop entry, we check for the reset combo once
    if btn_a.is_low() && btn_x.is_low() && btn_y.is_low() {
        info!("Detected reset button press, booting into flash.");
        embassy_rp::rom_data::reset_to_usb_boot(0, 0);

        #[allow(clippy::empty_loop)]
        loop {}
    }

    {
        let mut m = MUTEX_CONTROLLER_MODE.lock().await;
        *m = if btn_start.is_low() {
            Some(ControllerMode::Procon)
        } else if btn_x.is_low() {
            Some(ControllerMode::XInput)
        } else {
            Some(ControllerMode::GcAdapter)
        };
    }

    let input_consistency_mode = {
        while MUTEX_INPUT_CONSISTENCY_MODE.lock().await.is_none() {
            Timer::after(Duration::from_millis(100)).await;
        }
        MUTEX_INPUT_CONSISTENCY_MODE.lock().await.unwrap()
    };

    let mut previous_state = ControllerState::default();

    let mut controller_state = ControllerState::default();

    let gcc_publisher = CHANNEL_CONTROLLER_STATE.publisher().unwrap();

    let mut override_stick_state: Option<OverrideStickState> = None;

    // replace this with the input filter of your choice, if you so desire.
    let mut input_filter = DummyFilter;

    let mut initializing = true;

    let init_time = Instant::now();

    loop {
        update_button_states(
            &mut controller_state,
            &btn_a,
            &btn_b,
            &btn_x,
            &btn_y,
            &btn_start,
            &btn_l,
            &btn_r,
            &btn_zr,
            &btn_zl,
            &btn_dleft,
            &btn_dright,
            &btn_dup,
            &btn_ddown,
        );

        // not every loop pass is going to update the stick state
        if let Some(stick_state) = SIGNAL_STICK_STATE.try_take() {
            controller_state.stick_state = stick_state
        }

        if let Some(override_stick_state_opt) = SIGNAL_OVERRIDE_STICK_STATE.try_take() {
            trace!("Overridden stick state: {:?}", override_stick_state_opt);
            override_stick_state = override_stick_state_opt;
        }

        // check for a gcc state override (usually coming from the config task)
        if let Some(override_gcc_state) = SIGNAL_OVERRIDE_CONTROLLER_STATE.try_take() {
            trace!("Overridden gcc state: {:?}", override_gcc_state.report);
            let end_time = Instant::now() + Duration::from_millis(override_gcc_state.duration_ms);
            while Instant::now() < end_time {
                if input_consistency_mode == InputConsistencyMode::SuperHack {
                    if override_gcc_state.report != previous_state {
                        gcc_publisher.publish_immediate(override_gcc_state.report);
                        previous_state = override_gcc_state.report;
                    }
                } else {
                    gcc_publisher.publish_immediate(override_gcc_state.report);
                }

                yield_now().await;
            }
        };

        if let Some(override_state) = &override_stick_state {
            let mut overriden_gcc_state = controller_state;
            match override_state.which_stick {
                Stick::ControlStick => {
                    overriden_gcc_state.stick_state.ax = override_state.x;
                    overriden_gcc_state.stick_state.ay = override_state.y;
                }
                Stick::CStick => {
                    overriden_gcc_state.stick_state.cx = override_state.x;
                    overriden_gcc_state.stick_state.cy = override_state.y;
                }
            }
            gcc_publisher.publish_immediate(overriden_gcc_state);
        } else {
            input_filter.apply_filter(&mut controller_state);
            if input_consistency_mode == InputConsistencyMode::SuperHack {
                // transmit state always for the first 5 seconds to give the console time to initialize the controller
                if initializing && Instant::now().duration_since(init_time) > Duration::from_secs(5)
                {
                    initializing = false;
                }

                if controller_state != previous_state || initializing {
                    gcc_publisher.publish_immediate(controller_state);
                    previous_state = controller_state;
                }
            } else {
                gcc_publisher.publish_immediate(controller_state);
            }
        }

        // give other tasks a chance to do something
        yield_now().await;
    }
}

/// Task responsible for updating the stick states.
/// Publishes the result to STICK_SIGNAL.
///
/// Has to run on core0 because it makes use of SPI0.
#[embassy_executor::task]
#[inline(never)]
#[link_section = ".time_critical.update_stick_states_task"]
pub async fn update_stick_states_task(
    spi: Spi<'static, SPI0, embassy_rp::spi::Blocking>,
    spi_acs: Output<'static>,
    spi_ccs: Output<'static>,
) {
    // let some time pass before accepting stick inputs
    // to ensure sticks are properly zeroed
    Timer::after_secs(2).await;

    *SPI_SHARED.lock().await = Some(spi);
    *SPI_ACS_SHARED.lock().await = Some(spi_acs);
    *SPI_CCS_SHARED.lock().await = Some(spi_ccs);

    let mut controller_config = SIGNAL_CONFIG_CHANGE.wait().await;

    let mut controlstick_params = StickParams::from_stick_config(&controller_config.astick_config);
    let mut cstick_params = StickParams::from_stick_config(&controller_config.cstick_config);
    let mut filter_gains = FILTER_GAINS.get_normalized_gains(&controller_config);

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

    let mut is_calibrating = false;

    let mut ticker = Ticker::every(Duration::from_hz(1000));

    loop {
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

        if let Some(new_calibrating) = SIGNAL_IS_CALIBRATING.try_take() {
            is_calibrating = new_calibrating;
            if !is_calibrating {
                debug!("Reset the ticker.");
                ticker.reset();
            }
        }

        {
            let n = Instant::now();

            match (n - last_loop_time).as_micros() {
                a if a > 800 => debug!("Loop took {} us", a),
                _ => {}
            };
            last_loop_time = n;
        };

        SIGNAL_STICK_STATE.signal(current_stick_state);

        yield_now().await;
        ticker.next().await;

        if let Some(new_config) = SIGNAL_CONFIG_CHANGE.try_take() {
            controller_config = new_config;
            controlstick_params = StickParams::from_stick_config(&controller_config.astick_config);
            cstick_params = StickParams::from_stick_config(&controller_config.cstick_config);
            filter_gains = FILTER_GAINS.get_normalized_gains(&controller_config);

            info!("Controlstick params: {:?}", controlstick_params);
            info!("CStick params: {:?}", cstick_params);
        }
    }
}
