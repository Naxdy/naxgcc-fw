/**
 *  Storage for controller configuration, including helper functions & types, as well as sane defaults.
 *  Also includes necessary logic for configuring the controller & calibrating the sticks.
 */
use core::{cmp::min, f32::consts::PI};

use defmt::{debug, error, info, warn, Format};
use embassy_futures::yield_now;
use embassy_rp::{
    flash::{Async, Flash, ERASE_SIZE},
    peripherals::FLASH,
};
use packed_struct::{derive::PackedStruct, PackedStruct};

use crate::{
    helpers::{PackedFloat, ToPackedFloatArray, ToRegularArray, XyValuePair},
    input::{
        read_ext_adc, Stick, StickAxis, StickState, FLOAT_ORIGIN, SPI_ACS_SHARED, SPI_CCS_SHARED,
        SPI_SHARED,
    },
    stick::{
        calc_stick_values, legalize_notches, AppliedCalibration, CleanedCalibrationPoints,
        LinearizedCalibration, NotchCalibration, NotchStatus, CALIBRATION_ORDER,
        NOTCH_ADJUSTMENT_ORDER, NO_OF_ADJ_NOTCHES, NO_OF_CALIBRATION_POINTS, NO_OF_NOTCHES,
    },
    ADDR_OFFSET, FLASH_SIZE,
};

use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex, ThreadModeRawMutex},
    pubsub::{PubSubBehavior, Subscriber},
    signal::Signal,
};
use embassy_time::{Duration, Ticker, Timer};

use crate::{gcc_hid::GcReport, input::CHANNEL_GCC_STATE};

/// Whether we are currently calibrating the sticks. Updates are dispatched when the status changes.
/// Initial status is assumed to be false.
pub static SIGNAL_IS_CALIBRATING: Signal<ThreadModeRawMutex, bool> = Signal::new();

/// Config change signalled to the stick task.
pub static SIGNAL_CONFIG_CHANGE: Signal<ThreadModeRawMutex, ControllerConfig> = Signal::new();

/// Signal used to override the stick state in order to display desired stick positions during calibration.
pub static SIGNAL_OVERRIDE_STICK_STATE: Signal<
    CriticalSectionRawMutex,
    Option<OverrideStickState>,
> = Signal::new();

/// Dispatched when we want to override the GCC state for a short amount of time.
pub static SIGNAL_OVERRIDE_GCC_STATE: Signal<CriticalSectionRawMutex, OverrideGcReportInstruction> =
    Signal::new();

/// Struct used for overriding the GCC state for a given amount of
/// time, useful for providing feedback to the user e.g. if we just entered
/// a certain mode.
#[derive(Default, Debug, Clone, Format)]
pub struct OverrideGcReportInstruction {
    pub report: GcReport,
    pub duration_ms: u64,
}

const CONFIG_MODE_ENTRY_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Start,
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Y,
];

const LSTICK_CALIBRATION_COMBO: [AwaitableButtons; 2] = [AwaitableButtons::A, AwaitableButtons::X];

const RSTICK_CALIBRATION_COMBO: [AwaitableButtons; 2] = [AwaitableButtons::A, AwaitableButtons::Y];

/// This doesn't need to be super fast, since it's only used
/// in config mode.
const BUTTON_POLL_INTERVAL_MILLIS: u64 = 20;

/// This needs to be incremented for ANY change to ControllerConfig
/// else we risk loading uninitialized memory.
pub const CONTROLLER_CONFIG_REVISION: u8 = 1;

pub const DEFAULT_NOTCH_STATUS: [NotchStatus; NO_OF_NOTCHES] = [
    NotchStatus::Cardinal,
    NotchStatus::TertActive,
    NotchStatus::Secondary,
    NotchStatus::TertActive,
    NotchStatus::Cardinal,
    NotchStatus::TertActive,
    NotchStatus::Secondary,
    NotchStatus::TertActive,
    NotchStatus::Cardinal,
    NotchStatus::TertActive,
    NotchStatus::Secondary,
    NotchStatus::TertActive,
    NotchStatus::Cardinal,
    NotchStatus::TertActive,
    NotchStatus::Secondary,
    NotchStatus::TertActive,
];

#[rustfmt::skip]
const DEFAULT_CAL_POINTS_X: [f32; NO_OF_CALIBRATION_POINTS] = [
    0.3010610568,0.3603937084,// right
	0.3010903951,0.3000194135,
	0.3005567843,0.3471911134,// up right
	0.3006904343,0.3009976295,
	0.3000800899,0.300985051,// up
	0.3001020858,0.300852804,
	0.3008746305,0.2548450139,// up left
	0.3001434092,0.3012600593,
	0.3011594091,0.2400535218,// left
	0.3014621077,0.3011248469,
	0.3010860944,0.2552106305,// down left
	0.3002197989,0.3001679513,
	0.3004438517,0.300486505,// down
	0.3002766984,0.3012828579,
	0.3014959877,0.346512936,// down right
	0.3013398149,0.3007809916
];

#[rustfmt::skip]
const DEFAULT_CAL_POINTS_Y: [f32; NO_OF_CALIBRATION_POINTS] = [
    0.300092277, 0.3003803475,// right
	0.3002205792,0.301004752,
	0.3001241394,0.3464200104,// up right
	0.3001331245,0.3011881186,
	0.3010685972,0.3606900641,// up
	0.3001520488,0.3010662947,
	0.3008837105,0.3461478452,// up left
	0.3011732026,0.3007367683,
	0.3011345742,0.3000566197,// left
	0.3006843288,0.3009673425,
	0.3011228978,0.2547579852,// down left
	0.3011177285,0.301264851,
	0.3002376991,0.2403885431,// down
	0.3006540818,0.3010588401,
	0.3011093054,0.2555000655,// down right
	0.3000802760,0.3008482317
];

pub const DEFAULT_ANGLES: [f32; NO_OF_NOTCHES] = [
    0.,
    PI / 8.0,
    PI * 2. / 8.,
    PI * 3. / 8.,
    PI * 4. / 8.,
    PI * 5. / 8.,
    PI * 6. / 8.,
    PI * 7. / 8.,
    PI * 8. / 8.,
    PI * 9. / 8.,
    PI * 10. / 8.,
    PI * 11. / 8.,
    PI * 12. / 8.,
    PI * 13. / 8.,
    PI * 14. / 8.,
    PI * 15. / 8.,
];

#[derive(Clone, Format)]
pub struct OverrideStickState {
    pub x: u8,
    pub y: u8,
    pub which_stick: Stick,
}

#[derive(Clone, Copy, Debug, Format)]
enum AwaitableButtons {
    A,
    B,
    X,
    Y,
    Up,
    Down,
    Left,
    Right,
    Start,
    L,
    R,
}

#[derive(Clone, Copy, Debug, Format)]
enum NotchAdjustmentType {
    Clockwise,
    CounterClockwise,
    Reset,
    None,
}

#[derive(Debug, Clone, Format, PackedStruct)]
#[packed_struct(endian = "msb")]
pub struct StickConfig {
    #[packed_field(size_bits = "8")]
    pub x_waveshaping: u8,
    #[packed_field(size_bits = "8")]
    pub y_waveshaping: u8,
    #[packed_field(size_bits = "8")]
    pub analog_scaler: u8,
    #[packed_field(size_bits = "8")]
    pub x_snapback: i8, // not used for CStick
    #[packed_field(size_bits = "8")]
    pub y_snapback: i8, // not used for CStick
    #[packed_field(size_bits = "8")]
    pub cardinal_snapping: i8, // not used for CStick
    #[packed_field(size_bits = "8")]
    pub x_smoothing: u8,
    #[packed_field(size_bits = "8")]
    pub y_smoothing: u8,
    #[packed_field(element_size_bytes = "4")]
    pub cal_points_x: [PackedFloat; 32],
    #[packed_field(element_size_bytes = "4")]
    pub cal_points_y: [PackedFloat; 32],
    #[packed_field(element_size_bytes = "4")]
    pub angles: [PackedFloat; 16],
}

impl Default for StickConfig {
    fn default() -> Self {
        Self {
            x_waveshaping: 0,
            y_waveshaping: 0,
            x_snapback: 4,
            y_snapback: 4,
            x_smoothing: 0,
            y_smoothing: 0,
            cardinal_snapping: 6,
            analog_scaler: 100,
            cal_points_x: *DEFAULT_CAL_POINTS_X.to_packed_float_array(),
            cal_points_y: *DEFAULT_CAL_POINTS_Y.to_packed_float_array(),
            angles: *DEFAULT_ANGLES.to_packed_float_array(),
        }
    }
}

#[derive(Debug, Clone, Format, PackedStruct)]
#[packed_struct(endian = "msb")]
pub struct ControllerConfig {
    #[packed_field(size_bits = "8")]
    pub config_revision: u8,
    /// Toggle for input consistency mode. If true, the controller
    /// will trick the Switch into updating the state every 8.33ms
    /// instead of every 8ms. The tradeoff is a slight increase in
    /// input lag.
    #[packed_field(size_bits = "8")]
    pub input_consistency_mode: bool,
    #[packed_field(size_bytes = "328")]
    pub astick_config: StickConfig,
    #[packed_field(size_bytes = "328")]
    pub cstick_config: StickConfig,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            config_revision: CONTROLLER_CONFIG_REVISION,
            input_consistency_mode: true,
            astick_config: StickConfig::default(),
            cstick_config: StickConfig::default(),
        }
    }
}

impl ControllerConfig {
    pub fn from_flash_memory(
        mut flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    ) -> Result<Self, embassy_rp::flash::Error> {
        let mut controller_config_packed: <ControllerConfig as packed_struct::PackedStruct>::ByteArray = [0u8; 658]; // ControllerConfig byte size
        flash.blocking_read(ADDR_OFFSET, &mut controller_config_packed)?;

        match ControllerConfig::unpack(&controller_config_packed).unwrap() {
            a if a.config_revision == CONTROLLER_CONFIG_REVISION => {
                info!("Controller config loaded from flash: {}", a);
                Ok(a)
            }
            a => {
                warn!("Outdated controller config detected ({:02X}), or controller config was never present, using default.", a.config_revision);
                let cfg = ControllerConfig::default();
                info!("Going to save default controller config.");
                cfg.write_to_flash(&mut flash)?;
                Ok(cfg)
            }
        }
    }

    pub fn write_to_flash(
        &self,
        flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    ) -> Result<(), embassy_rp::flash::Error> {
        info!("Writing controller config to flash.");
        flash.blocking_erase(ADDR_OFFSET, ADDR_OFFSET + ERASE_SIZE as u32)?;
        flash.blocking_write(ADDR_OFFSET, &self.pack().unwrap())?;
        Ok(())
    }
}

trait WaitForButtonPress {
    /// Wait for a single button press.
    async fn wait_for_button_press(&mut self, button_to_wait_for: &AwaitableButtons);

    /// Wait for a single button release.
    async fn wait_for_button_release(&mut self, button_to_wait_for: &AwaitableButtons);

    /// Wait for multiple buttons to be pressed simultaneously.
    async fn wait_for_simultaneous_button_presses<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    );

    /// Wait for a single button press of specified buttons, and return the button that was pressed.
    async fn wait_and_filter_button_press<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) -> AwaitableButtons;

    /// See if one of the buttons in buttons_to_look_out_for is pressed, and return the pressed button, otherwise None.
    fn filter_button_press_if_present<const N: usize>(
        &mut self,
        buttons_to_look_out_for: &[AwaitableButtons; N],
    ) -> Option<AwaitableButtons>;

    /// Wait for multiple possible button combinations to be pressed simultaneously, and return the index of the combination that was pressed.
    async fn wait_and_filter_simultaneous_button_presses<const N: usize, const M: usize>(
        &mut self,
        buttons_to_wait_for: &[[AwaitableButtons; N]; M],
    ) -> usize;
}

impl<'a, T: RawMutex, const I: usize, const J: usize, const K: usize> WaitForButtonPress
    for Subscriber<'a, T, GcReport, I, J, K>
{
    async fn wait_for_button_press(&mut self, button_to_wait_for: &AwaitableButtons) {
        loop {
            let report = self.next_message_pure().await;

            if is_awaitable_button_pressed(&report, button_to_wait_for) {
                break;
            }

            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_for_button_release(&mut self, button_to_wait_for: &AwaitableButtons) {
        loop {
            let report = self.next_message_pure().await;

            if !is_awaitable_button_pressed(&report, button_to_wait_for) {
                break;
            }

            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_for_simultaneous_button_presses<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) {
        loop {
            let report = self.next_message_pure().await;

            if buttons_to_wait_for
                .iter()
                .all(|button| is_awaitable_button_pressed(&report, button))
            {
                break;
            }

            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_and_filter_button_press<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) -> AwaitableButtons {
        loop {
            let report = self.next_message_pure().await;

            for button in buttons_to_wait_for {
                if is_awaitable_button_pressed(&report, button) {
                    return *button;
                }
            }

            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    async fn wait_and_filter_simultaneous_button_presses<const N: usize, const M: usize>(
        &mut self,
        buttons_to_wait_for: &[[AwaitableButtons; N]; M],
    ) -> usize {
        loop {
            let report = self.next_message_pure().await;

            for (i, buttons) in buttons_to_wait_for.iter().enumerate() {
                if buttons
                    .iter()
                    .all(|button| is_awaitable_button_pressed(&report, button))
                {
                    return i;
                }
            }

            Timer::after_millis(BUTTON_POLL_INTERVAL_MILLIS).await;
        }
    }

    fn filter_button_press_if_present<const N: usize>(
        &mut self,
        buttons_to_look_out_for: &[AwaitableButtons; N],
    ) -> Option<AwaitableButtons> {
        let report = self.try_next_message_pure();

        if let Some(report) = report {
            for button in buttons_to_look_out_for {
                if is_awaitable_button_pressed(&report, button) {
                    return Some(*button);
                }
            }
        }

        return None;
    }
}

fn is_awaitable_button_pressed(report: &GcReport, button_to_wait_for: &AwaitableButtons) -> bool {
    match button_to_wait_for {
        AwaitableButtons::A => report.buttons_1.button_a,
        AwaitableButtons::B => report.buttons_1.button_b,
        AwaitableButtons::X => report.buttons_1.button_x,
        AwaitableButtons::Y => report.buttons_1.button_y,
        AwaitableButtons::Up => report.buttons_1.dpad_up,
        AwaitableButtons::Down => report.buttons_1.dpad_down,
        AwaitableButtons::Left => report.buttons_1.dpad_left,
        AwaitableButtons::Right => report.buttons_1.dpad_right,
        AwaitableButtons::Start => report.buttons_2.button_start,
        AwaitableButtons::L => report.buttons_2.button_l || report.trigger_l > 10,
        AwaitableButtons::R => report.buttons_2.button_r || report.trigger_r > 10,
    }
}

#[derive(Debug, Format)]
struct StickCalibrationProcess<'a> {
    which_stick: Stick,
    calibration_step: u8,
    gcc_config: &'a mut ControllerConfig,
    cal_points: [XyValuePair<f32>; NO_OF_CALIBRATION_POINTS],
    applied_calibration: AppliedCalibration,
}

impl<'a> StickCalibrationProcess<'a> {
    pub fn new(gcc_config: &'a mut ControllerConfig, which_stick: Stick) -> Self {
        Self {
            which_stick,
            calibration_step: 0,
            gcc_config,
            cal_points: [XyValuePair::default(); NO_OF_CALIBRATION_POINTS],
            applied_calibration: AppliedCalibration::default(),
        }
    }

    fn adjust_notch(&mut self, notch_adjustment_type: NotchAdjustmentType) {
        let stick_config = match self.which_stick {
            Stick::ControlStick => &mut self.gcc_config.astick_config,
            Stick::CStick => &mut self.gcc_config.cstick_config,
        };

        let notch_idx =
            NOTCH_ADJUSTMENT_ORDER[self.calibration_step as usize - NO_OF_CALIBRATION_POINTS];

        if self.applied_calibration.cleaned_calibration.notch_status[notch_idx]
            == NotchStatus::TertInactive
        {}

        // assumes a tick rate of 1ms
        match notch_adjustment_type {
            NotchAdjustmentType::Clockwise => {
                stick_config.angles[notch_idx] -= 0.0075;
            }
            NotchAdjustmentType::CounterClockwise => {
                stick_config.angles[notch_idx] += 0.0075;
            }
            NotchAdjustmentType::Reset => {
                stick_config.angles[notch_idx] =
                    PackedFloat(self.applied_calibration.measured_notch_angles[notch_idx]);
            }
            NotchAdjustmentType::None => {}
        }

        match notch_adjustment_type {
            NotchAdjustmentType::Clockwise
            | NotchAdjustmentType::CounterClockwise
            | NotchAdjustmentType::Reset => {
                let cleaned_calibration_points =
                    CleanedCalibrationPoints::from_temp_calibration_points(
                        &self.cal_points.map(|e| e.x),
                        &self.cal_points.map(|e| e.y),
                        &self.applied_calibration.measured_notch_angles,
                    );

                let linearized_calibration =
                    LinearizedCalibration::from_calibration_points(&cleaned_calibration_points);

                self.applied_calibration.stick_params.fit_coeffs = XyValuePair {
                    x: linearized_calibration.fit_coeffs.x.map(|e| e as f32),
                    y: linearized_calibration.fit_coeffs.y.map(|e| e as f32),
                };

                let notch_calibration = NotchCalibration::from_cleaned_and_linearized_calibration(
                    &cleaned_calibration_points,
                    &linearized_calibration,
                );

                self.applied_calibration.stick_params.affine_coeffs =
                    notch_calibration.affine_coeffs;
                self.applied_calibration.stick_params.boundary_angles =
                    notch_calibration.boundary_angles;

                stick_config.angles = *legalize_notches(
                    self.calibration_step as usize,
                    &self.applied_calibration.measured_notch_angles,
                    &stick_config.angles.to_regular_array(),
                )
                .to_packed_float_array();

                SIGNAL_CONFIG_CHANGE.signal(self.gcc_config.clone());
            }
            NotchAdjustmentType::None => {}
        }
    }

    async fn calibration_advance(&mut self) -> bool {
        info!(
            "Running calibration advance on stick {} at step {}",
            self.which_stick, self.calibration_step
        );

        let stick_config = match self.which_stick {
            Stick::ControlStick => &mut self.gcc_config.astick_config,
            Stick::CStick => &mut self.gcc_config.cstick_config,
        };
        let mut spi_unlocked = SPI_SHARED.lock().await;
        let mut spi_acs_unlocked = SPI_ACS_SHARED.lock().await;
        let mut spi_ccs_unlocked = SPI_CCS_SHARED.lock().await;

        let spi = spi_unlocked.as_mut().unwrap();
        let spi_acs = spi_acs_unlocked.as_mut().unwrap();
        let spi_ccs = spi_ccs_unlocked.as_mut().unwrap();

        if self.calibration_step < NO_OF_CALIBRATION_POINTS as u8 {
            let mut x: f32 = 0.;
            let mut y: f32 = 0.;

            for _ in 0..128 {
                x += read_ext_adc(self.which_stick, StickAxis::XAxis, spi, spi_acs, spi_ccs) as f32
                    / 4096.0;
                y += read_ext_adc(self.which_stick, StickAxis::YAxis, spi, spi_acs, spi_ccs) as f32
                    / 4096.0;
            }

            x /= 128.;
            y /= 128.;

            let idx = CALIBRATION_ORDER[self.calibration_step as usize];

            self.cal_points[idx] = XyValuePair { x, y };
        }

        self.calibration_step += 1;

        // TODO: phob does something related to undo here

        if self.calibration_step == NO_OF_CALIBRATION_POINTS as u8 {
            stick_config.angles = *legalize_notches(
                self.calibration_step as usize,
                &self.applied_calibration.measured_notch_angles,
                &self.applied_calibration.notch_angles,
            )
            .to_packed_float_array();

            self.applied_calibration = AppliedCalibration::from_points(
                &self.cal_points.map(|e| e.x),
                &self.cal_points.map(|e| e.y),
                &stick_config,
                self.which_stick,
            );
        }

        if self.calibration_step >= NO_OF_CALIBRATION_POINTS as u8 {
            let mut notch_idx = NOTCH_ADJUSTMENT_ORDER[min(
                self.calibration_step - NO_OF_CALIBRATION_POINTS as u8,
                NO_OF_ADJ_NOTCHES as u8 - 1,
            ) as usize];

            while self.applied_calibration.cleaned_calibration.notch_status[notch_idx]
                == NotchStatus::TertInactive
                && self.calibration_step < NO_OF_CALIBRATION_POINTS as u8 + NO_OF_ADJ_NOTCHES as u8
            {
                stick_config.angles = *legalize_notches(
                    self.calibration_step as usize,
                    &self.applied_calibration.measured_notch_angles,
                    &stick_config.angles.to_regular_array(),
                )
                .to_packed_float_array();

                self.calibration_step += 1;

                notch_idx = NOTCH_ADJUSTMENT_ORDER[min(
                    self.calibration_step - NO_OF_CALIBRATION_POINTS as u8,
                    NO_OF_ADJ_NOTCHES as u8 - 1,
                ) as usize];
            }
        }

        if self.calibration_step >= NO_OF_CALIBRATION_POINTS as u8 + NO_OF_ADJ_NOTCHES as u8 {
            stick_config.cal_points_x = self.cal_points.map(|p| p.x.into());
            stick_config.cal_points_y = self.cal_points.map(|p| p.y.into());

            SIGNAL_CONFIG_CHANGE.signal(self.gcc_config.clone());

            info!("Finished calibrating stick {}", self.which_stick);

            return true;
        }

        return false;
    }

    pub async fn calibrate_stick(&mut self) {
        info!("Beginning stick calibration for {}", self.which_stick);

        let mut gcc_subscriber = CHANNEL_GCC_STATE.subscriber().unwrap();
        SIGNAL_IS_CALIBRATING.signal(true);

        while {
            if self.calibration_step < NO_OF_CALIBRATION_POINTS as u8 {
                // Calibration phase

                let (x, y) = get_stick_display_coords(self.calibration_step as usize);
                debug!(
                    "Raw display coords for step {}: {}, {}",
                    self.calibration_step, x, y
                );

                SIGNAL_OVERRIDE_STICK_STATE.signal(Some(OverrideStickState {
                    x: x as u8,
                    y: y as u8,
                    which_stick: match self.which_stick {
                        Stick::ControlStick => Stick::CStick,
                        Stick::CStick => Stick::ControlStick,
                    },
                }));

                gcc_subscriber
                    .wait_for_button_release(&AwaitableButtons::A)
                    .await;

                // Prevent accidental double presses
                Timer::after_millis(100).await;

                gcc_subscriber
                    .wait_for_button_press(&AwaitableButtons::A)
                    .await;
            } else {
                // Notch adjustment phase

                gcc_subscriber
                    .wait_for_button_release(&AwaitableButtons::A)
                    .await;

                Timer::after_millis(100).await;

                let mut ticker = Ticker::every(Duration::from_millis(20));

                let notch_idx = NOTCH_ADJUSTMENT_ORDER
                    [self.calibration_step as usize - NO_OF_CALIBRATION_POINTS];

                let (init_x, init_y) =
                    calc_stick_values(self.applied_calibration.measured_notch_angles[notch_idx]);

                SIGNAL_OVERRIDE_STICK_STATE.signal(Some(OverrideStickState {
                    x: (init_x + FLOAT_ORIGIN) as u8,
                    y: (init_y + FLOAT_ORIGIN) as u8,
                    which_stick: match self.which_stick {
                        Stick::ControlStick => Stick::CStick,
                        Stick::CStick => Stick::ControlStick,
                    },
                }));

                'adjust: loop {
                    let btn_result = gcc_subscriber.filter_button_press_if_present(&[
                        AwaitableButtons::A,
                        AwaitableButtons::B,
                        AwaitableButtons::X,
                        AwaitableButtons::Y,
                    ]);

                    match btn_result {
                        Some(btn) => match btn {
                            AwaitableButtons::A => {
                                debug!("Btn A release pressed");
                                break 'adjust;
                            }
                            AwaitableButtons::B => self.adjust_notch(NotchAdjustmentType::Reset),
                            AwaitableButtons::X => {
                                self.adjust_notch(NotchAdjustmentType::Clockwise)
                            }
                            AwaitableButtons::Y => {
                                self.adjust_notch(NotchAdjustmentType::CounterClockwise)
                            }
                            _ => self.adjust_notch(NotchAdjustmentType::None),
                        },
                        None => self.adjust_notch(NotchAdjustmentType::None),
                    };

                    ticker.next().await;
                    yield_now().await;
                }
            };

            !self.calibration_advance().await
        } {}

        SIGNAL_IS_CALIBRATING.signal(false);
        SIGNAL_OVERRIDE_STICK_STATE.signal(None);
    }
}

fn get_stick_display_coords(current_step: usize) -> (f32, f32) {
    let idx = CALIBRATION_ORDER[current_step];
    if idx % 2 != 0 {
        let notch_idx = idx / 2;
        match calc_stick_values(DEFAULT_ANGLES[notch_idx]) {
            (x, y) => (x + FLOAT_ORIGIN, y + FLOAT_ORIGIN),
        }
    } else {
        (127.5, 127.5)
    }
}

async fn configuration_main_loop<
    'a,
    M: RawMutex,
    const C: usize,
    const S: usize,
    const P: usize,
>(
    current_config: &ControllerConfig,
    mut flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    gcc_subscriber: &mut Subscriber<'a, M, GcReport, C, S, P>,
) {
    let mut final_config = current_config.clone();
    let config_options = [LSTICK_CALIBRATION_COMBO, RSTICK_CALIBRATION_COMBO];

    'main: loop {
        match gcc_subscriber
            .wait_and_filter_simultaneous_button_presses(&config_options)
            .await
        {
            selection => match selection {
                0 => {
                    StickCalibrationProcess::new(&mut final_config, Stick::ControlStick)
                        .calibrate_stick()
                        .await;
                }
                1 => {
                    StickCalibrationProcess::new(&mut final_config, Stick::CStick)
                        .calibrate_stick()
                        .await;
                }
                s => {
                    error!("Invalid selection in config loop: {}", s);
                    continue;
                }
            },
        };

        final_config.write_to_flash(&mut flash).unwrap();

        break 'main;
    }

    info!("Exiting config main loop.");
}

#[embassy_executor::task]
pub async fn config_task(
    current_config: ControllerConfig,
    mut flash: Flash<'static, FLASH, Async, FLASH_SIZE>,
) {
    let mut gcc_subscriber = CHANNEL_GCC_STATE.subscriber().unwrap();

    info!("Config task is running.");

    loop {
        gcc_subscriber
            .wait_for_simultaneous_button_presses(&CONFIG_MODE_ENTRY_COMBO)
            .await;

        info!("Entering config mode.");

        SIGNAL_OVERRIDE_GCC_STATE.signal(OverrideGcReportInstruction {
            report: match GcReport::default() {
                mut a => {
                    a.trigger_r = 255;
                    a.trigger_l = 255;
                    a.buttons_2.button_l = true;
                    a.buttons_2.button_r = true;
                    a.buttons_1.button_x = true;
                    a.buttons_1.button_y = true;
                    a.buttons_1.button_a = true;
                    a.stick_x = 127;
                    a.stick_y = 127;
                    a.cstick_x = 127;
                    a.cstick_y = 127;
                    a
                }
            },
            duration_ms: 1000,
        });

        // Wait for the user to release the buttons
        Timer::after_millis(1500).await;

        configuration_main_loop(&current_config, &mut flash, &mut gcc_subscriber).await;

        info!("Exiting config mode.");
    }
}
