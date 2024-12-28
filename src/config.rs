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
use packed_struct::{
    derive::{PackedStruct, PrimitiveEnum_u8},
    PackedStruct,
};

use crate::{
    helpers::{PackedFloat, ToPackedFloatArray, ToRegularArray, XyValuePair},
    hid::gcc::{GcButtons1, GcButtons2, GcState},
    input::{
        read_ext_adc, ControllerState, Stick, StickAxis, FLOAT_ORIGIN, SPI_ACS_SHARED,
        SPI_CCS_SHARED, SPI_SHARED,
    },
    stick::{
        calc_stick_values, legalize_notches, AppliedCalibration, CleanedCalibrationPoints,
        LinearizedCalibration, NotchCalibration, NotchStatus, CALIBRATION_ORDER,
        NOTCH_ADJUSTMENT_ORDER, NO_OF_ADJ_NOTCHES, NO_OF_CALIBRATION_POINTS, NO_OF_NOTCHES,
    },
    usb_comms::{MUTEX_INPUT_CONSISTENCY_MODE, SIGNAL_CHANGE_RUMBLE_STRENGTH},
    ADDR_OFFSET, FLASH_SIZE,
};

use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex, ThreadModeRawMutex},
    pubsub::Subscriber,
    signal::Signal,
};
use embassy_time::Timer;

use crate::input::CHANNEL_CONTROLLER_STATE;

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
pub static SIGNAL_OVERRIDE_CONTROLLER_STATE: Signal<
    CriticalSectionRawMutex,
    OverrideGcReportInstruction,
> = Signal::new();

/// Dispatched when we want to enter config mode, sent from core1 so config mode
/// doesn't need to watch for the entire button combo every time.
static SIGNAL_CONFIG_MODE_STATUS_CHANGE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

const ABS_MAX_SNAPBACK: i8 = 10;

const MAX_WAVESHAPING: i8 = 15;

const MAX_SMOOTHING: i8 = 9;

const MAX_RUMBLE_STRENGTH: i8 = 11;

const MAX_CARDINAL_SNAP: i8 = 6;

const MIN_ANALOG_SCALER: u8 = 82;

const MAX_ANALOG_SCALER: u8 = 125;

/// Struct used for overriding the GCC state for a given amount of
/// time, useful for providing feedback to the user e.g. if we just entered
/// a certain mode.
#[derive(Default, Debug, Clone, Format)]
pub struct OverrideGcReportInstruction {
    pub report: ControllerState,
    pub duration_ms: u64,
}

const CONFIG_MODE_ENTRY_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Start,
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Y,
];

const LSTICK_CALIBRATION_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Y,
    AwaitableButtons::L,
];

const RSTICK_CALIBRATION_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::Y,
    AwaitableButtons::X,
    AwaitableButtons::R,
];

const LSTICK_SNAPBACK_X_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
];

const LSTICK_SNAPBACK_Y_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::NotZ,
    AwaitableButtons::Up,
    AwaitableButtons::Y,
];

const LSTICK_SNAPBACK_X_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
];

const LSTICK_SNAPBACK_Y_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::Y,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
];

const RSTICK_SNAPBACK_X_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Up,
    AwaitableButtons::Z,
];

const RSTICK_SNAPBACK_Y_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::Y,
    AwaitableButtons::Up,
    AwaitableButtons::Z,
];

const RSTICK_SNAPBACK_X_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Down,
    AwaitableButtons::Z,
];

const RSTICK_SNAPBACK_Y_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::Y,
    AwaitableButtons::Down,
    AwaitableButtons::Z,
];

const LSTICK_WAVESHAPING_X_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::X,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
    AwaitableButtons::L,
];

const LSTICK_WAVESHAPING_Y_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Y,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
    AwaitableButtons::L,
];

const LSTICK_WAVESHAPING_X_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::X,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
    AwaitableButtons::L,
];

const LSTICK_WAVESHAPING_Y_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Y,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
    AwaitableButtons::L,
];

const RSTICK_WAVESHAPING_X_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::X,
    AwaitableButtons::Up,
    AwaitableButtons::Z,
    AwaitableButtons::L,
];

const RSTICK_WAVESHAPING_Y_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Y,
    AwaitableButtons::Up,
    AwaitableButtons::Z,
    AwaitableButtons::L,
];

const RSTICK_WAVESHAPING_X_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::X,
    AwaitableButtons::Down,
    AwaitableButtons::Z,
    AwaitableButtons::L,
];

const RSTICK_WAVESHAPING_Y_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Y,
    AwaitableButtons::Down,
    AwaitableButtons::Z,
    AwaitableButtons::L,
];

const LSTICK_SMOOTH_X_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::X,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
];

const LSTICK_SMOOTH_Y_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::Y,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
];

const LSTICK_SMOOTH_X_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::X,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
];

const LSTICK_SMOOTH_Y_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::Y,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
];

const RSTICK_SMOOTH_X_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Z,
    AwaitableButtons::Y,
    AwaitableButtons::Up,
    AwaitableButtons::R,
];

const RSTICK_SMOOTH_Y_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Z,
    AwaitableButtons::X,
    AwaitableButtons::Up,
    AwaitableButtons::R,
];

const RSTICK_SMOOTH_X_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Z,
    AwaitableButtons::Y,
    AwaitableButtons::Down,
    AwaitableButtons::R,
];

const RSTICK_SMOOTH_Y_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Z,
    AwaitableButtons::X,
    AwaitableButtons::Down,
    AwaitableButtons::R,
];

const LSTICK_CARDINALSNAP_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::A,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
];

const LSTICK_CARDINALSNAP_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::A,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
];

const RSTICK_CARDINALSNAP_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Z,
    AwaitableButtons::A,
    AwaitableButtons::Up,
    AwaitableButtons::R,
];

const RSTICK_CARDINALSNAP_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Z,
    AwaitableButtons::A,
    AwaitableButtons::Down,
    AwaitableButtons::R,
];

const LSTICK_SCALING_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::L,
    AwaitableButtons::A,
    AwaitableButtons::Up,
    AwaitableButtons::NotZ,
];

const LSTICK_SCALING_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::L,
    AwaitableButtons::A,
    AwaitableButtons::Down,
    AwaitableButtons::NotZ,
];

const RSTICK_SCALING_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::L,
    AwaitableButtons::A,
    AwaitableButtons::Up,
    AwaitableButtons::Z,
];

const RSTICK_SCALING_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::L,
    AwaitableButtons::A,
    AwaitableButtons::Down,
    AwaitableButtons::Z,
];

const RUMBLE_STRENGTH_INCREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::B,
    AwaitableButtons::A,
    AwaitableButtons::Up,
    AwaitableButtons::Wildcard,
];

const RUMBLE_STRENGTH_DECREASE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::B,
    AwaitableButtons::A,
    AwaitableButtons::Down,
    AwaitableButtons::Wildcard,
];

const INPUT_CONSISTENCY_TOGGLE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::Z,
    AwaitableButtons::Start,
    AwaitableButtons::Wildcard,
];

const DISPLAY_WAVESHAPING_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::L,
    AwaitableButtons::Start,
    AwaitableButtons::Wildcard,
    AwaitableButtons::Wildcard,
];

const DISPLAY_SMOOTHING_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::R,
    AwaitableButtons::Start,
    AwaitableButtons::Wildcard,
    AwaitableButtons::Wildcard,
];

const DISPLAY_SNAPBACK_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::Up,
    AwaitableButtons::Start,
    AwaitableButtons::Wildcard,
    AwaitableButtons::Wildcard,
];

const EXIT_CONFIG_MODE_COMBO: [AwaitableButtons; 4] = [
    AwaitableButtons::A,
    AwaitableButtons::X,
    AwaitableButtons::Y,
    AwaitableButtons::Start,
];

/// This doesn't need to be super fast, since it's only used
/// in config mode.
const BUTTON_POLL_INTERVAL_MILLIS: u64 = 20;

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
    0.5279579,  0.37779236, // right
    0.5280895,  0.52808,
    0.5280571,  0.41205978, // up right
    0.5280628,  0.5280857,
    0.528059,   0.5217171, // up
    0.5281048,  0.52808,
    0.52776146, 0.6376648, // up left
    0.5280838,  0.5280762,
    0.5277729,  0.6680012, // left
    0.5280876,  0.5280762,
    0.5289936,  0.64232254, // down left
    0.52809143, 0.52807236,
    0.528574,   0.52160454, // down
    0.52809143, 0.5281086,
    0.5288601,  0.4079399, // down right
    0.5281067,  0.52807426
];
// cal_points_y: [PackedFloat(0.50214195), PackedFloat(0.51600456), PackedFloat(0.5022125), PackedFloat(0.5021801), PackedFloat(0.5019417), PackedFloat(0.39642334), PackedFloat(0.5022106), PackedFloat(0.5021858), PackedFloat(0.5018635), PackedFloat(0.3603382), PackedFloat(0.5022011), PackedFloat(0.5022621), PackedFloat(0.5016308), PackedFloat(0.4004345), PackedFloat(0.5022621), PackedFloat(0.5022583), PackedFloat(0.501112), PackedFloat(0.51377296), PackedFloat(0.5022392), PackedFloat(0.5022774), PackedFloat(0.5010319), PackedFloat(0.62000275), PackedFloat(0.50230026), PackedFloat(0.5023346), PackedFloat(0.50206566), PackedFloat(0.6559849), PackedFloat(0.50224113), PackedFloat(0.50229454), PackedFloat(0.50209045), PackedFloat(0.6217842), PackedFloat(0.50227165), PackedFloat(0.50227356)]
#[rustfmt::skip]
const DEFAULT_CAL_POINTS_Y: [f32; NO_OF_CALIBRATION_POINTS] = [
    0.50214195, 0.51600456, // right
    0.5022125,  0.5021801,
    0.5019417,  0.39642334, // up right
    0.5022106,  0.5021858,
    0.5018635,  0.3603382, // up
    0.5022011,  0.5022621,
    0.5016308,  0.4004345, // up left
    0.5022621,  0.5022583,
    0.501112,   0.51377296, // left
    0.5022392,  0.5022774,
    0.5010319,  0.62000275, // down left
    0.50230026, 0.5023346,
    0.50206566, 0.6559849, // down
    0.50224113, 0.50229454,
    0.50209045, 0.6217842, // down right
    0.50227165, 0.50227356
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

/// Enum button representation mainly used in the calibration process,
/// in conjunction with `is_awaitable_button_pressed`
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Format, PartialEq, Eq)]
pub enum AwaitableButtons {
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
    Z,
    /// Special, because Z is used for cstick calibration.
    NotZ,
    /// Can be used for padding arrays to a fixed length.
    Wildcard,
    /// Can be used for disabling certain button combinations.
    Impossible,
}

#[derive(Clone, Copy, Debug, Format)]
enum NotchAdjustmentType {
    Clockwise,
    CounterClockwise,
    Reset,
    None,
}

/// This needs to be incremented for ANY change to ControllerConfig
/// else we risk loading uninitialized memory.
pub const CONTROLLER_CONFIG_REVISION: u8 = 1;

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
    pub cardinal_snapping: i8,
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

#[derive(Debug, Clone, Copy, Format, PrimitiveEnum_u8, PartialEq, Eq)]
pub enum InputConsistencyMode {
    /// Transmit inputs every 8ms, same as the original GCC adapter (and any other).
    Original = 0,
    /// Forcibly delay transmissions to be 8.33ms apart, to better align with the game's frame rate.
    ConsistencyHack = 1,
    /// Transmit inputs _at most_ every 8.33ms, but don't transmit anything at all if the controller state doesn't change.
    /// This has the potential to drastically improve latency in certain situations, such as when you are waiting to react
    /// to something your opponent does.
    /// The name is not meant to imply that this is a hack that is super, but rather that this is super hacky.
    SuperHack = 2,
    /// Transmit inputs every 1 ms, for use on PC or other devices that are not garbage.
    /// This is not recommended for use on the Switch!
    PC = 3,
}

/// Not saved, but set upon plugging in the controller.
#[derive(Debug, Clone, Copy, Format, PrimitiveEnum_u8, PartialEq, Eq)]
pub enum ControllerMode {
    /// Advertise itself as a GCC adapter with 1 controller (itself) connected.
    GcAdapter = 0,
    /// Pretend to be a Nintendo Switch Pro Controller connected via USB.
    Procon = 1,
    /// Act as an XInput device, and also advertise itself with 1000Hz polling capability.
    XInput = 2,
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
    #[packed_field(size_bits = "8", ty = "enum")]
    pub input_consistency_mode: InputConsistencyMode,
    #[packed_field(size_bits = "8")]
    pub rumble_strength: u8,
    #[packed_field(size_bytes = "328")]
    pub astick_config: StickConfig,
    #[packed_field(size_bytes = "328")]
    pub cstick_config: StickConfig,
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            config_revision: CONTROLLER_CONFIG_REVISION,
            input_consistency_mode: InputConsistencyMode::ConsistencyHack,
            astick_config: StickConfig::default(),
            rumble_strength: 9,
            cstick_config: StickConfig::default(),
        }
    }
}

impl ControllerConfig {
    pub fn from_flash_memory(
        flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    ) -> Result<Self, embassy_rp::flash::Error> {
        let mut controller_config_packed: <ControllerConfig as packed_struct::PackedStruct>::ByteArray = ControllerConfig::default().pack().unwrap();

        flash.blocking_read(ADDR_OFFSET, &mut controller_config_packed)?;

        match ControllerConfig::unpack(&controller_config_packed) {
            Ok(cfg) => match cfg {
                a if a.config_revision == CONTROLLER_CONFIG_REVISION => {
                    info!("Controller config loaded from flash: {}", a);
                    return Ok(a);
                }
                a => {
                    warn!("Outdated controller config detected ({:02X}), or controller config was never present, using default.", a.config_revision);
                }
            },
            Err(_) => {
                warn!("Controller config corrupted or flash uninitialized, using default.");
            }
        };

        let cfg = ControllerConfig::default();
        info!("Going to save default controller config.");
        cfg.write_to_flash(flash)?;
        Ok(cfg)
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

/// Trait for providing button presses, used in the calibration process.
trait ButtonPressProvider {
    /// Wait for a single button press.
    // TODO: remove allow once this is used somewhere
    #[allow(dead_code)]
    async fn wait_for_button_press(&mut self, button_to_wait_for: &AwaitableButtons);

    /// Wait for a single button release.
    async fn wait_for_button_release(&mut self, button_to_wait_for: &AwaitableButtons);

    /// Wait for multiple buttons to be pressed simultaneously. Non-exclusive.
    async fn wait_for_simultaneous_button_presses<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    );

    /// Wait for a single button press of specified buttons, and return the button that was pressed. Non-exclusive.
    async fn wait_and_filter_button_press<const N: usize>(
        &mut self,
        buttons_to_wait_for: &[AwaitableButtons; N],
    ) -> AwaitableButtons;

    /// See if one of the buttons in buttons_to_look_out_for is pressed, and return the pressed button, otherwise None. Non-exclusive.
    fn filter_button_press_if_present<const N: usize>(
        &mut self,
        buttons_to_look_out_for: &[AwaitableButtons; N],
    ) -> Option<AwaitableButtons>;

    /// Wait for multiple possible button combinations to be pressed simultaneously, and return the index of the combination that was pressed.
    ///
    /// Note that this is done non-exclusively, so if the pressed buttons match multiple combinations, the first matching index in the array will be returned.
    async fn wait_and_filter_simultaneous_button_presses<const N: usize, const M: usize>(
        &mut self,
        buttons_to_wait_for: &[[AwaitableButtons; N]; M],
    ) -> usize;
}

impl<'a, T: RawMutex, const I: usize, const J: usize, const K: usize> ButtonPressProvider
    for Subscriber<'a, T, ControllerState, I, J, K>
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

        None
    }
}

pub fn is_awaitable_button_pressed(
    report: &ControllerState,
    button_to_wait_for: &AwaitableButtons,
) -> bool {
    match button_to_wait_for {
        AwaitableButtons::A => report.button_a,
        AwaitableButtons::B => report.button_b,
        AwaitableButtons::X => report.button_x,
        AwaitableButtons::Y => report.button_y,
        AwaitableButtons::Up => report.dpad_up,
        AwaitableButtons::Down => report.dpad_down,
        AwaitableButtons::Left => report.dpad_left,
        AwaitableButtons::Right => report.dpad_right,
        AwaitableButtons::Start => report.button_start,
        AwaitableButtons::L => report.trigger_l,
        AwaitableButtons::R => report.trigger_r,
        AwaitableButtons::Z => report.trigger_zr,
        AwaitableButtons::Wildcard => true,
        AwaitableButtons::Impossible => false,
        AwaitableButtons::NotZ => !report.trigger_zr,
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
        {
            return;
        }

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
                    stick_config.angles.to_regular_array(),
                )
                .to_packed_float_array();

                SIGNAL_CONFIG_CHANGE.signal(self.gcc_config.clone());
            }
            NotchAdjustmentType::None => {}
        }
    }

    fn calibration_skip_measurement(&mut self) {
        self.calibration_step = NO_OF_CALIBRATION_POINTS as u8;

        let stick_config = match self.which_stick {
            Stick::ControlStick => &mut self.gcc_config.astick_config,
            Stick::CStick => &mut self.gcc_config.cstick_config,
        };

        for i in 0..NO_OF_CALIBRATION_POINTS {
            self.cal_points[i] = XyValuePair {
                x: *stick_config.cal_points_x[i],
                y: *stick_config.cal_points_y[i],
            };
        }

        self.applied_calibration = AppliedCalibration::from_points(
            stick_config.cal_points_x.to_regular_array(),
            stick_config.cal_points_y.to_regular_array(),
            stick_config,
        );
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
            self.applied_calibration = AppliedCalibration::from_points(
                &self.cal_points.map(|e| e.x),
                &self.cal_points.map(|e| e.y),
                stick_config,
            );

            stick_config.angles = *legalize_notches(
                self.calibration_step as usize,
                &self.applied_calibration.measured_notch_angles,
                &DEFAULT_ANGLES,
            )
            .to_packed_float_array();
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
                    stick_config.angles.to_regular_array(),
                )
                .to_packed_float_array();

                self.calibration_step += 1;

                notch_idx = NOTCH_ADJUSTMENT_ORDER[min(
                    self.calibration_step - NO_OF_CALIBRATION_POINTS as u8,
                    NO_OF_ADJ_NOTCHES as u8 - 1,
                ) as usize];
            }
        }

        stick_config.cal_points_x = self.cal_points.map(|p| p.x.into());
        stick_config.cal_points_y = self.cal_points.map(|p| p.y.into());

        if self.calibration_step >= NO_OF_CALIBRATION_POINTS as u8 + NO_OF_ADJ_NOTCHES as u8 {
            SIGNAL_CONFIG_CHANGE.signal(self.gcc_config.clone());

            info!("Finished calibrating stick {}", self.which_stick);

            return true;
        }

        if self.calibration_step == NO_OF_CALIBRATION_POINTS as u8 {
            SIGNAL_CONFIG_CHANGE.signal(self.gcc_config.clone());
        }

        false
    }

    pub async fn calibrate_stick(&mut self) {
        info!("Beginning stick calibration for {}", self.which_stick);

        let mut gcc_subscriber = CHANNEL_CONTROLLER_STATE.subscriber().unwrap();
        SIGNAL_IS_CALIBRATING.signal(true);

        let mut done = false;

        while !done {
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

                let btn_result = gcc_subscriber
                    .wait_and_filter_button_press(&[AwaitableButtons::A, AwaitableButtons::Start])
                    .await;

                if btn_result == AwaitableButtons::Start {
                    self.calibration_skip_measurement();
                    continue;
                }
            } else {
                // Notch adjustment phase

                gcc_subscriber
                    .wait_for_button_release(&AwaitableButtons::A)
                    .await;

                Timer::after_millis(100).await;

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

                    Timer::after_millis(1).await;
                    yield_now().await;
                }
            };

            done = self.calibration_advance().await;
        }

        SIGNAL_IS_CALIBRATING.signal(false);
        SIGNAL_OVERRIDE_STICK_STATE.signal(None);
    }
}

fn get_stick_display_coords(current_step: usize) -> (f32, f32) {
    let idx = CALIBRATION_ORDER[current_step];
    if idx % 2 != 0 {
        let notch_idx = idx / 2;
        let (x, y) = calc_stick_values(DEFAULT_ANGLES[notch_idx]);
        (x + FLOAT_ORIGIN, y + FLOAT_ORIGIN)
    } else {
        (127.5, 127.5)
    }
}

pub async fn override_gcc_state_and_wait(state: &OverrideGcReportInstruction) {
    SIGNAL_OVERRIDE_CONTROLLER_STATE.signal(state.clone());
    Timer::after_millis(state.duration_ms).await;
}

async fn configuration_main_loop<
    'a,
    M: RawMutex,
    const C: usize,
    const S: usize,
    const P: usize,
>(
    current_config: &ControllerConfig,
    flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    gcc_subscriber: &mut Subscriber<'a, M, ControllerState, C, S, P>,
) -> ControllerConfig {
    let mut final_config = current_config.clone();
    let config_options = [
        EXIT_CONFIG_MODE_COMBO,
        LSTICK_CALIBRATION_COMBO,
        RSTICK_CALIBRATION_COMBO,
        LSTICK_SNAPBACK_X_INCREASE_COMBO,
        LSTICK_SNAPBACK_Y_INCREASE_COMBO,
        LSTICK_SNAPBACK_X_DECREASE_COMBO,
        LSTICK_SNAPBACK_Y_DECREASE_COMBO,
        // NOTE: cstick snapback is currently
        // unused, but we have the combos here
        // anyway.
        RSTICK_SNAPBACK_X_INCREASE_COMBO,
        RSTICK_SNAPBACK_Y_INCREASE_COMBO,
        RSTICK_SNAPBACK_X_DECREASE_COMBO,
        RSTICK_SNAPBACK_Y_DECREASE_COMBO,
        LSTICK_WAVESHAPING_X_INCREASE_COMBO,
        LSTICK_WAVESHAPING_Y_INCREASE_COMBO,
        LSTICK_WAVESHAPING_X_DECREASE_COMBO,
        LSTICK_WAVESHAPING_Y_DECREASE_COMBO,
        RSTICK_WAVESHAPING_X_INCREASE_COMBO,
        RSTICK_WAVESHAPING_Y_INCREASE_COMBO,
        RSTICK_WAVESHAPING_X_DECREASE_COMBO,
        RSTICK_WAVESHAPING_Y_DECREASE_COMBO,
        LSTICK_SMOOTH_X_INCREASE_COMBO,
        LSTICK_SMOOTH_Y_INCREASE_COMBO,
        LSTICK_SMOOTH_X_DECREASE_COMBO,
        LSTICK_SMOOTH_Y_DECREASE_COMBO,
        RSTICK_SMOOTH_X_INCREASE_COMBO,
        RSTICK_SMOOTH_Y_INCREASE_COMBO,
        RSTICK_SMOOTH_X_DECREASE_COMBO,
        RSTICK_SMOOTH_Y_DECREASE_COMBO,
        LSTICK_CARDINALSNAP_INCREASE_COMBO,
        LSTICK_CARDINALSNAP_DECREASE_COMBO,
        RSTICK_CARDINALSNAP_INCREASE_COMBO,
        RSTICK_CARDINALSNAP_DECREASE_COMBO,
        LSTICK_SCALING_INCREASE_COMBO,
        LSTICK_SCALING_DECREASE_COMBO,
        RSTICK_SCALING_INCREASE_COMBO,
        RSTICK_SCALING_DECREASE_COMBO,
        RUMBLE_STRENGTH_INCREASE_COMBO,
        RUMBLE_STRENGTH_DECREASE_COMBO,
        INPUT_CONSISTENCY_TOGGLE_COMBO,
        DISPLAY_WAVESHAPING_COMBO,
        DISPLAY_SMOOTHING_COMBO,
        DISPLAY_SNAPBACK_COMBO,
    ];

    'main: loop {
        let selection = gcc_subscriber
            .wait_and_filter_simultaneous_button_presses(&config_options)
            .await;
        match selection {
            // exit
            0 => {
                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127,
                        stick_y: 127,
                        cstick_x: 127,
                        cstick_y: 127,
                    }
                    .into(),
                    duration_ms: 1000,
                })
                .await;

                break 'main;
            }
            // calibrate lstick
            1 => {
                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 255,
                        stick_y: 255,
                        cstick_x: 127,
                        cstick_y: 127,
                    }
                    .into(),
                    duration_ms: 1000,
                })
                .await;
                StickCalibrationProcess::new(&mut final_config, Stick::ControlStick)
                    .calibrate_stick()
                    .await;
            }
            // calibrate rstick
            2 => {
                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127,
                        stick_y: 127,
                        cstick_x: 255,
                        cstick_y: 255,
                    }
                    .into(),
                    duration_ms: 1000,
                })
                .await;
                StickCalibrationProcess::new(&mut final_config, Stick::CStick)
                    .calibrate_stick()
                    .await;
            }
            // snapback changes
            i if (3..=10).contains(&i) => {
                let stick = match i {
                    3..=6 => Stick::ControlStick,
                    7..=10 => Stick::CStick,
                    _ => unreachable!(),
                };

                let axis = match i {
                    3 | 7 | 5 | 9 => StickAxis::XAxis,
                    4 | 8 | 6 | 10 => StickAxis::YAxis,
                    _ => unreachable!(),
                };

                let stick_config = match stick {
                    Stick::ControlStick => &mut final_config.astick_config,
                    Stick::CStick => &mut final_config.cstick_config,
                };

                let to_adjust = match axis {
                    StickAxis::XAxis => &mut stick_config.x_snapback,
                    StickAxis::YAxis => &mut stick_config.y_snapback,
                };

                *to_adjust = (*to_adjust
                    + match i {
                        3 | 7 | 4 | 8 => 1,
                        5 | 9 | 6 | 10 => -1,
                        _ => unreachable!(),
                    })
                .clamp(-ABS_MAX_SNAPBACK, ABS_MAX_SNAPBACK);

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: (127
                            + match stick {
                                Stick::ControlStick => match axis {
                                    StickAxis::XAxis => *to_adjust,
                                    StickAxis::YAxis => 0,
                                },
                                Stick::CStick => 0,
                            }) as u8,
                        stick_y: (127
                            + match stick {
                                Stick::ControlStick => match axis {
                                    StickAxis::XAxis => 0,
                                    StickAxis::YAxis => *to_adjust,
                                },
                                Stick::CStick => 0,
                            }) as u8,
                        cstick_x: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => match axis {
                                    StickAxis::XAxis => *to_adjust,
                                    StickAxis::YAxis => 0,
                                },
                            }) as u8,
                        cstick_y: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => match axis {
                                    StickAxis::XAxis => 0,
                                    StickAxis::YAxis => *to_adjust,
                                },
                            }) as u8,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // waveshaping changes
            i if (11..=18).contains(&i) => {
                let stick = match i {
                    11..=14 => Stick::ControlStick,
                    15..=18 => Stick::CStick,
                    _ => unreachable!(),
                };

                let axis = match i {
                    11 | 15 | 13 | 17 => StickAxis::XAxis,
                    12 | 16 | 14 | 18 => StickAxis::YAxis,
                    _ => unreachable!(),
                };

                let stick_config = match stick {
                    Stick::ControlStick => &mut final_config.astick_config,
                    Stick::CStick => &mut final_config.cstick_config,
                };

                let to_adjust = match axis {
                    StickAxis::XAxis => &mut stick_config.x_waveshaping,
                    StickAxis::YAxis => &mut stick_config.y_waveshaping,
                };

                *to_adjust = (*to_adjust as i8
                    + match i {
                        11 | 15 | 12 | 16 => 1,
                        13 | 17 | 14 | 18 => -1,
                        _ => unreachable!(),
                    })
                .clamp(0, MAX_WAVESHAPING) as u8;

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: (127
                            + match stick {
                                Stick::ControlStick => match axis {
                                    StickAxis::XAxis => *to_adjust,
                                    StickAxis::YAxis => 0,
                                },
                                Stick::CStick => 0,
                            }) as u8,
                        stick_y: (127
                            + match stick {
                                Stick::ControlStick => match axis {
                                    StickAxis::XAxis => 0,
                                    StickAxis::YAxis => *to_adjust,
                                },
                                Stick::CStick => 0,
                            }) as u8,
                        cstick_x: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => match axis {
                                    StickAxis::XAxis => *to_adjust,
                                    StickAxis::YAxis => 0,
                                },
                            }) as u8,
                        cstick_y: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => match axis {
                                    StickAxis::XAxis => 0,
                                    StickAxis::YAxis => *to_adjust,
                                },
                            }) as u8,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // smoothing changes
            i if (19..=26).contains(&i) => {
                let stick = match i {
                    19..=22 => Stick::ControlStick,
                    23..=26 => Stick::CStick,
                    _ => unreachable!(),
                };

                let axis = match i {
                    19 | 23 | 21 | 25 => StickAxis::XAxis,
                    20 | 24 | 22 | 26 => StickAxis::YAxis,
                    _ => unreachable!(),
                };

                let stick_config = match stick {
                    Stick::ControlStick => &mut final_config.astick_config,
                    Stick::CStick => &mut final_config.cstick_config,
                };

                let to_adjust = match axis {
                    StickAxis::XAxis => &mut stick_config.x_smoothing,
                    StickAxis::YAxis => &mut stick_config.y_smoothing,
                };

                *to_adjust = (*to_adjust as i8
                    + match i {
                        19 | 23 | 20 | 24 => 1,
                        21 | 25 | 22 | 26 => -1,
                        _ => unreachable!(),
                    })
                .clamp(0, MAX_SMOOTHING) as u8;

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: (127
                            + match stick {
                                Stick::ControlStick => match axis {
                                    StickAxis::XAxis => *to_adjust,
                                    StickAxis::YAxis => 0,
                                },
                                Stick::CStick => 0,
                            }) as u8,
                        stick_y: (127
                            + match stick {
                                Stick::ControlStick => match axis {
                                    StickAxis::XAxis => 0,
                                    StickAxis::YAxis => *to_adjust,
                                },
                                Stick::CStick => 0,
                            }) as u8,
                        cstick_x: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => match axis {
                                    StickAxis::XAxis => *to_adjust,
                                    StickAxis::YAxis => 0,
                                },
                            }) as u8,
                        cstick_y: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => match axis {
                                    StickAxis::XAxis => 0,
                                    StickAxis::YAxis => *to_adjust,
                                },
                            }) as u8,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // cardinalsnap increase/decrease
            i if (27..=30).contains(&i) => {
                let stick = match i {
                    27 | 28 => Stick::ControlStick,
                    29 | 30 => Stick::CStick,
                    _ => unreachable!(),
                };

                let to_adjust = match stick {
                    Stick::ControlStick => &mut final_config.astick_config.cardinal_snapping,
                    Stick::CStick => &mut final_config.cstick_config.cardinal_snapping,
                };

                *to_adjust = (*to_adjust
                    + match i {
                        27 | 29 => 1,
                        28 | 30 => -1,
                        _ => unreachable!(),
                    })
                .clamp(-1, MAX_CARDINAL_SNAP);

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127,
                        stick_y: (127
                            + match stick {
                                Stick::ControlStick => *to_adjust,
                                Stick::CStick => 0,
                            }) as u8,
                        cstick_x: 127,
                        cstick_y: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => *to_adjust,
                            }) as u8,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // scaling changes
            i if (31..=34).contains(&i) => {
                let stick = match i {
                    31 | 32 => Stick::ControlStick,
                    33 | 34 => Stick::CStick,
                    _ => unreachable!(),
                };

                let to_adjust = match stick {
                    Stick::ControlStick => &mut final_config.astick_config.analog_scaler,
                    Stick::CStick => &mut final_config.cstick_config.analog_scaler,
                };

                *to_adjust = (*to_adjust as i8
                    + match i {
                        31 | 33 => 1,
                        32 | 34 => -1,
                        _ => unreachable!(),
                    })
                .clamp(MIN_ANALOG_SCALER as i8, MAX_ANALOG_SCALER as i8)
                    as u8;

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127,
                        stick_y: (127
                            + match stick {
                                Stick::ControlStick => *to_adjust,
                                Stick::CStick => 0,
                            }) as u8,
                        cstick_x: 127,
                        cstick_y: (127
                            + match stick {
                                Stick::ControlStick => 0,
                                Stick::CStick => *to_adjust,
                            }) as u8,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // rumble strength changes
            i if (35..=36).contains(&i) => {
                let to_adjust = &mut final_config.rumble_strength;

                *to_adjust = (*to_adjust as i8
                    + match i {
                        35 => 1,
                        36 => -1,
                        _ => unreachable!(),
                    })
                .clamp(0, MAX_RUMBLE_STRENGTH) as u8;

                SIGNAL_CHANGE_RUMBLE_STRENGTH.signal(*to_adjust);

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            button_z: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127,
                        stick_y: 127 + *to_adjust,
                        cstick_x: 127,
                        cstick_y: 127,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // input consistency toggle
            37 => {
                final_config.input_consistency_mode = match final_config.input_consistency_mode {
                    InputConsistencyMode::Original => InputConsistencyMode::PC,
                    InputConsistencyMode::PC => InputConsistencyMode::ConsistencyHack,
                    InputConsistencyMode::ConsistencyHack => InputConsistencyMode::SuperHack,
                    InputConsistencyMode::SuperHack => InputConsistencyMode::Original,
                };

                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127,
                        stick_y: (127_i8
                            + match final_config.input_consistency_mode {
                                InputConsistencyMode::Original => -69,
                                InputConsistencyMode::PC => -42,
                                InputConsistencyMode::ConsistencyHack => 42,
                                InputConsistencyMode::SuperHack => 69,
                            }) as u8,
                        cstick_x: 127,
                        cstick_y: 127,
                    }
                    .into(),
                    duration_ms: 750,
                })
                .await;

                SIGNAL_CONFIG_CHANGE.signal(final_config.clone());
            }
            // display waveshaping values on both sticks
            38 => {
                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127 + final_config.astick_config.x_waveshaping,
                        stick_y: 127 + final_config.astick_config.y_waveshaping,
                        cstick_x: 127 + final_config.cstick_config.x_waveshaping,
                        cstick_y: 127 + final_config.cstick_config.y_waveshaping,
                    }
                    .into(),
                    duration_ms: 1000,
                })
                .await;
            }
            // display stick smoothing values on both sticks
            39 => {
                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: 127 + final_config.astick_config.x_smoothing,
                        stick_y: 127 + final_config.astick_config.y_smoothing,
                        cstick_x: 127 + final_config.cstick_config.x_smoothing,
                        cstick_y: 127 + final_config.cstick_config.y_smoothing,
                    }
                    .into(),
                    duration_ms: 1000,
                })
                .await;
            }
            // display snapback values on both sticks
            40 => {
                override_gcc_state_and_wait(&OverrideGcReportInstruction {
                    report: GcState {
                        trigger_r: 255,
                        trigger_l: 255,
                        buttons_2: GcButtons2 {
                            button_r: true,
                            button_l: true,
                            ..Default::default()
                        },
                        buttons_1: GcButtons1 {
                            button_x: true,
                            button_y: true,
                            button_a: true,
                            ..Default::default()
                        },
                        stick_x: (127 + final_config.astick_config.x_snapback) as u8,
                        stick_y: (127 + final_config.astick_config.y_snapback) as u8,
                        cstick_x: (127 + final_config.cstick_config.x_snapback) as u8,
                        cstick_y: (127 + final_config.cstick_config.y_snapback) as u8,
                    }
                    .into(),
                    duration_ms: 1000,
                })
                .await;
            }
            s => {
                error!("Invalid selection in config loop: {}", s);
                continue;
            }
        };

        final_config.write_to_flash(flash).unwrap();
    }

    info!("Exiting config main loop.");

    final_config
}

#[embassy_executor::task]
pub async fn config_task(mut flash: Flash<'static, FLASH, Async, FLASH_SIZE>) {
    let mut gcc_subscriber = CHANNEL_CONTROLLER_STATE.subscriber().unwrap();

    info!("Config task is running.");

    // We are loading the config from flash slightly deferred mainly because
    // if a debug probe is connected, it could potentially interfere.
    // This means we need to dispatch "updated" config status at least once to the
    // other tasks.

    Timer::after_millis(10).await;

    let mut current_config = ControllerConfig::from_flash_memory(&mut flash).unwrap();

    {
        let mut m_input_consistency = MUTEX_INPUT_CONSISTENCY_MODE.lock().await;
        *m_input_consistency = Some(current_config.input_consistency_mode);
    }

    SIGNAL_CHANGE_RUMBLE_STRENGTH.signal(current_config.rumble_strength);
    SIGNAL_CONFIG_CHANGE.signal(current_config.clone());

    loop {
        let desired_config_state = SIGNAL_CONFIG_MODE_STATUS_CHANGE.wait().await;

        if !desired_config_state {
            info!("Desired config state is false, skipping config mode.");
            continue;
        }

        info!("Entering config mode.");

        override_gcc_state_and_wait(&OverrideGcReportInstruction {
            report: GcState {
                trigger_r: 255,
                trigger_l: 255,
                buttons_2: GcButtons2 {
                    button_l: true,
                    button_r: true,
                    ..Default::default()
                },
                buttons_1: GcButtons1 {
                    button_x: true,
                    button_y: true,
                    button_a: true,
                    ..Default::default()
                },
                stick_x: 127,
                stick_y: 127,
                cstick_x: 127,
                cstick_y: 127,
            }
            .into(),
            duration_ms: 1000,
        })
        .await;

        current_config =
            configuration_main_loop(&current_config, &mut flash, &mut gcc_subscriber).await;

        info!("Exiting config mode.");

        SIGNAL_CONFIG_MODE_STATUS_CHANGE.signal(false);
    }
}

#[embassy_executor::task]
pub async fn enter_config_mode_task() {
    let mut gcc_subscriber = CHANNEL_CONTROLLER_STATE.subscriber().unwrap();

    info!("Enter config mode task is running.");

    loop {
        gcc_subscriber
            .wait_for_simultaneous_button_presses(&CONFIG_MODE_ENTRY_COMBO)
            .await;

        info!("Signalling config mode status change.");

        SIGNAL_CONFIG_MODE_STATUS_CHANGE.signal(true);

        Timer::after_millis(1000).await;

        while SIGNAL_CONFIG_MODE_STATUS_CHANGE.wait().await {}
    }
}
