/**
 *  Storage for controller configuration, including helper functions & types, as well as sane defaults.
 */
use core::f32::consts::PI;

use defmt::{info, warn, Format};
use embassy_rp::{
    flash::{Async, Flash, ERASE_SIZE},
    peripherals::FLASH,
};
use packed_struct::{derive::PackedStruct, PackedStruct};

use crate::{
    packed_float::{PackedFloat, ToPackedFloatArray},
    stick::{NotchStatus, NO_OF_CALIBRATION_POINTS, NO_OF_NOTCHES},
    ADDR_OFFSET, FLASH_SIZE,
};

/// This needs to be incremented for ANY change to ControllerConfig
/// else we risk loading uninitialized memory.
pub const CONTROLLER_CONFIG_REVISION: u8 = 2;

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

const DEFAULT_ANGLES: [f32; NO_OF_NOTCHES] = [
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

#[derive(Debug, Clone, Format, PackedStruct)]
#[packed_struct(endian = "msb")]
pub struct ControllerConfig {
    #[packed_field(size_bits = "8")]
    pub config_revision: u8,
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
    #[packed_field(size_bits = "8")]
    pub astick_analog_scaler: u8,
    #[packed_field(size_bits = "8")]
    pub cstick_analog_scaler: u8,
    #[packed_field(size_bits = "8")]
    pub x_snapback: i8,
    #[packed_field(size_bits = "8")]
    pub y_snapback: i8,
    #[packed_field(size_bits = "8")]
    pub x_smoothing: u8,
    #[packed_field(size_bits = "8")]
    pub y_smoothing: u8,
    #[packed_field(size_bits = "8")]
    pub c_xsmoothing: u8,
    #[packed_field(size_bits = "8")]
    pub c_ysmoothing: u8,
    #[packed_field(element_size_bytes = "4")]
    pub temp_cal_points_ax: [PackedFloat; 32],
    #[packed_field(element_size_bytes = "4")]
    pub temp_cal_points_ay: [PackedFloat; 32],
    #[packed_field(element_size_bytes = "4")]
    pub temp_cal_points_cx: [PackedFloat; 32],
    #[packed_field(element_size_bytes = "4")]
    pub temp_cal_points_cy: [PackedFloat; 32],
    #[packed_field(element_size_bytes = "4")]
    pub a_angles: [PackedFloat; 16],
    #[packed_field(element_size_bytes = "4")]
    pub c_angles: [PackedFloat; 16],
}

impl Default for ControllerConfig {
    fn default() -> Self {
        Self {
            config_revision: CONTROLLER_CONFIG_REVISION,
            config_version: 0,
            ax_waveshaping: 0,
            ay_waveshaping: 0,
            cx_waveshaping: 0,
            cy_waveshaping: 0,
            astick_analog_scaler: 0,
            cstick_analog_scaler: 0,
            x_snapback: 0,
            y_snapback: 0,
            x_smoothing: 0,
            y_smoothing: 0,
            c_xsmoothing: 0,
            c_ysmoothing: 0,
            temp_cal_points_ax: *DEFAULT_CAL_POINTS_X.to_packed_float_array(),
            temp_cal_points_ay: *DEFAULT_CAL_POINTS_Y.to_packed_float_array(),
            temp_cal_points_cx: *DEFAULT_CAL_POINTS_X.to_packed_float_array(),
            temp_cal_points_cy: *DEFAULT_CAL_POINTS_Y.to_packed_float_array(),
            a_angles: *DEFAULT_ANGLES.to_packed_float_array(),
            c_angles: *DEFAULT_ANGLES.to_packed_float_array(),
        }
    }
}

impl ControllerConfig {
    pub fn from_flash_memory(
        mut flash: &mut Flash<'static, FLASH, Async, FLASH_SIZE>,
    ) -> Result<Self, embassy_rp::flash::Error> {
        let mut controller_config_packed: <ControllerConfig as packed_struct::PackedStruct>::ByteArray = [0u8; 654]; // ControllerConfig byte size
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
