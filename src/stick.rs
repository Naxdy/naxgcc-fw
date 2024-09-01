// vast majority of this is taken from Phob firmware

use core::f32::consts::PI;

use defmt::{debug, info, trace, Format};
use libm::{atan2f, cosf, fabs, fabsf, fmaxf, fminf, roundf, sinf, sqrtf};

use crate::{
    config::{StickConfig, DEFAULT_ANGLES, DEFAULT_NOTCH_STATUS},
    helpers::{ToRegularArray, XyValuePair},
};

/// fit order for the linearization
const FIT_ORDER: usize = 3;
const NUM_COEFFS: usize = FIT_ORDER + 1;
pub const NO_OF_NOTCHES: usize = 16;
pub const NO_OF_ADJ_NOTCHES: usize = 12;
pub const NO_OF_CALIBRATION_POINTS: usize = 32;
const MAX_ORDER: usize = 20;

/// 28 degrees; this is the max angular deflection of the stick.
const MAX_STICK_ANGLE: f32 = 0.488_692_2;

#[rustfmt::skip]
//                                                                right        notch 1      up right     notch 2      up           notch 3      up left      notch 4      left         notch 5      down left    notch 6      down         notch 7      down right   notch 8
//                                                                0            1            2            3            4            5            6            7            8            9            10           11           12           13           14           15
pub const CALIBRATION_ORDER: [usize; NO_OF_CALIBRATION_POINTS] = [ 0, 1,        8, 9,       16, 17,       24, 25,      4, 5,        12, 13,      20, 21,      28, 29,      2, 3,        6, 7,        10, 11,      14, 15,      18, 19,      22, 23,      26, 27,      30, 31 ];

#[rustfmt::skip]
//                                                              up right     up left      down left    down right   notch 1      notch 2      notch 3      notch 4      notch 5      notch 6      notch 7      notch 8
pub const NOTCH_ADJUSTMENT_ORDER: [usize; NO_OF_ADJ_NOTCHES] = [2,           6,           10,          14,          1,           3,           5,           7,           9,           11,          13,          15];

#[derive(Clone, Debug, Default, Format)]
pub struct StickParams {
    // these are the linearization coefficients
    pub fit_coeffs: XyValuePair<[f32; NUM_COEFFS]>,

    // these are the notch remap parameters
    pub affine_coeffs: [[f32; 4]; 16], // affine transformation coefficients for all regions of the stick
    pub boundary_angles: [f32; 16], // angles at the boundaries between regions of the stick (in the plane)
}

impl From<&StickConfig> for StickParams {
    /// Generate a StickParam struct from a stick config
    fn from(stick_config: &StickConfig) -> Self {
        let cleaned_cal_points = CleanedCalibrationPoints::from_temp_calibration_points(
            stick_config.cal_points_x.to_regular_array(),
            stick_config.cal_points_y.to_regular_array(),
            stick_config.angles.to_regular_array(),
        );

        let linearized_cal = LinearizedCalibration::from_calibration_points(&cleaned_cal_points);

        let notch_cal = NotchCalibration::from_cleaned_and_linearized_calibration(
            &cleaned_cal_points,
            &linearized_cal,
        );

        Self {
            fit_coeffs: XyValuePair {
                x: linearized_cal.fit_coeffs.x.map(|e| e as f32),
                y: linearized_cal.fit_coeffs.y.map(|e| e as f32),
            },
            affine_coeffs: notch_cal.affine_coeffs,
            boundary_angles: notch_cal.boundary_angles,
        }
    }
}

#[derive(Clone, Debug, Format, Copy, Eq, PartialEq)]
pub enum NotchStatus {
    TertInactive,
    TertActive,
    Secondary,
    Cardinal,
}

#[derive(Clone, Debug, Format)]
pub struct CleanedCalibrationPoints {
    pub cleaned_points: XyValuePair<[f32; NO_OF_NOTCHES + 1]>,
    pub notch_points: XyValuePair<[f32; NO_OF_NOTCHES + 1]>,
    pub notch_status: [NotchStatus; NO_OF_NOTCHES],
}

impl Default for CleanedCalibrationPoints {
    fn default() -> Self {
        Self {
            cleaned_points: XyValuePair {
                x: [0f32; NO_OF_NOTCHES + 1],
                y: [0f32; NO_OF_NOTCHES + 1],
            },
            notch_points: XyValuePair {
                x: [0f32; NO_OF_NOTCHES + 1],
                y: [0f32; NO_OF_NOTCHES + 1],
            },
            notch_status: DEFAULT_NOTCH_STATUS,
        }
    }
}

impl CleanedCalibrationPoints {
    pub fn from_temp_calibration_points(
        cal_points_x: &[f32; NO_OF_CALIBRATION_POINTS],
        cal_points_y: &[f32; NO_OF_CALIBRATION_POINTS],
        notch_angles: &[f32; NO_OF_NOTCHES],
    ) -> Self {
        let mut out = Self::default();

        trace!(
            "Raw calibration points x {} and y {}:",
            cal_points_x,
            cal_points_y
        );

        debug!("Notch angles: {}", notch_angles);

        for i in 0..NO_OF_NOTCHES {
            // add the origin values to the first x,y point
            out.cleaned_points.x[0] += cal_points_x[i * 2];
            out.cleaned_points.y[0] += cal_points_y[i * 2];

            // copy the cal point into the cleaned list
            out.cleaned_points.x[i + 1] = cal_points_x[i * 2 + 1];
            out.cleaned_points.y[i + 1] = cal_points_y[i * 2 + 1];

            (out.notch_points.x[i + 1], out.notch_points.y[i + 1]) = {
                let (a, b) = calc_stick_values(notch_angles[i]);
                (roundf(a), roundf(b))
            }
        }

        // TODO: put the below in a macro to clean it up a bit, once it's confirmed to work
        // remove the largest and smallest two origin values to remove outliers
        // first, find their indices
        let mut smallest_x = 0;
        let mut small_x = 0;
        let mut large_x = 0;
        let mut largest_x = 0;

        let mut smallest_y = 0;
        let mut small_y = 0;
        let mut large_y = 0;
        let mut largest_y = 0;

        for i in 0..NO_OF_NOTCHES {
            if cal_points_x[i * 2] < cal_points_x[smallest_x] {
                small_x = smallest_x;
                smallest_x = i * 2;
            } else if cal_points_x[i * 2] < cal_points_x[small_x] {
                small_x = i * 2;
            }

            if cal_points_x[i * 2] > cal_points_x[largest_x] {
                large_x = largest_x;
                largest_x = i * 2;
            } else if cal_points_x[i * 2] > cal_points_x[large_x] {
                large_x = i * 2;
            }

            if cal_points_y[i * 2] < cal_points_y[smallest_y] {
                small_y = smallest_y;
                smallest_y = i * 2;
            } else if cal_points_y[i * 2] < cal_points_y[small_y] {
                small_y = i * 2;
            }

            if cal_points_y[i * 2] > cal_points_y[largest_y] {
                large_y = largest_y;
                largest_y = i * 2;
            } else if cal_points_y[i * 2] > cal_points_y[large_y] {
                large_y = i * 2;
            }
        }

        // TODO: make this whole thing a function? it looks very ugly
        out.cleaned_points.x[0] -= cal_points_x[smallest_x];
        out.cleaned_points.x[0] -= cal_points_x[small_x];
        out.cleaned_points.x[0] -= cal_points_x[large_x];
        out.cleaned_points.x[0] -= cal_points_x[largest_x];

        out.cleaned_points.y[0] -= cal_points_y[smallest_y];
        out.cleaned_points.y[0] -= cal_points_y[small_y];
        out.cleaned_points.y[0] -= cal_points_y[large_y];
        out.cleaned_points.y[0] -= cal_points_y[largest_y];

        out.cleaned_points.x[0] /= (NO_OF_NOTCHES - 4) as f32;
        out.cleaned_points.y[0] /= (NO_OF_NOTCHES - 4) as f32;

        #[allow(clippy::needless_range_loop)]
        for i in 0..NO_OF_NOTCHES {
            let delta_x = out.cleaned_points.x[i + 1] - out.cleaned_points.x[0];
            let delta_y = out.cleaned_points.y[i + 1] - out.cleaned_points.y[0];
            let mag = sqrtf(delta_x * delta_x + delta_y * delta_y);

            // if the cleaned point was at the center and would be a firefox notch
            // average the previous and next points (cardinal & diagonal) for some sanity
            if mag < 0.02 && (i % 2 != 0) {
                let prev_index = ((i - 1 + NO_OF_NOTCHES) % NO_OF_NOTCHES) + 1;
                let next_index = ((i + 1) % NO_OF_NOTCHES) + 1;

                out.cleaned_points.x[i + 1] =
                    (out.cleaned_points.x[prev_index] + out.cleaned_points.x[next_index]) / 2.0;
                out.cleaned_points.y[i + 1] =
                    (out.cleaned_points.y[prev_index] + out.cleaned_points.y[next_index]) / 2.0;

                out.notch_points.x[i + 1] =
                    (out.notch_points.x[prev_index] + out.notch_points.x[next_index]) / 2.0;
                out.notch_points.y[i + 1] =
                    (out.notch_points.y[prev_index] + out.notch_points.y[next_index]) / 2.0;

                trace!("Skipping notch {}", i + 1);

                // Mark that notch adjustment should be skipped for this
                out.notch_status[i] = NotchStatus::TertInactive;
            } else {
                out.notch_status[i] = DEFAULT_NOTCH_STATUS[i];
            }
        }

        trace!(
            "Final points clean_x: {:?}, clean_y: {:?}, notch_x: {:?}, notch_y: {:?}",
            out.cleaned_points.x,
            out.cleaned_points.y,
            out.notch_points.x,
            out.notch_points.y
        );

        trace!("The notch statuses are: {:?}", out.notch_status);

        out
    }
}

#[derive(Clone, Debug, Default)]
pub struct LinearizedCalibration {
    pub fit_coeffs: XyValuePair<[f64; NUM_COEFFS]>,

    pub linearized_points: XyValuePair<[f32; NO_OF_NOTCHES + 1]>,
}

impl LinearizedCalibration {
    ///
    /// Generate a fit to linearize the stick response.
    ///
    /// Inputs: cleaned points X and Y, (must be 17 points for each of these, the first being the center, the others starting at 3 oclock and going around counterclockwise)
    ///
    /// Outputs: linearization fit coefficients for X and Y
    pub fn from_calibration_points(cleaned_calibration_points: &CleanedCalibrationPoints) -> Self {
        let mut fit_points_x = [0f64; 5];
        let mut fit_points_y = [0f64; 5];

        let in_x = cleaned_calibration_points
            .cleaned_points
            .x
            .map(|e| e as f64);
        let in_y = cleaned_calibration_points
            .cleaned_points
            .y
            .map(|e| e as f64);

        fit_points_x[0] = in_x[8 + 1];
        fit_points_x[1] = (in_x[6 + 1] + in_x[10 + 1]) / 2.0f64;
        fit_points_x[2] = in_x[0];
        fit_points_x[3] = (in_x[2 + 1] + in_x[14 + 1]) / 2.0f64;
        fit_points_x[4] = in_x[1];

        fit_points_y[0] = in_y[12 + 1];
        fit_points_y[1] = (in_y[10 + 1] + in_y[14 + 1]) / 2.0f64;
        fit_points_y[2] = in_y[0];
        fit_points_y[3] = (in_y[6 + 1] + in_y[2 + 1]) / 2.0f64;
        fit_points_y[4] = in_y[4 + 1];

        let x_output: [f64; 5] = [27.5, 53.2537879754, 127.5, 201.7462120246, 227.5];
        let y_output: [f64; 5] = [27.5, 53.2537879754, 127.5, 201.7462120246, 227.5];

        let mut fit_coeffs_x =
            fit_curve::<5, NUM_COEFFS>(FIT_ORDER as i32, &fit_points_x, &x_output);
        let mut fit_coeffs_y =
            fit_curve::<5, NUM_COEFFS>(FIT_ORDER as i32, &fit_points_y, &y_output);

        let x_zero_error = linearize(fit_points_x[2] as f32, &fit_coeffs_x.map(|e| e as f32));
        let y_zero_error = linearize(fit_points_y[2] as f32, &fit_coeffs_y.map(|e| e as f32));

        fit_coeffs_x[3] -= x_zero_error as f64;
        fit_coeffs_y[3] -= y_zero_error as f64;

        let mut linearized_points_x = [0f32; NO_OF_NOTCHES + 1];
        let mut linearized_points_y = [0f32; NO_OF_NOTCHES + 1];

        for i in 0..=NO_OF_NOTCHES {
            linearized_points_x[i] = linearize(in_x[i] as f32, &fit_coeffs_x.map(|e| e as f32));
            linearized_points_y[i] = linearize(in_y[i] as f32, &fit_coeffs_y.map(|e| e as f32));
        }

        debug!(
            "Linearized points x: {:?}, y: {:?}",
            linearized_points_x, linearized_points_y
        );

        Self {
            fit_coeffs: XyValuePair {
                x: fit_coeffs_x,
                y: fit_coeffs_y,
            },
            linearized_points: XyValuePair {
                x: linearized_points_x,
                y: linearized_points_y,
            },
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct NotchCalibration {
    pub affine_coeffs: [[f32; 4]; 16],
    pub boundary_angles: [f32; 16],
}

impl NotchCalibration {
    pub fn from_cleaned_and_linearized_calibration(
        cleaned_calibration_points: &CleanedCalibrationPoints,
        linearized_calibration: &LinearizedCalibration,
    ) -> Self {
        let mut out = Self::default();

        for i in 1..=NO_OF_NOTCHES {
            let mut points_in = [[0f32; 3]; 3];
            let mut points_out = [[0f32; 3]; 3];

            if i == NO_OF_NOTCHES {
                points_in[0][0] = linearized_calibration.linearized_points.x[0];
                points_in[0][1] = linearized_calibration.linearized_points.x[i];
                points_in[0][2] = linearized_calibration.linearized_points.x[1];
                points_in[1][0] = linearized_calibration.linearized_points.y[0];
                points_in[1][1] = linearized_calibration.linearized_points.y[i];
                points_in[1][2] = linearized_calibration.linearized_points.y[1];
                points_in[2][0] = 1.;
                points_in[2][1] = 1.;
                points_in[2][2] = 1.;
                points_out[0][0] = cleaned_calibration_points.notch_points.x[0];
                points_out[0][1] = cleaned_calibration_points.notch_points.x[i];
                points_out[0][2] = cleaned_calibration_points.notch_points.x[1];
                points_out[1][0] = cleaned_calibration_points.notch_points.y[0];
                points_out[1][1] = cleaned_calibration_points.notch_points.y[i];
                points_out[1][2] = cleaned_calibration_points.notch_points.y[1];
                points_out[2][0] = 1.;
                points_out[2][1] = 1.;
                points_out[2][2] = 1.;
            } else {
                points_in[0][0] = linearized_calibration.linearized_points.x[0];
                points_in[0][1] = linearized_calibration.linearized_points.x[i];
                points_in[0][2] = linearized_calibration.linearized_points.x[i + 1];
                points_in[1][0] = linearized_calibration.linearized_points.y[0];
                points_in[1][1] = linearized_calibration.linearized_points.y[i];
                points_in[1][2] = linearized_calibration.linearized_points.y[i + 1];
                points_in[2][0] = 1.;
                points_in[2][1] = 1.;
                points_in[2][2] = 1.;
                points_out[0][0] = cleaned_calibration_points.notch_points.x[0];
                points_out[0][1] = cleaned_calibration_points.notch_points.x[i];
                points_out[0][2] = cleaned_calibration_points.notch_points.x[i + 1];
                points_out[1][0] = cleaned_calibration_points.notch_points.y[0];
                points_out[1][1] = cleaned_calibration_points.notch_points.y[i];
                points_out[1][2] = cleaned_calibration_points.notch_points.y[i + 1];
                points_out[2][0] = 1.;
                points_out[2][1] = 1.;
                points_out[2][2] = 1.;
            }
            trace!("In points: {:?}", points_in);
            trace!("Out points: {:?}", points_out);

            let temp = inverse(&points_in);

            let a = matrix_mult(&points_out, &temp);

            trace!("The transform matrix is: {:?}", a);

            #[allow(clippy::needless_range_loop)]
            for j in 0..2 {
                for k in 0..2 {
                    out.affine_coeffs[i - 1][j * 2 + k] = a[j][k];
                }
            }

            trace!(
                "Transform coefficients for this region are: {:?}",
                out.affine_coeffs[i - 1]
            );

            out.boundary_angles[i - 1] = match atan2f(
                linearized_calibration.linearized_points.y[i]
                    - linearized_calibration.linearized_points.y[0],
                linearized_calibration.linearized_points.x[i]
                    - linearized_calibration.linearized_points.x[0],
            ) {
                a if a < out.boundary_angles[0] => a + 2. * PI,
                a => a,
            };
        }

        out
    }
}

#[derive(Debug, Clone, Format)]
pub struct AppliedCalibration {
    pub stick_params: StickParams,
    pub cleaned_calibration: CleanedCalibrationPoints,
    pub notch_angles: [f32; NO_OF_NOTCHES],
    pub measured_notch_angles: [f32; NO_OF_NOTCHES],
}

impl Default for AppliedCalibration {
    fn default() -> Self {
        Self {
            stick_params: StickParams::default(),
            cleaned_calibration: CleanedCalibrationPoints::default(),
            notch_angles: DEFAULT_ANGLES,
            measured_notch_angles: [0f32; NO_OF_NOTCHES],
        }
    }
}

impl AppliedCalibration {
    pub fn from_points(
        cal_points_x: &[f32; NO_OF_CALIBRATION_POINTS],
        cal_points_y: &[f32; NO_OF_CALIBRATION_POINTS],
        stick_config: &StickConfig,
    ) -> Self {
        let mut stick_params = StickParams::from(stick_config);

        let (stripped_cal_points_x, stripped_cal_points_y) =
            strip_cal_points(cal_points_x, cal_points_y);

        let stripped_cleaned_calibration = CleanedCalibrationPoints::from_temp_calibration_points(
            &stripped_cal_points_x,
            &stripped_cal_points_y,
            &DEFAULT_ANGLES,
        );

        let linearized_calibration =
            LinearizedCalibration::from_calibration_points(&stripped_cleaned_calibration);

        stick_params.fit_coeffs = XyValuePair {
            x: linearized_calibration.fit_coeffs.x.map(|e| e as f32),
            y: linearized_calibration.fit_coeffs.y.map(|e| e as f32),
        };

        let notch_calibration = NotchCalibration::from_cleaned_and_linearized_calibration(
            &stripped_cleaned_calibration,
            &linearized_calibration,
        );

        stick_params.affine_coeffs = notch_calibration.affine_coeffs;
        stick_params.boundary_angles = notch_calibration.boundary_angles;

        let original_cleaned_calibration = CleanedCalibrationPoints::from_temp_calibration_points(
            cal_points_x,
            cal_points_y,
            &DEFAULT_ANGLES,
        );

        let (transformed_cal_points_x, transformed_cal_points_y) = transform_cal_points(
            &original_cleaned_calibration.cleaned_points.x,
            &original_cleaned_calibration.cleaned_points.y,
            &stick_params,
            stick_config,
        );

        info!(
            "Transformed calibration points x: {:?}, y: {:?}",
            transformed_cal_points_x, transformed_cal_points_y
        );

        let measured_notch_angles =
            compute_stick_angles(&transformed_cal_points_x, &transformed_cal_points_y);

        info!("Measured notch angles: {:?}", measured_notch_angles);

        let cleaned_with_measured_notch_angles =
            CleanedCalibrationPoints::from_temp_calibration_points(
                cal_points_x,
                cal_points_y,
                &measured_notch_angles,
            );

        let cleaned_notch_angles = clean_notches(
            &measured_notch_angles,
            &cleaned_with_measured_notch_angles.notch_status,
        );

        let cleaned_full = CleanedCalibrationPoints::from_temp_calibration_points(
            cal_points_x,
            cal_points_y,
            &cleaned_notch_angles,
        );

        let linearized_full = LinearizedCalibration::from_calibration_points(&cleaned_full);

        stick_params.fit_coeffs = XyValuePair {
            x: linearized_full.fit_coeffs.x.map(|e| e as f32),
            y: linearized_full.fit_coeffs.y.map(|e| e as f32),
        };

        let notch_calibrate_full = NotchCalibration::from_cleaned_and_linearized_calibration(
            &cleaned_full,
            &linearized_full,
        );

        stick_params.affine_coeffs = notch_calibrate_full.affine_coeffs;
        stick_params.boundary_angles = notch_calibrate_full.boundary_angles;

        Self {
            stick_params,
            measured_notch_angles,
            notch_angles: cleaned_notch_angles,
            cleaned_calibration: cleaned_full,
        }
    }
}

pub fn legalize_notches(
    current_step: usize,
    measured_notch_angles: &[f32; NO_OF_NOTCHES],
    notch_angles: &[f32; NO_OF_NOTCHES],
) -> [f32; NO_OF_NOTCHES] {
    let mut out = *notch_angles;

    for i in current_step..44 {
        let idx = NOTCH_ADJUSTMENT_ORDER[i - NO_OF_CALIBRATION_POINTS];
        out[idx] = legalize_notch(idx as isize, measured_notch_angles, &out);
    }

    out
}

fn legalize_notch(
    idx: isize,
    measured_notch_angles: &[f32; NO_OF_NOTCHES],
    notch_angles: &[f32; NO_OF_NOTCHES],
) -> f32 {
    let is_diagonal = (idx - 2) % 4 == 0;

    let prev_idx = if is_diagonal {
        (idx - 2 + NO_OF_NOTCHES as isize) % NO_OF_NOTCHES as isize
    } else {
        (idx - 1 + NO_OF_NOTCHES as isize) % NO_OF_NOTCHES as isize
    } as usize;
    let next_idx = if is_diagonal {
        (idx + 2) % NO_OF_NOTCHES as isize
    } else {
        (idx + 1) % NO_OF_NOTCHES as isize
    } as usize;

    let prev_angle = notch_angles[prev_idx];
    let next_angle = match notch_angles[next_idx] {
        a if a < prev_angle => a + 2. * PI,
        a => a,
    };

    let prev_meas_angle = measured_notch_angles[prev_idx];
    let this_meas_angle = measured_notch_angles[idx as usize];
    let next_meas_angle = match measured_notch_angles[next_idx] {
        a if a < prev_meas_angle => a + 2. * PI,
        a => a,
    };

    let (cmp_amt, str_amt) = if is_diagonal {
        (0.769, 1.3)
    } else {
        (0.666, 1.5)
    };

    let min_threshold = 0.15 / 0.975;
    let deadzone_limit = 0.2875 / 0.95;
    let deadzone_plus = 0.325 / 0.9375;

    let lower_compress_limit = prev_angle + cmp_amt * (this_meas_angle - prev_meas_angle);
    let upper_compress_limit = next_angle - cmp_amt * (next_meas_angle - this_meas_angle);

    let lower_strech_limit = if next_idx % 4 == 0
        && !is_diagonal
        && (next_meas_angle - this_meas_angle) > min_threshold
        && (next_meas_angle - this_meas_angle) < deadzone_limit
    {
        next_angle - fmaxf(str_amt * (next_meas_angle - this_meas_angle), deadzone_plus)
    } else {
        next_angle - str_amt * (next_meas_angle - this_meas_angle)
    };

    let upper_strech_limit = if prev_idx % 4 == 0
        && !is_diagonal
        && (this_meas_angle - prev_meas_angle) > min_threshold
        && (this_meas_angle - prev_meas_angle) < deadzone_limit
    {
        prev_angle + fmaxf(str_amt * (this_meas_angle - prev_meas_angle), deadzone_plus)
    } else {
        prev_angle + str_amt * (this_meas_angle - prev_meas_angle)
    };

    let lower_distort_limit = fmaxf(lower_compress_limit, lower_strech_limit);
    let upper_distort_limit = match fminf(upper_compress_limit, upper_strech_limit) {
        a if a < lower_distort_limit => a + 2. * PI,
        a => a,
    };

    fminf(
        upper_distort_limit,
        fmaxf(notch_angles[idx as usize], lower_distort_limit),
    )
}

/// Sets notches to measured values if absent.
fn clean_notches(
    measured_notch_angles: &[f32; NO_OF_NOTCHES],
    notch_status: &[NotchStatus; NO_OF_NOTCHES],
) -> [f32; NO_OF_NOTCHES] {
    let mut out = [0f32; NO_OF_NOTCHES];

    for i in 0..NO_OF_NOTCHES {
        if notch_status[i] == NotchStatus::TertInactive {
            out[i] = measured_notch_angles[i];
        }
    }

    out
}

fn angle_on_sphere(x: f32, y: f32) -> f32 {
    let xx = sinf(x * MAX_STICK_ANGLE / 100.) * cosf(y * MAX_STICK_ANGLE / 100.);
    let yy = cosf(x * MAX_STICK_ANGLE / 100.) * sinf(y * MAX_STICK_ANGLE / 100.);
    match atan2f(yy, xx) {
        a if a < 0. => a + 2. * PI,
        a => a,
    }
}

fn compute_stick_angles(
    x_in: &[f32; NO_OF_NOTCHES + 1],
    y_in: &[f32; NO_OF_NOTCHES + 1],
) -> [f32; NO_OF_NOTCHES] {
    let mut angles = [0f32; NO_OF_NOTCHES];

    for i in 0..NO_OF_NOTCHES {
        if i % 2 == 0 {
            angles[i] = DEFAULT_ANGLES[i];
        } else {
            angles[i] = angle_on_sphere(x_in[i + 1], y_in[i + 1]);
            debug!(
                "Computed angle for x,y: ({}, {}) is: {}",
                x_in[i + 1],
                y_in[i + 1],
                angles[i]
            );
        }
    }

    angles
}

fn transform_cal_points(
    cal_points_x: &[f32; NO_OF_NOTCHES + 1],
    cal_points_y: &[f32; NO_OF_NOTCHES + 1],
    stick_params: &StickParams,
    stick_config: &StickConfig,
) -> ([f32; NO_OF_NOTCHES + 1], [f32; NO_OF_NOTCHES + 1]) {
    let mut transformed_points_x = [0f32; NO_OF_NOTCHES + 1];
    let mut transformed_points_y = [0f32; NO_OF_NOTCHES + 1];

    for i in 0..NO_OF_NOTCHES + 1 {
        let x = linearize(cal_points_x[i], &stick_params.fit_coeffs.x);
        let y = linearize(cal_points_y[i], &stick_params.fit_coeffs.y);
        let (out_x, out_y) = notch_remap(x, y, stick_params, stick_config, true);
        transformed_points_x[i] = out_x;
        transformed_points_y[i] = out_y;
    }

    (transformed_points_x, transformed_points_y)
}
/// Removes the notches from un-cleaned cal points
/// so we can get the original values of the notches after the affine transform.
fn strip_cal_points(
    cal_points_x: &[f32; NO_OF_CALIBRATION_POINTS],
    cal_points_y: &[f32; NO_OF_CALIBRATION_POINTS],
) -> (
    [f32; NO_OF_CALIBRATION_POINTS],
    [f32; NO_OF_CALIBRATION_POINTS],
) {
    let mut stripped_points_x = [0f32; NO_OF_CALIBRATION_POINTS];
    let mut stripped_points_y = [0f32; NO_OF_CALIBRATION_POINTS];
    for i in 0..NO_OF_CALIBRATION_POINTS {
        (stripped_points_x[i], stripped_points_y[i]) = if (i + 1) % 4 == 0 {
            (cal_points_x[0], cal_points_y[0])
        } else {
            (cal_points_x[i], cal_points_y[i])
        }
    }

    (stripped_points_x, stripped_points_y)
}

fn inverse(in_mat: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out_mat = [[0f32; 3]; 3];

    let det = in_mat[0][0] * (in_mat[1][1] * in_mat[2][2] - in_mat[2][1] * in_mat[1][2])
        - in_mat[0][1] * (in_mat[1][0] * in_mat[2][2] - in_mat[1][2] * in_mat[2][0])
        + in_mat[0][2] * (in_mat[1][0] * in_mat[2][1] - in_mat[1][1] * in_mat[2][0]);

    let invdet = 1. / det;

    out_mat[0][0] = (in_mat[1][1] * in_mat[2][2] - in_mat[2][1] * in_mat[1][2]) * invdet;
    out_mat[0][1] = (in_mat[0][2] * in_mat[2][1] - in_mat[0][1] * in_mat[2][2]) * invdet;
    out_mat[0][2] = (in_mat[0][1] * in_mat[1][2] - in_mat[0][2] * in_mat[1][1]) * invdet;
    out_mat[1][0] = (in_mat[1][2] * in_mat[2][0] - in_mat[1][0] * in_mat[2][2]) * invdet;
    out_mat[1][1] = (in_mat[0][0] * in_mat[2][2] - in_mat[0][2] * in_mat[2][0]) * invdet;
    out_mat[1][2] = (in_mat[1][0] * in_mat[0][2] - in_mat[0][0] * in_mat[1][2]) * invdet;
    out_mat[2][0] = (in_mat[1][0] * in_mat[2][1] - in_mat[2][0] * in_mat[1][1]) * invdet;
    out_mat[2][1] = (in_mat[2][0] * in_mat[0][1] - in_mat[0][0] * in_mat[2][1]) * invdet;
    out_mat[2][2] = (in_mat[0][0] * in_mat[1][1] - in_mat[1][0] * in_mat[0][1]) * invdet;

    out_mat
}

#[allow(clippy::needless_range_loop)]
fn matrix_mult(a: &[[f32; 3]; 3], b: &[[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let mut out = [[0f32; 3]; 3];

    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                out[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    out
}

/// Calculate the power of a number
fn curve_fit_power(base: f64, exponent: u32) -> f64 {
    if exponent == 0 {
        return 1.0;
    }

    let mut val = base;

    for _ in 1..exponent {
        val *= base;
    }

    val
}

/// Substitutes a column in a matrix with a vector
fn sub_col<const N: usize>(
    matrix: &[[f64; N]; N],
    t: &[f64; MAX_ORDER],
    col: usize,
    n: usize,
) -> [[f64; N]; N] {
    let mut m = *matrix;

    for i in 0..n {
        m[i][col] = t[i];
    }

    m
}

/// Calculate the determinant of a matrix
fn det<const N: usize>(matrix: &[[f64; N]; N]) -> f64 {
    let mut matrix = *matrix;
    let sign = trianglize(&mut matrix);

    if sign == 0 {
        return 0.;
    }

    let mut p = 1f64;

    for (i, elem) in matrix.iter().enumerate().take(N) {
        p *= elem[i];
    }

    p * (sign as f64)
}

/// Trianglize a matrix
fn trianglize<const N: usize>(matrix: &mut [[f64; N]; N]) -> i32 {
    let mut sign = 1;
    let mut matrix = *matrix;

    for i in 0..N {
        let mut max = 0;
        for row in i..N {
            if fabs(matrix[row][i]) > fabs(matrix[max][i]) {
                max = row;
            }
        }
        if max > 0 {
            sign = -sign;
            matrix.swap(i, max);
        }
        if matrix[i][i] == 0. {
            return 0;
        }
        for row in i + 1..N {
            let factor = matrix[row][i] / matrix[i][i];
            if factor == 0. {
                continue;
            }
            for col in i..N {
                matrix[row][col] -= factor * matrix[i][col];
            }
        }
    }

    sign
}

fn fit_curve<const N: usize, const NCOEFFS: usize>(
    order: i32,
    px: &[f64; N],
    py: &[f64; N],
) -> [f64; NCOEFFS] {
    let mut coeffs = [0f64; NCOEFFS];

    if NCOEFFS != (order + 1) as usize {
        panic!(
            "Invalid coefficients length, expected {}, but got {}",
            order + 1,
            NCOEFFS
        );
    }

    if NCOEFFS > MAX_ORDER || NCOEFFS < 2 {
        panic!("Matrix size out of bounds");
    }

    if N < 1 {
        panic!("Not enough points to fit");
    }

    let mut t = [0f64; MAX_ORDER];
    let mut s = [0f64; MAX_ORDER * 2 + 1];

    for i in 0..N {
        let x = px[i];
        let y = py[i];
        for (j, elem) in s.iter_mut().enumerate().take(NCOEFFS * 2 - 1) {
            *elem += curve_fit_power(x, j as u32);
        }
        for (j, elem) in t.iter_mut().enumerate().take(NCOEFFS) {
            *elem += y * curve_fit_power(x, j as u32);
        }
    }

    //Master matrix LHS of linear equation
    let mut matrix = [[0f64; NCOEFFS]; NCOEFFS];

    for i in 0..NCOEFFS {
        matrix[i][..NCOEFFS].copy_from_slice(&s[i..(NCOEFFS + i)]);
    }

    let denom = det(&matrix);

    for i in 0..NCOEFFS {
        coeffs[NCOEFFS - i - 1] = det(&sub_col(&matrix, &t, i, NCOEFFS)) / denom;
    }

    coeffs
}

/// Compute the stick x/y coordinates from a given angle.
/// The stick moves spherically, so it requires 3D trigonometry.
pub fn calc_stick_values(angle: f32) -> (f32, f32) {
    let x =
        100. * atan2f(sinf(MAX_STICK_ANGLE) * cosf(angle), cosf(MAX_STICK_ANGLE)) / MAX_STICK_ANGLE;
    let y =
        100. * atan2f(sinf(MAX_STICK_ANGLE) * sinf(angle), cosf(MAX_STICK_ANGLE)) / MAX_STICK_ANGLE;

    (x, y)
}

#[inline(never)]
#[link_section = ".time_critical.linearize"]
pub fn linearize(point: f32, coefficients: &[f32; NUM_COEFFS]) -> f32 {
    coefficients[0] * (point * point * point)
        + coefficients[1] * (point * point)
        + coefficients[2] * point
        + coefficients[3]
}

#[inline(never)]
#[link_section = ".time_critical.notch_remap"]
pub fn notch_remap(
    x_in: f32,
    y_in: f32,
    stick_params: &StickParams,
    stick_config: &StickConfig,
    is_calibrating: bool,
) -> (f32, f32) {
    //determine the angle between the x unit vector and the current position vector
    let angle = match atan2f(y_in, x_in) {
        //unwrap the angle based on the first region boundary
        a if a < stick_params.boundary_angles[0] => a + PI * 2.0,
        a => a,
    };

    //go through the region boundaries from lowest angle to highest, checking if the current position vector is in that region
    //if the region is not found then it must be between the first and the last boundary, ie the last region
    //we check GATE_REGIONS*2 because each notch has its own very small region we use to make notch values more consistent
    let region = 'a: {
        for i in 1..NO_OF_NOTCHES {
            if angle < stick_params.boundary_angles[i] {
                break 'a i - 1;
            }
        }
        NO_OF_NOTCHES - 1
    };

    let stick_scale = stick_config.analog_scaler as f32 / 100.;

    let mut x_out = stick_scale
        * (stick_params.affine_coeffs[region][0] * x_in
            + stick_params.affine_coeffs[region][1] * y_in);
    let mut y_out = stick_scale
        * (stick_params.affine_coeffs[region][2] * x_in
            + stick_params.affine_coeffs[region][3] * y_in);

    if !is_calibrating {
        if stick_config.cardinal_snapping > 0 {
            if fabsf(x_out) < stick_config.cardinal_snapping as f32 + 0.5 && fabsf(y_out) >= 79.5 {
                x_out = 0.;
            }
            if fabsf(y_out) < stick_config.cardinal_snapping as f32 + 0.5 && fabsf(x_out) >= 79.5 {
                y_out = 0.;
            }
        } else if stick_config.cardinal_snapping == -1 {
            if fabsf(x_out) < 6.5 && fabsf(y_out) >= 79.5 {
                x_out = 0.;
            }
            if fabsf(y_out) < 6.5 && fabsf(x_out) >= 79.5 {
                y_out = 0.;
            }
        }

        if fabsf(x_out) < 3. && fabsf(y_out) < 3. {
            x_out = 0.;
            y_out = 0.;
        }
    }

    (x_out, y_out)
}
