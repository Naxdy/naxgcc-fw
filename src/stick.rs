// vast majority of this is taken from Phob firmware

use core::f32::consts::PI;

use defmt::{debug, Format};
use libm::{atan2f, cosf, fabs, roundf, sinf, sqrtf};

use crate::{
    input::{ControllerConfig, Stick},
    packed_float::ToRegularArray,
};

/// fit order for the linearization
const FIT_ORDER: usize = 3;
const NUM_COEFFS: usize = FIT_ORDER + 1;
pub const NO_OF_NOTCHES: usize = 16;
pub const NO_OF_CALIBRATION_POINTS: usize = 32;
const MAX_ORDER: usize = 20;

/// 28 degrees; this is the max angular deflection of the stick.
const MAX_STICK_ANGLE: f32 = 0.4886921906;

const NOTCH_STATUS_DEFAULTS: [NotchStatus; NO_OF_NOTCHES] = [
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
pub const DEFAULT_CAL_POINTS_X: [f32; NO_OF_CALIBRATION_POINTS] = [
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
pub const DEFAULT_CAL_POINTS_Y: [f32; NO_OF_CALIBRATION_POINTS] = [
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

#[derive(Clone, Debug, Default, Format)]
pub struct StickParams {
    // these are the linearization coefficients
    pub fit_coeffs_x: [f32; NUM_COEFFS],
    pub fit_coeffs_y: [f32; NUM_COEFFS],

    // these are the notch remap parameters
    pub affine_coeffs: [[f32; 16]; 4], // affine transformation coefficients for all regions of the stick
    pub boundary_angles: [f32; 4], // angles at the boundaries between regions of the stick (in the plane)
}

impl StickParams {
    /// Generate StickParams structs for the sticks, returned as a tuple of (analog_stick, c_stick)
    pub fn from_controller_config(controller_config: &ControllerConfig) -> (Self, Self) {
        let cleaned_cal_points_astick = CleanedCalibrationPoints::from_temp_calibration_points(
            &controller_config.temp_cal_points_ax.to_regular_array(),
            &controller_config.temp_cal_points_ay.to_regular_array(),
            &controller_config.a_angles.to_regular_array(),
        );

        let cleaned_cal_points_cstick = CleanedCalibrationPoints::from_temp_calibration_points(
            &controller_config.temp_cal_points_cx.to_regular_array(),
            &controller_config.temp_cal_points_cy.to_regular_array(),
            &controller_config.c_angles.to_regular_array(),
        );

        todo!()
    }
}

#[derive(Clone, Debug, Format, Copy)]
enum NotchStatus {
    TertInactive,
    TertActive,
    Secondary,
    Cardinal,
}

#[derive(Clone, Debug)]
struct CleanedCalibrationPoints {
    pub cleaned_points_x: [f32; NO_OF_NOTCHES + 1],
    pub cleaned_points_y: [f32; NO_OF_NOTCHES + 1],
    pub notch_points_x: [f32; NO_OF_NOTCHES + 1],
    pub notch_points_y: [f32; NO_OF_NOTCHES + 1],
    pub notch_status: [NotchStatus; NO_OF_NOTCHES],
}

impl Default for CleanedCalibrationPoints {
    fn default() -> Self {
        Self {
            cleaned_points_x: [0f32; NO_OF_NOTCHES + 1],
            cleaned_points_y: [0f32; NO_OF_NOTCHES + 1],
            notch_points_x: [0f32; NO_OF_NOTCHES + 1],
            notch_points_y: [0f32; NO_OF_NOTCHES + 1],
            notch_status: NOTCH_STATUS_DEFAULTS,
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

        debug!("Raw calibration points:");
        for i in 0..NO_OF_CALIBRATION_POINTS {
            debug!("({}, {})", cal_points_x[i], cal_points_y[i])
        }

        debug!("Notch angles: {}", notch_angles);

        for i in 0..NO_OF_NOTCHES {
            // add the origin values to the first x,y point
            out.cleaned_points_x[0] += cal_points_x[i * 2];
            out.cleaned_points_y[0] += cal_points_y[i * 2];

            // copy the cal point into the cleaned list
            out.cleaned_points_x[i + 1] = cal_points_x[i * 2 + 1];
            out.cleaned_points_y[i + 1] = cal_points_y[i * 2 + 1];

            (out.notch_points_x[i + 1], out.notch_points_y[i + 1]) =
                match calc_stick_values(notch_angles[i]) {
                    (a, b) => (roundf(a), roundf(b)),
                };
        }

        // TODO: put the below in a macro to clean it up a bit, once it's confirmed to work
        // remove the largest and smallest two origin values to remove outliers
        // first, find their indices
        let mut i = 0;
        let x_by_size = &mut cal_points_x.map(|e| {
            i += 1;
            (i - 1, e)
        });

        tiny_sort::unstable::sort_by(x_by_size, |a, b| a.1.partial_cmp(&b.1).unwrap());

        let smallest_x = x_by_size[0].0;
        let small_x = x_by_size[1].0;
        let large_x = x_by_size[x_by_size.len() - 2].0;
        let largest_x = x_by_size[x_by_size.len() - 1].0;

        // do the same for y
        let mut i = 0;
        let y_by_size = &mut cal_points_y.map(|e| {
            i += 1;
            (i - 1, e)
        });

        tiny_sort::unstable::sort_by(y_by_size, |a, b| a.1.partial_cmp(&b.1).unwrap());

        let smallest_y = y_by_size[0].0;
        let small_y = y_by_size[1].0;
        let large_y = y_by_size[y_by_size.len() - 2].0;
        let largest_y = y_by_size[y_by_size.len() - 1].0;

        // TODO: make this whole thing a function? it looks very ugly
        out.cleaned_points_x[0] -= cal_points_x[smallest_x];
        out.cleaned_points_x[0] -= cal_points_x[small_x];
        out.cleaned_points_x[0] -= cal_points_x[large_x];
        out.cleaned_points_x[0] -= cal_points_x[largest_x];

        out.cleaned_points_y[0] -= cal_points_y[smallest_y];
        out.cleaned_points_y[0] -= cal_points_y[small_y];
        out.cleaned_points_y[0] -= cal_points_y[large_y];
        out.cleaned_points_y[0] -= cal_points_y[largest_y];

        out.cleaned_points_x[0] /= (NO_OF_NOTCHES - 4) as f32;
        out.cleaned_points_y[0] /= (NO_OF_NOTCHES - 4) as f32;

        for i in 0..NO_OF_NOTCHES {
            let delta_x = out.cleaned_points_x[i + 1] - out.cleaned_points_x[0];
            let delta_y = out.cleaned_points_y[i + 1] - out.cleaned_points_y[0];
            let mag = sqrtf(delta_x * delta_x + delta_y * delta_y);

            // if the cleaned point was at the center and would be a firefox notch
            // average the previous and next points (cardinal & diagonal) for some sanity
            if mag < 0.02 && (i % 2 == 0) {
                let prev_index = ((i + NO_OF_NOTCHES - 1) % NO_OF_NOTCHES) + 1;
                let next_index = ((i + 1) % NO_OF_NOTCHES) + 1;

                out.cleaned_points_x[i + 1] =
                    (out.cleaned_points_x[prev_index] + out.cleaned_points_x[next_index]) / 2.0;
                out.cleaned_points_y[i + 1] =
                    (out.cleaned_points_y[prev_index] + out.cleaned_points_y[next_index]) / 2.0;

                out.notch_points_x[i + 1] =
                    (out.notch_points_x[prev_index] + out.notch_points_x[next_index]) / 2.0;
                out.notch_points_y[i + 1] =
                    (out.notch_points_y[prev_index] + out.notch_points_y[next_index]) / 2.0;

                debug!("Skipping notch {}", i + 1);

                // Mark that notch adjustment should be skipped for this
                out.notch_status[i] = NotchStatus::TertInactive;
            } else {
                out.notch_status[i] = NOTCH_STATUS_DEFAULTS[i];
            }
        }

        debug!("Final points:");
        for i in 0..=NO_OF_NOTCHES {
            debug!(
                "Cleaned: ({}, {}), Notch: ({}, {})",
                out.cleaned_points_x[i],
                out.cleaned_points_y[i],
                out.notch_points_x[i],
                out.notch_points_y[i],
            );
        }

        debug!("The notch statuses are: {:?}", out.notch_status);

        out
    }
}

#[derive(Clone, Debug, Default)]
struct LinearizedCalibration {
    pub fit_coeffs_x: [f64; NUM_COEFFS],
    pub fit_coeffs_y: [f64; NUM_COEFFS],

    pub out_x: [f32; NO_OF_NOTCHES],
    pub out_y: [f32; NO_OF_NOTCHES],
}

impl LinearizedCalibration {
    ///
    /// Generate a fit to linearize the stick response.
    ///
    /// Inputs:
    ///     cleaned points X and Y, (must be 17 points for each of these, the first being the center, the others starting at 3 oclock and going around counterclockwise)
    ///
    ///	Outputs:
    ///		linearization fit coefficients for X and Y
    pub fn from_points(in_x: &[f64; 17], in_y: &[f64; 17]) -> Self {
        let mut fit_points_x = [0f64; 5];
        let mut fit_points_y = [0f64; 5];

        fit_points_x[0] = in_x[8 + 1];
        fit_points_x[1] = (in_x[6 + 1] + in_x[10 + 1]) / 2.0f64;
        fit_points_x[2] = in_x[0];
        fit_points_x[3] = (in_x[2 + 1] + in_x[14 + 1]) / 2.0f64;
        fit_points_x[4] = in_x[0 + 1];

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

        fit_coeffs_x[3] = fit_coeffs_x[3] - x_zero_error as f64;
        fit_coeffs_y[3] = fit_coeffs_y[3] - y_zero_error as f64;

        let mut out_x = [0f32; NO_OF_NOTCHES];
        let mut out_y = [0f32; NO_OF_NOTCHES];

        for i in 0..=NO_OF_NOTCHES {
            out_x[i] = linearize(in_x[i] as f32, &fit_coeffs_x.map(|e| e as f32));
            out_y[i] = linearize(in_y[i] as f32, &fit_coeffs_y.map(|e| e as f32));
        }

        Self {
            fit_coeffs_x,
            fit_coeffs_y,
            out_x,
            out_y,
        }
    }
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
    let sign = trianglize(matrix);

    if sign == 0 {
        return 0.;
    }

    let mut p = 1f64;

    for i in 0..N {
        p *= matrix[i][i];
    }

    p * (sign as f64)
}

/// Trianglize a matrix
fn trianglize<const N: usize>(matrix: &[[f64; N]; N]) -> i32 {
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
            let tmp = matrix[i];
            matrix[i] = matrix[max];
            matrix[max] = tmp;
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
        for j in 0..NCOEFFS * 2 - 1 {
            s[j] += curve_fit_power(x, j as u32);
        }
        for j in 0..NCOEFFS {
            t[j] += y * curve_fit_power(x, j as u32);
        }
    }

    //Master matrix LHS of linear equation
    let mut matrix = [[0f64; NCOEFFS]; NCOEFFS];

    for i in 0..NCOEFFS {
        for j in 0..NCOEFFS {
            matrix[i][j] = s[i + j];
        }
    }

    let denom = det(&matrix);

    for i in 0..NCOEFFS {
        coeffs[NCOEFFS - i - 1] = det(&sub_col(&matrix, &t, i, NCOEFFS)) / denom;
    }

    coeffs
}

/// Compute the stick x/y coordinates from a given angle.
/// The stick moves spherically, so it requires 3D trigonometry.
fn calc_stick_values(angle: f32) -> (f32, f32) {
    let x =
        100. * atan2f(sinf(MAX_STICK_ANGLE) * cosf(angle), cosf(MAX_STICK_ANGLE)) / MAX_STICK_ANGLE;
    let y =
        100. * atan2f(sinf(MAX_STICK_ANGLE) * sinf(angle), cosf(MAX_STICK_ANGLE)) / MAX_STICK_ANGLE;

    (x, y)
}

pub fn linearize(point: f32, coefficients: &[f32; 4]) -> f32 {
    coefficients[0] * (point * point * point)
        + coefficients[1] * (point * point)
        + coefficients[2] * point
        + coefficients[3]
}

pub fn notch_remap(
    x_in: f32,
    y_in: f32,
    stick_params: &StickParams,
    controller_config: &ControllerConfig,
    which_stick: Stick,
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

    let stick_scale = match which_stick {
        Stick::ControlStick => controller_config.astick_analog_scaler as f32 / 100.,
        Stick::CStick => controller_config.cstick_analog_scaler as f32 / 100.,
    };

    let x_out = stick_scale
        * (stick_params.affine_coeffs[region][0] * x_in
            + stick_params.affine_coeffs[region][1] * y_in);
    let y_out = stick_scale
        * (stick_params.affine_coeffs[region][2] * x_in
            + stick_params.affine_coeffs[region][3] * y_in);

    // TODO: here, add calibration step shenanigans

    (x_out, y_out)
}
