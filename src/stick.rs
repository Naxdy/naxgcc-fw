// vast majority of this is taken from Phob firmware

use core::{f32::consts::PI, iter::Filter};

use defmt::Format;
use libm::{atan2f, fabs, powf};

use crate::{
    input::{ControllerConfig, Stick},
    stick,
};

/// fit order for the linearization
const FIT_ORDER: usize = 3;
const NUM_COEFFS: usize = FIT_ORDER + 1;
const NO_OF_NOTCHES: usize = 16;
const MAX_ORDER: usize = 20;

/// Filter gains for 800Hz, the ones for 1000Hz are provided by `get_norm_gains`
pub const FILTER_GAINS: FilterGains = FilterGains {
    max_stick: 100.,
    x_vel_decay: 0.1,
    y_vel_decay: 0.1,
    x_vel_pos_factor: 0.01,
    y_vel_pos_factor: 0.01,
    x_vel_damp: 0.125,
    y_vel_damp: 0.125,
    vel_thresh: 1.,
    accel_thresh: 3.,
    x_smoothing: 0.0,
    y_smoothing: 0.0,
    c_xsmoothing: 0.0,
    c_ysmoothing: 0.0,
};

#[derive(Clone, Debug, Default, Format)]
pub struct StickParams {
    // these are the linearization coefficients
    pub fit_coeffs_x: [f32; NUM_COEFFS],
    pub fit_coeffs_y: [f32; NUM_COEFFS],

    // these are the notch remap parameters
    pub affine_coeffs: [[f32; 16]; 4], // affine transformation coefficients for all regions of the stick
    pub boundary_angles: [f32; 4], // angles at the boundaries between regions of the stick (in the plane)
}

#[derive(Clone, Debug, Default, Format)]
pub struct FilterGains {
    /// What's the max stick distance from the center
    pub max_stick: f32,
    /// filtered velocity terms
    /// how fast the filtered velocity falls off in the absence of stick movement.
    /// Probably don't touch this.
    pub x_vel_decay: f32, //0.1 default for 1.2ms timesteps, larger for bigger timesteps
    pub y_vel_decay: f32,
    /// how much the current position disagreement impacts the filtered velocity.
    /// Probably don't touch this.
    pub x_vel_pos_factor: f32, //0.01 default for 1.2ms timesteps, larger for bigger timesteps
    pub y_vel_pos_factor: f32,
    /// how much to ignore filtered velocity when computing the new stick position.
    /// DO CHANGE THIS
    /// Higher gives shorter rise times and slower fall times (more pode, less snapback)
    pub x_vel_damp: f32, //0.125 default for 1.2ms timesteps, smaller for bigger timesteps
    pub y_vel_damp: f32,
    /// speed and accel thresholds below which we try to follow the stick better
    /// These may need tweaking according to how noisy the signal is
    /// If it's noisier, we may need to add additional filtering
    /// If the timesteps are *really small* then it may need to be increased to get
    ///   above the noise floor. Or some combination of filtering and playing with
    ///   the thresholds.
    pub vel_thresh: f32, //1 default for 1.2ms timesteps, larger for bigger timesteps
    pub accel_thresh: f32, //5 default for 1.2ms timesteps, larger for bigger timesteps
    /// This just applies a low-pass filter.
    /// The purpose is to provide delay for single-axis ledgedashes.
    /// Must be between 0 and 1. Larger = more smoothing and delay.
    pub x_smoothing: f32,
    pub y_smoothing: f32,
    /// Same thing but for C-stick
    pub c_xsmoothing: f32,
    pub c_ysmoothing: f32,
}

#[derive(Clone, Debug, Default)]
struct LinearizeCalibrationOutput {
    pub fit_coeffs_x: [f64; NUM_COEFFS],
    pub fit_coeffs_y: [f64; NUM_COEFFS],

    pub out_x: [f32; NO_OF_NOTCHES],
    pub out_y: [f32; NO_OF_NOTCHES],
}

pub fn run_kalman(
    x_z: f32,
    y_z: f32,
    controller_config: &ControllerConfig,
    filter_gains: &FilterGains,
) -> (f32, f32) {
    todo!()
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

pub fn linearize(point: f32, coefficients: &[f32; 4]) -> f32 {
    coefficients[0] * (point * point * point)
        + coefficients[1] * (point * point)
        + coefficients[2] * point
        + coefficients[3]
}

///
/// Generate a fit to linearize the stick response.
///
/// Inputs:
///     cleaned points X and Y, (must be 17 points for each of these, the first being the center, the others starting at 3 oclock and going around counterclockwise)
///
///	Outputs:
///		linearization fit coefficients for X and Y
pub fn linearize_calibration(in_x: &[f64; 17], in_y: &[f64; 17]) -> LinearizeCalibrationOutput {
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

    let mut fit_coeffs_x = fit_curve::<5, NUM_COEFFS>(FIT_ORDER as i32, &fit_points_x, &x_output);
    let mut fit_coeffs_y = fit_curve::<5, NUM_COEFFS>(FIT_ORDER as i32, &fit_points_y, &y_output);

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

    LinearizeCalibrationOutput {
        fit_coeffs_x,
        fit_coeffs_y,
        out_x,
        out_y,
    }
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

fn vel_damp_from_snapback(snapback: i8) -> f32 {
    match snapback {
        a if a >= 0 => 0.125 * powf(2., (snapback - 4) as f32 / 3.0),
        _ => 1. - 0.25 * powf(2., (snapback + 4) as f32 / 3.0),
    }
}

/// Returns filter gains for 1000Hz polling rate
pub fn get_norm_gains(controller_config: &ControllerConfig) -> FilterGains {
    let mut gains = FILTER_GAINS.clone();

    gains.x_vel_damp = vel_damp_from_snapback(controller_config.x_snapback);
    gains.y_vel_damp = vel_damp_from_snapback(controller_config.y_snapback);

    gains.x_smoothing = controller_config.x_smoothing as f32 / 10.;
    gains.y_smoothing = controller_config.y_smoothing as f32 / 10.;

    gains.c_xsmoothing = controller_config.c_xsmoothing as f32 / 10.;
    gains.c_ysmoothing = controller_config.c_ysmoothing as f32 / 10.;

    // The below is assuming the sticks to be polled at 1000Hz
    let time_factor = 1.0 / 1.2;
    let time_divisor = 1.2 / 1.0;

    let vel_thresh = 1.0 / (gains.vel_thresh * time_factor);
    let accel_thresh = 1.0 / (gains.accel_thresh * time_factor);

    FilterGains {
        max_stick: gains.max_stick * gains.max_stick,
        x_vel_decay: gains.x_vel_decay * time_factor,
        y_vel_decay: gains.y_vel_decay * time_factor,
        x_vel_pos_factor: gains.x_vel_pos_factor * time_factor,
        y_vel_pos_factor: gains.y_vel_pos_factor * time_factor,
        x_vel_damp: gains.x_vel_damp
            * match controller_config.x_snapback {
                a if a >= 0 => time_factor,
                _ => 1.0,
            },
        y_vel_damp: gains.y_vel_damp
            * match controller_config.y_snapback {
                a if a >= 0 => time_factor,
                _ => 1.0,
            },
        vel_thresh,
        accel_thresh,
        x_smoothing: powf(1.0 - gains.x_smoothing, time_divisor),
        y_smoothing: powf(1.0 - gains.y_smoothing, time_divisor),
        c_xsmoothing: powf(1.0 - gains.c_xsmoothing, time_divisor),
        c_ysmoothing: powf(1.0 - gains.c_ysmoothing, time_divisor),
    }
}
