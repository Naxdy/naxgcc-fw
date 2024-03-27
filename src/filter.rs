use defmt::Format;
use libm::{fminf, powf};

use crate::input::{ControllerConfig, Stick};

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

#[derive(Debug, Clone, Default)]
pub struct WaveshapingValues {
    pub old_x_pos: f32,
    pub old_y_pos: f32,
    pub old_x_vel: f32,
    pub old_y_vel: f32,
    pub old_x_out: f32,
    pub old_y_out: f32,
}

fn calc_waveshaping_mult(setting: u8) -> f32 {
    if setting > 0 && setting <= 5 {
        1. / (440. - 40. * setting as f32)
    } else if setting > 5 && setting <= 15 {
        1. / (340. - 20. * setting as f32)
    } else {
        0.
    }
}

fn vel_damp_from_snapback(snapback: i8) -> f32 {
    match snapback {
        a if a >= 0 => 0.125 * powf(2., (snapback - 4) as f32 / 3.0),
        _ => 1. - 0.25 * powf(2., (snapback + 4) as f32 / 3.0),
    }
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

impl FilterGains {
    /// Returns filter gains for 1000Hz polling rate
    pub fn normalize_gains(&self, controller_config: &ControllerConfig) -> Self {
        let mut gains = self.clone();

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
}

pub fn run_kalman(
    x_z: f32,
    y_z: f32,
    controller_config: &ControllerConfig,
    filter_gains: &FilterGains,
) -> (f32, f32) {
    todo!()
}

/// This simulates an idealized sort of pode:
///
///  if the stick is moving fast, it responds poorly, while
///  if the stick is moving slowly, it follows closely.
///
/// It's not suitable to be the sole filter, but when put after
///  the smart snapback filter, it should be able to hold the
///  output at the rim longer when released.
///
/// Output is a tuple of the x and y positions.
pub fn run_waveshaping(
    x_pos: f32,
    y_pos: f32,
    x_waveshaping: u8,
    y_waveshaping: u8,
    waveshaping_values: &mut WaveshapingValues,
    filter_gains: &FilterGains,
) -> (f32, f32) {
    let x_factor = calc_waveshaping_mult(x_waveshaping);
    let y_factor = calc_waveshaping_mult(y_waveshaping);

    let x_vel = x_pos - waveshaping_values.old_x_pos;
    let y_vel = y_pos - waveshaping_values.old_y_pos;

    let x_vel_smooth = 0.5 * (x_vel + waveshaping_values.old_x_vel);
    let y_vel_smooth = 0.5 * (y_vel + waveshaping_values.old_y_vel);

    let old_x_pos_weight = fminf(
        1.,
        x_vel_smooth * x_vel_smooth * filter_gains.vel_thresh * x_factor,
    );
    let new_x_pos_weight = 1. - old_x_pos_weight;
    let old_y_pos_weight = fminf(
        1.,
        y_vel_smooth * y_vel_smooth * filter_gains.vel_thresh * y_factor,
    );
    let new_y_pos_weight = 1. - old_y_pos_weight;

    let x_out = x_pos * new_x_pos_weight + waveshaping_values.old_x_out * old_x_pos_weight;
    let y_out = y_pos * new_y_pos_weight + waveshaping_values.old_y_out * old_y_pos_weight;

    waveshaping_values.old_x_pos = x_pos;
    waveshaping_values.old_y_pos = y_pos;
    waveshaping_values.old_x_vel = x_vel_smooth;
    waveshaping_values.old_y_vel = y_vel_smooth;
    waveshaping_values.old_x_out = x_out;
    waveshaping_values.old_y_out = y_out;

    (x_out, y_out)
}
