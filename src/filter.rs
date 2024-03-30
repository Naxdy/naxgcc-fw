use defmt::Format;
use libm::{fmaxf, fminf, powf};

use crate::{
    config::{ControllerConfig, StickConfig},
    helpers::XyValuePair,
};

macro_rules! run_kalman_on_axis {
    ($self:ident, $axis:ident, $snapback:expr, $vel_weight1:ident, $vel_weight2:ident, $old_pos_filt:ident, $filter_gains:ident, $old_vel_filt:ident, $old_pos_diff:ident, $accel:ident, $vel_smooth:ident, $stick_distance6:ident) => {
        if $snapback > 0 {
            $self.vel_filt.$axis = $vel_weight1 * $self.vel.$axis
                + (1. - $filter_gains.vel_decay.$axis) * $vel_weight2 * $old_vel_filt.$axis
                + $filter_gains.vel_pos_factor.$axis * $old_pos_diff.$axis;

            let pos_weight_vel_acc = 1.
                - fminf(
                    1.,
                    $vel_smooth.$axis * $vel_smooth.$axis * $filter_gains.vel_thresh
                        + $accel.$axis * $accel.$axis * $filter_gains.accel_thresh,
                );
            let pos_weight1 = fmaxf(pos_weight_vel_acc, $stick_distance6);
            let pos_weight2 = 1. - pos_weight1;

            $self.pos_filt.$axis = pos_weight1 * $self.pos.$axis
                + pos_weight2
                    * ($old_pos_filt.$axis
                        + (1. - $filter_gains.vel_damp.$axis) * $self.vel_filt.$axis)
        } else if $snapback < 0 {
            let lpf = $old_pos_filt.$axis * $filter_gains.vel_damp.$axis
                + $self.pos.$axis * (1. - $filter_gains.vel_damp.$axis);

            let pos_weight_vel_acc = 1.
                - fminf(
                    1.,
                    $vel_smooth.$axis * $vel_smooth.$axis * $filter_gains.vel_thresh
                        + $accel.$axis * $accel.$axis * $filter_gains.accel_thresh,
                );
            let pos_weight1 = fmaxf(pos_weight_vel_acc, $stick_distance6);
            let pos_weight2 = 1. - pos_weight1;

            $self.pos_filt.$axis = pos_weight1 * $self.pos.$axis + pos_weight2 * lpf;
        } else {
            $self.pos_filt.$axis = $self.pos.$axis;
        }
    };
}

/// Filter gains for 800Hz, the ones for 1000Hz are provided by `get_norm_gains`
pub const FILTER_GAINS: FilterGains = FilterGains {
    max_stick: 100.,
    vel_decay: XyValuePair { x: 0.1, y: 0.1 },
    vel_pos_factor: XyValuePair { x: 0.01, y: 0.01 },
    vel_damp: XyValuePair { x: 0.125, y: 0.125 },
    vel_thresh: 1.,
    accel_thresh: 3.,
    smoothing: XyValuePair { x: 0.0, y: 0.0 },
    c_smoothing: XyValuePair { x: 0.0, y: 0.0 },
};

#[derive(Debug, Clone, Default)]
pub struct WaveshapingValues {
    pub old_pos: XyValuePair<f32>,
    pub old_vel: XyValuePair<f32>,
    pub old_out: XyValuePair<f32>,
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
    pub vel_decay: XyValuePair<f32>, //0.1 default for 1.2ms timesteps, larger for bigger timesteps
    /// how much the current position disagreement impacts the filtered velocity.
    /// Probably don't touch this.
    pub vel_pos_factor: XyValuePair<f32>, //0.01 default for 1.2ms timesteps, larger for bigger timesteps
    /// how much to ignore filtered velocity when computing the new stick position.
    /// DO CHANGE THIS
    /// Higher gives shorter rise times and slower fall times (more pode, less snapback)
    pub vel_damp: XyValuePair<f32>, //0.125 default for 1.2ms timesteps, smaller for bigger timesteps
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
    pub smoothing: XyValuePair<f32>,
    /// Same thing but for C-stick
    pub c_smoothing: XyValuePair<f32>,
}

impl FilterGains {
    /// Returns filter gains for 1000Hz polling rate
    pub fn get_normalized_gains(&self, controller_config: &ControllerConfig) -> Self {
        let mut gains = self.clone();

        gains.vel_damp.x = vel_damp_from_snapback(controller_config.astick_config.x_snapback);
        gains.vel_damp.y = vel_damp_from_snapback(controller_config.astick_config.y_snapback);

        gains.smoothing.x = controller_config.astick_config.x_smoothing as f32 / 10.;
        gains.smoothing.y = controller_config.astick_config.y_smoothing as f32 / 10.;

        gains.c_smoothing.x = controller_config.cstick_config.x_smoothing as f32 / 10.;
        gains.c_smoothing.y = controller_config.cstick_config.y_smoothing as f32 / 10.;

        // The below is assuming the sticks to be polled at 1000Hz
        let time_factor = 1.0 / 1.2;
        let time_divisor = 1.2 / 1.0;

        let vel_thresh = 1.0 / (gains.vel_thresh * time_factor);
        let accel_thresh = 1.0 / (gains.accel_thresh * time_factor);

        FilterGains {
            max_stick: gains.max_stick * gains.max_stick,
            vel_decay: XyValuePair {
                x: gains.vel_decay.x * time_factor,
                y: gains.vel_decay.y * time_factor,
            },
            vel_pos_factor: XyValuePair {
                x: gains.vel_pos_factor.x * time_factor,
                y: gains.vel_pos_factor.y * time_factor,
            },
            vel_damp: XyValuePair {
                x: gains.vel_damp.x
                    * match controller_config.astick_config.x_snapback {
                        a if a >= 0 => time_factor,
                        _ => 1.0,
                    },
                y: gains.vel_damp.y
                    * match controller_config.astick_config.y_snapback {
                        a if a >= 0 => time_factor,
                        _ => 1.0,
                    },
            },
            vel_thresh,
            accel_thresh,
            smoothing: XyValuePair {
                x: powf(1.0 - gains.smoothing.x, time_divisor),
                y: powf(1.0 - gains.smoothing.y, time_divisor),
            },
            c_smoothing: XyValuePair {
                x: powf(1.0 - gains.c_smoothing.x, time_divisor),
                y: powf(1.0 - gains.c_smoothing.y, time_divisor),
            },
        }
    }
}

#[derive(Clone, Debug, Format, Default)]
pub struct KalmanState {
    pos: XyValuePair<f32>,
    vel: XyValuePair<f32>,
    vel_filt: XyValuePair<f32>,
    pos_filt: XyValuePair<f32>,
}

impl KalmanState {
    // runs kalman filter
    #[link_section = ".time_critical.run_kalman"]
    pub fn run_kalman(
        &mut self,
        x_z: f32,
        y_z: f32,
        stick_config: &StickConfig,
        filter_gains: &FilterGains,
    ) -> (f32, f32) {
        let old_pos = self.pos;
        let old_vel = self.vel;
        let old_vel_filt = self.vel_filt;
        let old_pos_filt = self.pos_filt;

        self.pos.x = x_z;
        self.pos.y = y_z;
        self.vel.x = x_z - old_pos.x;
        self.vel.y = y_z - old_pos.y;

        let vel_smooth = XyValuePair {
            x: 0.5 * (self.vel.x + old_vel.x),
            y: 0.5 * (self.vel.y + old_vel.y),
        };
        let accel = XyValuePair {
            x: self.vel.x - old_vel.x,
            y: self.vel.y - old_vel.y,
        };
        let old_pos_diff = XyValuePair {
            x: old_pos.x - old_pos_filt.x,
            y: old_pos.y - old_pos_filt.y,
        };

        let stick_distance2 = fminf(
            filter_gains.max_stick,
            self.pos.x * self.pos.x + self.pos.y * self.pos.y,
        ) / filter_gains.max_stick;
        let stick_distance6 = stick_distance2 * stick_distance2 * stick_distance2;

        let vel_weight1 = stick_distance2;
        let vel_weight2 = 1. - vel_weight1;

        //modified velocity to feed into our kalman filter.
        //We don't actually want an accurate model of the velocity, we want to suppress snapback without adding delay
        //term 1: weight current velocity according to r^2
        //term 2: the previous filtered velocity, weighted the opposite and also set to decay
        //term 3: a corrective factor based on the disagreement between real and filtered position

        //the current position weight used for the filtered position is whatever is larger of
        //  a) 1 minus the sum of the squares of
        //    1) the smoothed velocity divided by the velocity threshold
        //    2) the acceleration divided by the accel threshold
        //  b) stick r^6
        //When the stick is moving slowly, we want to weight it highly, in order to achieve
        //  quick control for inputs such as tilts. We lock out using both velocity and
        //  acceleration in order to rule out snapback.
        //When the stick is near the rim, we also want instant response, and we know snapback
        //  doesn't reach the rim.

        //In calculating the filtered stick position, we have the following components
        //term 1: current position, weighted according to the above weight
        //term 2: a predicted position based on the filtered velocity and previous filtered position,
        //  with the filtered velocity damped, and the overall term weighted inverse of the previous term
        //term 3: the integral error correction term

        //But if we xSnapback or ySnapback is zero, we skip the calculation
        run_kalman_on_axis!(
            self,
            x,
            stick_config.x_snapback,
            vel_weight1,
            vel_weight2,
            old_pos_filt,
            filter_gains,
            old_vel_filt,
            old_pos_diff,
            accel,
            vel_smooth,
            stick_distance6
        );

        run_kalman_on_axis!(
            self,
            y,
            stick_config.y_snapback,
            vel_weight1,
            vel_weight2,
            old_pos_filt,
            filter_gains,
            old_vel_filt,
            old_pos_diff,
            accel,
            vel_smooth,
            stick_distance6
        );

        self.get_xy()
    }

    pub fn get_xy(&self) -> (f32, f32) {
        (self.pos_filt.x, self.pos_filt.y)
    }
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
#[link_section = ".time_critical.run_waveshaping"]
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

    let x_vel = x_pos - waveshaping_values.old_pos.x;
    let y_vel = y_pos - waveshaping_values.old_pos.y;

    let x_vel_smooth = 0.5 * (x_vel + waveshaping_values.old_vel.x);
    let y_vel_smooth = 0.5 * (y_vel + waveshaping_values.old_vel.y);

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

    let x_out = x_pos * new_x_pos_weight + waveshaping_values.old_out.x * old_x_pos_weight;
    let y_out = y_pos * new_y_pos_weight + waveshaping_values.old_out.y * old_y_pos_weight;

    waveshaping_values.old_pos.x = x_pos;
    waveshaping_values.old_pos.y = y_pos;
    waveshaping_values.old_vel.x = x_vel_smooth;
    waveshaping_values.old_vel.y = y_vel_smooth;
    waveshaping_values.old_out.x = x_out;
    waveshaping_values.old_out.y = y_out;

    (x_out, y_out)
}
