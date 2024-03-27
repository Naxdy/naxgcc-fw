use libm::fminf;

use crate::{
    input::{ControllerConfig, Stick},
    stick::FilterGains,
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
    which_stick: Stick,
    waveshaping_values: &mut WaveshapingValues,
    controller_config: &ControllerConfig,
    filter_gains: &FilterGains,
) -> (f32, f32) {
    let x_vel = x_pos - waveshaping_values.old_x_pos;
    let y_vel = y_pos - waveshaping_values.old_y_pos;

    let x_vel_smooth = 0.5 * (x_vel + waveshaping_values.old_x_vel);
    let y_vel_smooth = 0.5 * (y_vel + waveshaping_values.old_y_vel);

    let x_factor = calc_waveshaping_mult(match which_stick {
        Stick::ControlStick => controller_config.ax_waveshaping,
        Stick::CStick => controller_config.cx_waveshaping,
    });
    let y_factor = calc_waveshaping_mult(match which_stick {
        Stick::ControlStick => controller_config.ay_waveshaping,
        Stick::CStick => controller_config.cy_waveshaping,
    });

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
