#![no_std]

use serde::{Deserialize, Serialize};

pub const SAMPLE_TIME_MS: u32 = 10;
mod ekf;
mod model;
pub mod dp;
pub mod filter {
    pub use crate::ekf::*;
    pub use crate::model::*;
}

pub mod external {
    pub use nalgebra;
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum LogMessage {
    Controller(ControllerMessage),
    Bench(BenchMessage),
    Alive,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct BenchMessage {
    pub time_ms: u64,
    pub control: f32,
    pub signed_rot_speed: f32,
    pub abs_rot_speed: f32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControllerMessage {
    pub time_ms: u64,
    pub control: f32,
    pub sensor_pend_angle: f32,
    pub sensor_wheel_velocity: f32,
    pub pend_angle: f32,
    pub pend_velocity: f32,
    pub wheel_velocity: f32,
}
