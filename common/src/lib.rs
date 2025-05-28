#![no_std]

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct LogMessage {
    pub ticks: u32,
    pub pwm: u32,
}
