use embassy_executor::Spawner;
use embassy_rp::i2c::{self, I2c};
use embassy_time::Timer;
use defmt::{error, info};

use crate::{encoder::{MagneticEncoder, RotaryEncoder}, entrypoints::Irqs};

pub async fn entrypoint(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let i2c = I2c::new_async(p.I2C0, scl, sda, Irqs, i2c::Config::default());
    let mut encoder = MagneticEncoder {
        channel: i2c,
    };

    loop {
        match encoder.rotation().await {
            Ok(rotation) => {
                info!("Rotation = {}", rotation);
            },
            Err(e) => {
                error!("Failed to read: {:?}", e);
            }
        }
        Timer::after_millis(100).await;
    }
}


