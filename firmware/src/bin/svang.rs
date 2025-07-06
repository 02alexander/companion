#![no_std]
#![no_main]

use core::f32::consts::PI;

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::I2c;
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker};
use firmware::encoder::{MagneticEncoder, RotaryEncoder};
use firmware::motor::NidecMotor;

use {defmt_rtt as _, panic_probe as _};

fn sub_angles(a: f32, b: f32) -> f32 {
    let mut diff_angle = a - b;
    if diff_angle < -core::f32::consts::PI {
        diff_angle += 2.0 * core::f32::consts::PI;
    } else if diff_angle > core::f32::consts::PI {
        diff_angle -= 2.0 * core::f32::consts::PI;
    }
    diff_angle
}

#[embassy_executor::main]
pub async fn entrypoint(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let dir_pin = Output::new(p.PIN_12, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE5, p.PIN_11, Default::default());
    let mut motor = NidecMotor::new(dir_pin, motor_pwm);
    motor.set_output(0.0);

    let mut ticker = Ticker::every(Duration::from_millis(10));

    let sda = p.PIN_0;
    let scl = p.PIN_1;
    let i2c = I2c::new_async(p.I2C0, scl, sda, firmware::Irqs, Default::default());
    let mut encoder = MagneticEncoder { channel: i2c };
    info!("Reading from encoder...");
    let ref_angle = encoder.rotation().await.unwrap();
    info!("ref_angle = {}", ref_angle);

    let mut prev_angles = [0.0; 2000];

    loop {
        if let Ok(raw_angle) = encoder.rotation().await {
            let angle = sub_angles(raw_angle, ref_angle);

            prev_angles.rotate_right(1);
            prev_angles[0] = angle;
            let sm: f32 = prev_angles.iter().sum();
            info!(
                "{} {}",
                ref_angle,
                ref_angle + sm / prev_angles.len() as f32
            );
        } else {
            warn!("Error readin from encoder!");
        }

        let t = Instant::now().as_micros() as f32 * 1e-6;
        motor.set_output(libm::cosf(t * 2.0 * PI) * 0.2);

        ticker.next().await;
    }
}
