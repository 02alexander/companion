#![no_std]
#![no_main]

use core::f32::consts::PI;

use common::dp::{cont_u, disc_state};
use common::filter::{pendulum_model, EKF};
use embassy_rp::i2c::I2c;
use firmware::dptable::DP_TABLE;
use firmware::encoder::{MagneticEncoder, RotaryEncoder};
use firmware::motor::NidecMotor;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker};

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

    let dir_pin = Output::new(p.PIN_8, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, Default::default());
    let mut motor = NidecMotor::new(dir_pin, motor_pwm);
    motor.set_output(0.0);

    let _enable_pin = Output::new(p.PIN_11, Level::High);

    info!("Entering loop...");
    let mut ticker = Ticker::every(Duration::from_millis(10));

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let i2c = I2c::new_async(
        p.I2C0,
        scl,
        sda,
        firmware::Irqs,
        Default::default(),
    );
    let mut encoder = MagneticEncoder { channel: i2c };
    let ref_angle = -1.8149946;
    // let ref_angle = encoder.rotation().await.unwrap();
    info!("ref_angle = {}", ref_angle);

    let mut prev_angles = [0.0; 2000];

    let mut ekf = EKF::from_model(pendulum_model());
    
    loop {

        ekf.time_update(motor.output);

        if let Ok(raw_angle) = encoder.rotation().await {
            let angle = sub_angles(raw_angle, ref_angle);
            ekf.measurment_update([0.0, angle, 0.0].into());
        }

        let state = ekf.model.C * ekf.x;

        let dstate = disc_state(state);
        let u = cont_u(DP_TABLE[dstate[0]][dstate[1]][dstate[2]] as usize);
        motor.set_output(u);
        info!("u = {}", u);
        

        // let t = Instant::now().as_micros() as f32*1e-6;
        // motor.set_output(libm::cosf(t*2.0*PI)*0.2);

        ticker.next().await;
    }
}
