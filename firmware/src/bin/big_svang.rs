#![no_std]
#![no_main]

use core::f32::consts::PI;

use common::dp::{cont_u, Agent};
use common::filter::{pendulum_model, Model, NLModel, EKF};
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
    while diff_angle < -core::f32::consts::PI {
        diff_angle += 2.0 * core::f32::consts::PI;
    } 
    while diff_angle > core::f32::consts::PI {
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
    let ref_angle = sub_angles(-1.8149946, PI);
    // let ref_angle = encoder.rotation().await.unwrap();
    info!("ref_angle = {}", ref_angle);

    let mut prev_angles = [0.0; 2000];

    let mut ekf = EKF::from_model(NLModel {dt: 0.01});
    
    let mut agent = Agent::new();
    agent.parameters = [-0.040831022,0.0051453677,0.23291656,-1.8110611,0.58743167,].into();

    loop {

        ekf.time_update(1.0);

        if let Ok(raw_angle) = encoder.rotation().await {
            let angle = sub_angles(raw_angle, ref_angle);

            let pred = ekf.x[1];
            
            ekf.measurment_update_from_error([sub_angles(angle, pred)].into());

        }

        info!("{} {} {}", ekf.x[0], sub_angles(ekf.x[1], 0.0), ekf.x[2]);
        let u = cont_u(agent.best_action(&ekf.x));
        motor.set_output(u);
        info!("u = {}", u);

        ticker.next().await;
    }
}
