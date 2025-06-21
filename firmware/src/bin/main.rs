#![no_std]
#![no_main]
use {defmt_rtt as _, panic_probe as _};

use common::LogMessage;
use common::external::nalgebra::Matrix1x3;
use common::filter::{pendulum_model, Model, NLModel};
use common::{ControllerMessage, SAMPLE_TIME_MS, filter::EKF};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::I2c;
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker, Timer};
use firmware::encoder::{MagneticEncoder, RotaryEncoder};
use firmware::motor::NidecMotor;
use firmware::server::{GOT_CONNECTION, start_network, transmitter};
use heapless::mpmc::Q4;

use {defmt_rtt as _, panic_probe as _};

static LOG_MSG_QUEUE: Q4<LogMessage> = Q4::new();

fn sub_angles(a: f32, b: f32) -> f32 {
    let mut diff_angle = a - b;
    if diff_angle < -core::f32::consts::PI {
        diff_angle += 2.0 * core::f32::consts::PI;
    } else if diff_angle > core::f32::consts::PI {
        diff_angle -= 2.0 * core::f32::consts::PI;
    }
    diff_angle
}

/// Waits for the wheel to settle down and then reads the encoder.
pub async fn get_reference<T: RotaryEncoder>(encoder: &mut T) -> Result<f32, T::Error> {
    let mut last = encoder.rotation().await?;
    'outer: loop {
        for _ in 0..8 {
            Timer::after_millis(40).await;
            let cur = encoder.rotation().await?;
            if sub_angles(cur, last).abs() > 0.01 {
                last = cur;
                continue 'outer;
            }
        }
        break;
    }
    Ok(last)
}

#[embassy_executor::main]
pub async fn entrypoint(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let dir_pin = Output::new(p.PIN_8, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, Default::default());
    let mut motor1 = NidecMotor::new(dir_pin, motor_pwm);
    motor1.set_output(0.0);

    let dir_pin = Output::new(p.PIN_14, Level::Low);
    let motor_pwm = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, Default::default());
    let mut motor2 = NidecMotor::new(dir_pin, motor_pwm);
    motor2.set_output(0.0);

    let mut motor = motor1;

    // Initialize network server.
    // use firmware::assign::*;
    // use firmware::split_resources;
    // let r = split_resources!(p);
    // let (stack, control) = start_network(r.net, &spawner).await;
    // info!("network started!");
    // unwrap!(spawner.spawn(transmitter(stack, control, &LOG_MSG_QUEUE)));

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let i2c = I2c::new_async(p.I2C0, scl, sda, firmware::Irqs, Default::default());

    let mut encoder = MagneticEncoder { channel: i2c };

    info!("Fetching rotation...");
    let bottom_angle = -1.8149946;
    let ref_angle = sub_angles(bottom_angle, core::f32::consts::PI);

    // info!("Waiting for connection...");
    // while let None = GOT_CONNECTION.dequeue() {
    //     Timer::after_millis(500).await;
    // }
    // info!("Got connection!");
    // Timer::after_millis(500).await;

    // let ts = Duration::from_millis(500 as u64);
    let ts = Duration::from_millis(SAMPLE_TIME_MS as u64);
    let mut ticker = Ticker::every(ts);

    let mut ekf = EKF::from_model(NLModel {dt: SAMPLE_TIME_MS as f32 *1e-3});

    info!("Entering control loop!");
    loop {
        ticker.next().await;

        if let Ok(raw_angle) = encoder.rotation().await {
            let angle = sub_angles(raw_angle, ref_angle);
            let pred = ekf.x[1];            
            ekf.measurment_update_from_error([sub_angles(angle, pred)].into());
        } else {
            warn!("Failed to read from encoder");
        }
        let y = ekf.model.h(ekf.x);

        let f: Matrix1x3<f32> = [[-0.00582551],[-8.00347],[-0.967164]].into();
        let u = (-f * ekf.x)[0];
        motor.set_output(u.clamp(-1.0, 1.0));
        info!("u = {}", u);

        ekf.time_update(motor.output);


        let msg = ControllerMessage {
            time_ms: Instant::now().as_millis(),
            wheel_velocity: y[0],
            control: motor.output,
            pend_angle: y[0],
            pend_velocity: y[0],
            sensor_pend_angle: 0.0,
            sensor_wheel_velocity: 0.0,
        };

        let _ = LOG_MSG_QUEUE.enqueue(LogMessage::Controller(msg));
    }
}
