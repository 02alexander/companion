use crate::encoder::{MagneticEncoder, RotaryEncoder};
use crate::motor::NidecMotor;
use crate::server::{GOT_CONNECTION, start_network, transmitter};
use crate::*;
use common::LogMessage;
use common::external::nalgebra::Matrix1x3;
use common::filter::{Model, pendulum_model};
use common::{ControllerMessage, SAMPLE_TIME_MS, filter::EKF};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::I2c;
use embassy_rp::pwm::Pwm;
use embassy_time::{Duration, Instant, Ticker, Timer};
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

pub async fn entrypoint(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // let encoder_pin = Input::new(p.PIN_12, Pull::Down);
    // let pwm_pin = p.PIN_15;
    // let pwm_slice = p.PWM_SLICE7;
    // let direction_pin = Output::new(p.PIN_14, Level::Low);

    let other_pwm_pin = Output::new(p.PIN_15, Level::High);
    let encoder_pin = Input::new(p.PIN_9, Pull::Down);
    let pwm_pin = p.PIN_7;
    let pwm_slice = p.PWM_SLICE3;
    let direction_pin = Output::new(p.PIN_8, Level::Low);

    let motor_pwm = Pwm::new_output_b(pwm_slice, pwm_pin, Default::default());
    let mut motor = NidecMotor::new(direction_pin, motor_pwm);
    let _enable_pin = Output::new(p.PIN_11, Level::High);
    motor.set_output(0.0);

    // Initialize network server.
    // use crate::assign::*;
    // let r = split_resources!(p);
    // let (stack, control) = start_network(r.net, &spawner).await;
    // info!("network started!");
    // unwrap!(spawner.spawn(transmitter(stack, control, &LOG_MSG_QUEUE)));

    let sda = p.PIN_16;
    let scl = p.PIN_17;
    let i2c = I2c::new_async(
        p.I2C0,
        scl,
        sda,
        crate::entrypoints::Irqs,
        Default::default(),
    );

    let mut encoder = MagneticEncoder { channel: i2c };

    let ts = Duration::from_millis(SAMPLE_TIME_MS as u64);
    let mut ticker = Ticker::every(ts);

    ticker.next().await;

    info!("Fetching rotation...");
    let ref_angle = get_reference(&mut encoder).await.unwrap();
    info!("ref_angle = {}", ref_angle);
    info!("Fetched rotation!");
    let ref_angle = -2.147573 - 0.0055 - 0.017;
    let ref_angle = sub_angles(ref_angle, core::f32::consts::PI);

    // info!("Waiting for connection...");
    // while let None = GOT_CONNECTION.dequeue() {
    //     Timer::after_millis(500).await;
    // }
    // info!("Got connection!");
    // Timer::after_millis(2000).await;

    let mut ekf = EKF::from_model(pendulum_model());

    let mut pos_step = true;
    let mut last_step = Instant::now();

    info!("Entering control loop!");
    loop {
        let Ok(cur_angle) = encoder.rotation().await else {
            info!("Failed to read from encoder");
            continue;
        };
        let diff_angle = sub_angles(cur_angle, ref_angle);
        info!("angles {} - {} = {}", cur_angle, ref_angle, diff_angle);
        let y = ekf.model.h(ekf.x);
        let F: Matrix1x3<f32> = [
            [-0.005822502577461619],
            [-8.180109598430773],
            [-0.976298125919451],
        ]
        .into();
        let u = (-F * y)[0];
        motor.set_output(u.clamp(-1.0, 1.0));
        info!("u = {}", u);

        // if Instant::now() - last_step > Duration::from_millis(1000) {
        //     if pos_step {
        //         motor.set_output(0.1);
        //     } else {
        //         motor.set_output(-0.1);
        //     }
        //     info!("Setting to {}", motor.output);
        //     pos_step = !pos_step;
        //     last_step = Instant::now();
        // }

        // motor.set_output(0.0);
        // info!("{} {}", u, motor.output);

        ekf.time_update(motor.output);

        ekf.measurment_update([0.0, diff_angle, 0.0].into())
            .unwrap();

        let msg = ControllerMessage {
            time_ms: Instant::now().as_millis(),
            wheel_velocity: y[0],
            control: motor.output,
            pend_angle: y[1],
            pend_velocity: y[2],
            sensor_pend_angle: diff_angle,
            sensor_wheel_velocity: motor.ticks() as f32,
        };

        let _ = LOG_MSG_QUEUE.enqueue(LogMessage::Controller(msg));

        ticker.next().await;
    }
}
