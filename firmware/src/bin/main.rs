#![no_std]
#![no_main]
use {defmt_rtt as _, panic_probe as _};

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

    // let encoder_pin = Input::new(p.PIN_12, Pull::Down);
    // let pwm_pin = p.PIN_15;
    // let pwm_slice = p.PWM_SLICE7;
    // let direction_pin = Output::new(p.PIN_14, Level::Low);

    let _other_pwm_pin = Output::new(p.PIN_15, Level::High);
    let _encoder_pin = Input::new(p.PIN_9, Pull::Down);

    let pwm_pin = p.PIN_7;
    let pwm_slice = p.PWM_SLICE3;
    let direction_pin = Output::new(p.PIN_8, Level::Low);

    let motor_pwm = Pwm::new_output_b(pwm_slice, pwm_pin, Default::default());
    let mut motor = NidecMotor::new(direction_pin, motor_pwm);
    let _enable_pin = Output::new(p.PIN_11, Level::High);
    motor.set_output(0.0);

    // Initialize network server.
    use firmware::assign::*;
    use firmware::split_resources;
    let r = split_resources!(p);
    let (stack, control) = start_network(r.net, &spawner).await;
    info!("network started!");
    unwrap!(spawner.spawn(transmitter(stack, control, &LOG_MSG_QUEUE)));

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

    info!("Fetching rotation...");
    let ref_angle = get_reference(&mut encoder).await.unwrap();
    info!("ref_angle = {}", ref_angle);
    info!("Fetched rotation!");
    let ref_angle = -2.147573 - 0.0055 - 0.017;
    let ref_angle = sub_angles(ref_angle, core::f32::consts::PI);

    info!("Waiting for connection...");
    while let None = GOT_CONNECTION.dequeue() {
        Timer::after_millis(500).await;
    }
    info!("Got connection!");
    Timer::after_millis(500).await;

    let ts = Duration::from_millis(SAMPLE_TIME_MS as u64);
    let mut ticker = Ticker::every(ts);

    let mut ekf = EKF::from_model(pendulum_model());

    info!("Entering control loop!");
    loop {
        let Ok(cur_angle) = encoder.rotation().await else {
            info!("Failed to read from encoder");
            continue;
        };
        let diff_angle = sub_angles(cur_angle, ref_angle);
        info!("angles {} - {} = {}", cur_angle, ref_angle, diff_angle);
        let y = ekf.model.h(ekf.x);

        #[allow(clippy::ercessive_precision)]
        let f: Matrix1x3<f32> = [
            [-0.005822502577461619],
            [-8.180109598430773],
            [-0.976298125919451],
        ]
        .into();
        let u = (-f * y)[0];
        motor.set_output(u.clamp(-1.0, 1.0));
        info!("u = {}", u);
        
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
            sensor_wheel_velocity: 0.0,
        };

        let _ = LOG_MSG_QUEUE.enqueue(LogMessage::Controller(msg));

        ticker.next().await;
    }
}
